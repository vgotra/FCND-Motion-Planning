import argparse
import time
import msgpack
import numpy as np
import numpy.linalg as la

from enum import Enum, auto
from path_helpers import closest_point, prune_waypoints, find_start_goal, are_close
from planning_utils import a_star, heuristic, create_grid, create_grid_and_edges, a_star_graph, a_star_skeleton
from udacidrone import Drone, global_to_local, local_to_global
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from plotting import plot_map_graph, plot_map_skeleton, plot_map
from skimage.morphology import medial_axis
from skimage.util import invert

import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx


class States(Enum):
    """
    Additional complexity because of some complex API for simulator (go to home position, etc.)
    With easy and simple API we should be able to plan before takeoff
    """
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    TAKEOFF_TO_SAFE_ALTITUDE = auto()
    ON_SAFE_ALTITUDE = auto()
    ARRIVING_TO_HOME_POSITION = auto()
    ARRIVED_TO_HOME_POSITION = auto()
    TAKEOFF_TO_DEF_ALTITUDE = auto()
    ON_DEF_ALTITUDE = auto()
    PLANNING = auto()
    WAYPOINT_ARRIVING = auto()
    WAYPOINT_ARRIVED = auto()
    WAYPOINT_STABILIZATION = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()


class PathAlgorithm(Enum):
    Simple = 0
    Skeleton = 1
    Graphs = 2

class MotionPlanning(Drone):

    def __init__(self, connection, algorithm, lat, lon, save_plot_maps):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.expected_local_position = ()
        self.in_mission = True
        self.default_altitude = 5
        self.safety_distance = 2

        self.goal_lat = lat
        self.goal_lon = lon
        self.algorithm = algorithm

        self.plot_maps = save_plot_maps

        # for some reason global home can be changed - and this can cause some problem for grid and relative ne
        # positions
        self.takeoff_global_home = []

        self.data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        # get max altitude to fly to home position safely :) Frank Sinatra - "Fly Me To The Moon" :) higher - better :)
        self.safe_altitude = int(np.max(self.data[:, 2]) + 115)

        with open('colliders.csv') as fp:
            global_home_line = fp.readline().strip()
        coord_array = global_home_line.split(", ")
        self.start_lat = float(coord_array[0].split(" ")[1])
        self.start_lon = float(coord_array[1].split(" ")[1])

        # initial state
        self.flight_state = States.MANUAL

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF_TO_SAFE_ALTITUDE:
            altitude = abs(self.local_position[2])
            if altitude >= self.safe_altitude:
                print('arrived to safe altitude: ', altitude)
                self.flight_state = States.ON_SAFE_ALTITUDE
        if self.flight_state == States.TAKEOFF_TO_DEF_ALTITUDE:
            # can be a problem if we have a home at roof and don't have map of obstacles :)
            # a lot of cases to think ^)
            altitude = abs(self.local_position[2])
            if altitude <= self.default_altitude:
                print('arrived to default altitude: ', altitude)
                self.flight_state = States.ON_DEF_ALTITUDE
        if self.flight_state == States.WAYPOINT_ARRIVING:
            completed = are_close(self.local_position, self.expected_local_position[:3], [.13, .13, .13])
            if completed.all():
                print("waypoint arrived")
                self.flight_state = States.WAYPOINT_ARRIVED

    def velocity_callback(self):
        if self.flight_state == States.ARRIVING_TO_HOME_POSITION:
            home_global_position = [self.start_lon, self.start_lat, self.safe_altitude]
            completed = are_close(self.global_position, home_global_position, [.00013, .00013, .13])
            if completed.all():
                print("arrived to home global position: ", self.global_position)
                self.flight_state = States.ARRIVED_TO_HOME_POSITION
        if self.flight_state == States.WAYPOINT_ARRIVED:
            # velocity can be changed there to flight correctly
            completed = are_close(self.local_velocity, [0, 0, 0], [.13, .13, .13])
            # if velocity is stable - check for another waypoint
            if completed.all():
                print("waypoint stabilization")
                self.flight_state = States.WAYPOINT_STABILIZATION

    def state_callback(self):
        if not self.in_mission:
            return
        elif self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.TAKEOFF:
            self.takeoff_to_safe_altitude()
        elif self.flight_state == States.TAKEOFF_TO_SAFE_ALTITUDE:
            pass  # logic in position callback
        elif self.flight_state == States.ON_SAFE_ALTITUDE:
            self.go_to_home_position()
        elif self.flight_state == States.ARRIVING_TO_HOME_POSITION:
            pass  # logic in position callback
        elif self.flight_state == States.ARRIVED_TO_HOME_POSITION:
            self.takeoff_to_def_altitude()  # Do not die on impact :)))
        elif self.flight_state == States.TAKEOFF_TO_DEF_ALTITUDE:
            pass # logic in position callback
        elif self.flight_state == States.ON_DEF_ALTITUDE:
            self.plan_path()
        elif self.flight_state == States.PLANNING:
            self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT_ARRIVING:
            pass  # logic in position callback
        elif self.flight_state == States.WAYPOINT_ARRIVED:
            pass  # logic in velocity callback
        elif self.flight_state == States.WAYPOINT_STABILIZATION:
            self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            self.landing_transition()
        elif self.flight_state == States.LANDING:
            self.disarming_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def plan_path(self):
        print("searching for a path ...")
        print("goal lon = {0}, lat = {1}".format(self.goal_lon, self.goal_lat))
        print("algorithm = {0}".format(self.algorithm))

        path = []
        north_offset = 0
        east_offset = 0

        print('global home: {0}, position: {1}, local position: {2}'
              .format(self.global_home, self.global_position, self.local_position))

        if self.algorithm == PathAlgorithm.Simple:
            grid, north_offset, east_offset = create_grid(self.data, self.default_altitude, self.safety_distance)
            print("north offset = {0}, east offset = {1}".format(north_offset, east_offset))

            # Define starting point on the grid (this is just grid center)
            # Set goal as some arbitrary position on the grid
            grid_start = self.to_grid_ne(self.start_lon, self.start_lat, north_offset, east_offset)
            grid_goal = self.to_grid_ne(self.goal_lon, self.goal_lat, north_offset, east_offset)
            print("grid start = {0}, goal = {1}".format(grid_start, grid_goal))

            path, _ = a_star(grid, heuristic, grid_start, grid_goal)
            print("count of waypoints: ", len(path))
            path = prune_waypoints(path)
            print("count of waypoints after prune: ", len(path))

            if self.plot_maps:
                plot_map(grid, path, grid_start, grid_goal)

        elif self.algorithm == PathAlgorithm.Skeleton:
            grid, north_offset, east_offset = create_grid(self.data, self.default_altitude, self.safety_distance)
            print("north offset = {0}, east offset = {1}".format(north_offset, east_offset))

            # Define starting point on the grid (this is just grid center)
            # Set goal as some arbitrary position on the grid
            grid_start = self.to_grid_ne(self.start_lon, self.start_lat, north_offset, east_offset)
            grid_goal = self.to_grid_ne(self.goal_lon, self.goal_lat, north_offset, east_offset)
            print("grid start = {0}, goal = {1}".format(grid_start, grid_goal))

            skeleton = medial_axis(invert(grid))
            skeleton_start, skeleton_goal = find_start_goal(skeleton, grid_start, grid_goal)

            print("skeleton start = {0}, goal = {1}".format(skeleton_start, skeleton_goal))

            path, _ = a_star_skeleton(invert(skeleton).astype(np.int), heuristic, tuple(skeleton_start),
                                      tuple(skeleton_goal))
            print("count of waypoints: ", len(path))
            path = prune_waypoints(path)
            print("count of waypoints after prune: ", len(path))

            if self.plot_maps:
                plot_map_skeleton(grid, skeleton, path, grid_start, grid_goal)

        elif self.algorithm == PathAlgorithm.Graphs:
            grid, edges, north_offset, east_offset = create_grid_and_edges(self.data, self.default_altitude,
                                                                           self.safety_distance)
            print("north offset = {0}, east offset = {1}".format(north_offset, east_offset))

            start_ne = self.to_grid_ne(self.start_lon, self.start_lat, north_offset, east_offset)
            goal_ne = self.to_grid_ne(self.goal_lon, self.goal_lat, north_offset, east_offset)
            print("grid start_ne = {0}, goal_ne = {1}".format(start_ne, goal_ne))

            g = nx.Graph()
            for e in edges:
                p1 = e[0]
                p2 = e[1]
                dist = la.norm(np.array(p2) - np.array(p1))
                g.add_edge(p1, p2, weight=dist)
            start_ne_g = closest_point(g, start_ne)
            goal_ne_g = closest_point(g, goal_ne)

            path, cost = a_star_graph(g, heuristic, start_ne_g, goal_ne_g)

            print("count of waypoints: ", len(path))
            path = prune_waypoints(path)
            print("count of waypoints after prune: ", len(path))

            if self.plot_maps:
                plot_map_graph(grid, edges, path, start_ne, goal_ne)

        # Convert path to waypoints
        waypoints = [[int(p[0]) + north_offset, int(p[1]) + east_offset, self.default_altitude, 0] for p in path]
        self.waypoints = waypoints
        self.send_waypoints()
        self.flight_state = States.PLANNING

    def arming_transition(self):
        print("arming transition")
        self.arm()
        self.take_control()
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        # Read in obstacle map
        self.takeoff(self.default_altitude)
        self.takeoff_global_home = self.global_home
        print('takeoff global home: {0}, position: {1}, local position: {2}'
              .format(self.global_home, self.global_position, self.local_position))
        self.flight_state = States.TAKEOFF

    def takeoff_to_safe_altitude(self):
        print("takeoff to safe altitude")
        self.takeoff(self.safe_altitude)
        self.flight_state = States.TAKEOFF_TO_SAFE_ALTITUDE

    def takeoff_to_def_altitude(self):
        print("takeoff to default altitude")
        self.takeoff(self.default_altitude)
        self.flight_state = States.TAKEOFF_TO_DEF_ALTITUDE

    def waypoint_transition(self):
        print("waypoint transition")
        # if we have waypoints take first and fly to it
        if self.waypoints:
            wp = self.waypoints.pop(0)
            # we need to correct that waypoint - because after set_home_position
            # we have absolutely strange behaviour related to global_home, and API, etc.
            # Not sure about quality of float operations in Python and numpy/etc :(
            self.expected_local_position = self.correct_waypoint(wp)
            self.cmd_position(*self.expected_local_position)
            print("waypoint arriving")
            self.flight_state = States.WAYPOINT_ARRIVING
        else:
            # otherwise - just set that waypoints completed
            # and we should land
            print("waypoint completed")
            self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        # make landing smooth
        self.takeoff(0.1)
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        print("creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("closing log file")
        self.stop_log()

    def send_waypoints(self):
        print("sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def go_to_home_position(self):
        print('go to home position: lon {0}, lat {1}'.format(self.start_lon, self.start_lat))
        self.set_home_position(self.start_lon, self.start_lat, 0)
        self.flight_state = States.ARRIVING_TO_HOME_POSITION

    def to_grid_ne(self, lon, lat, north_offset, east_offset):
        north_east_local = global_to_local([lon, lat, 0], self.takeoff_global_home)
        grid_north_east = (int(north_east_local[0] - north_offset), int(north_east_local[1] - east_offset))
        return grid_north_east

    def correct_waypoint(self, waypoint):
        """
        Correct waypoint for set_cmd_position
        Many reasons for that - set_home_position, translations of positions,
        difference between plotted grid and simulator, absence of valid info in simulator, etc.
        """
        global_ll = local_to_global(waypoint[:3], self.takeoff_global_home)
        local_wp = global_to_local(global_ll, self.global_home)
        local_wp = [local_wp[0], local_wp[1], local_wp[2], 0]
        return local_wp


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--alg', type=int, default=0, help="Path algorithm: 0 - simple, 1 - skeleton, 2 - graphs")
    parser.add_argument('--lat', type=float, default=37.790669, help="Goal latitude")
    parser.add_argument('--lon', type=float, default=-122.399613, help="Goal longitude")
    parser.add_argument('--plots', type=bool, default=True, help="Save plot maps")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=10000)
    drone = MotionPlanning(conn, PathAlgorithm(args.alg), args.lat, args.lon, args.plots)

    time.sleep(1)
    drone.start()
