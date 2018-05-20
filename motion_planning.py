import argparse
import time
#import matplotlib.pyplot as plt
import numpy as np
import msgpack

from enum import Enum
from collections import deque
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from planning_utils import StateDiagram, Plot, GpsLocation
from planner import Planner
from shapely.geometry import Polygon, Point

class WayPointResult(Enum):
    NOTREACHED = 0
    REACHED = 1
    PATH_COMPLETE = 2

class States(Enum):
    MANUAL = 0
    ARMED = 1
    PLANNING = 2
    TAKEOFF = 3
    WAYPOINT = 4
    LANDING = 5
    DISARMING = 6

class PlanResult(Enum):
    NOT_PLANNED = 0,
    PLAN_SUCCESS = 1,
    PLAN_FAILED = 2

class MotionPlanning(Drone):
    def __init__(self, connection):
        super().__init__(connection)

        self.WAYPOINT_HIT_RADIUS = 5.0

        self.in_mission = True
        self.takeoff_altitude = 3.0

        # create state diagram and set the initial state
        self.flight_state, self.state_diagram = self.create_state_diagram()

        self.plot = Plot()
        self.plan_status = PlanResult.NOT_PLANNED
        self.planner = Planner()

        self.current_waypoint = (0,0,0)         # current waypoint to be followed
        self.current_wp_index = 0               # index into the waypoint np.array
        self.all_waypoints = []                 # all waypoints computed by the planner

        self.min_altitude = 5
        self.max_altitude = 50

        self.path2d_with_cost = []              # 2d path returned by the planner. This has the cost
                                                # of reaching goal from each state. Is used for 3d planning

        super().register_callback(MsgID.GLOBAL_POSITION, self.record_initial_gps)
    
    def create_state_diagram(self):
        # each state in the diagram has a condition that checks if the state
        # work is complete and has a transition function
        state_diagram = StateDiagram(self)

        state_diagram.add(States.MANUAL, MsgID.STATE, None, self.arming_transition)

        # transition to TAKEOFF, if drone is armed and in ARMING state
        state_diagram.add(States.ARMED, MsgID.STATE, lambda: self.armed, 
                                self.planning_transition)

        # in case we are not able to plan a route for the given states, declare failure
        # and shutdown
        state_diagram.add(States.PLANNING, MsgID.STATE, 
                                lambda: self.plan_status,
                                PlanResult.PLAN_SUCCESS, self.takeoff_transition,
                                PlanResult.PLAN_FAILED, self.disarming_transition)

        # when the drone reaches the given take off altitude, switch to waypoint
        state_diagram.add(States.TAKEOFF, MsgID.LOCAL_POSITION, 
                                self.has_reached_altitude, self.waypoint_transition)

        # when one waypoint has been reached, move to the next one BUT if there
        # are no more waypoints then go to landing transition                        
        state_diagram.add(States.WAYPOINT, MsgID.LOCAL_POSITION, self.has_waypoint_reached, 
                                WayPointResult.REACHED, self.waypoint_transition,
                                WayPointResult.PATH_COMPLETE, self.landing_transition)

        # when the drone has landed, go to disarm transition                                
        state_diagram.add(States.LANDING, MsgID.LOCAL_VELOCITY, self.has_landed, 
                                self.disarming_transition)

        # when drone has disarmed, go to manual mode                        
        state_diagram.add(States.DISARMING, MsgID.STATE, self.has_disarmed, 
                                self.manual_transition)

        return States.MANUAL, state_diagram


    def planning_transition(self):
        print('planning_transition')

        self.flight_state = States.PLANNING

        print('Loading map...')
        self.planner.load_map()

        pos = self.planner.home_gps_pos
        self.set_home_position(pos.lon, pos.lat, pos.altitude)

        print("Setting home to : ", pos)
        print("global_home: ", self.global_home)

        # local position of the drone

        current_global = self.global_position
        current_local = global_to_local(current_global, self.global_home)

        start = current_local

        print('Local pos: ', current_local)

        #goal = (-self.planner.north_min + 10, -self.planner.east_min + 10)

        goal_gps = GpsLocation(37.79489867, -122.39687191, 0)
        goal = global_to_local(goal_gps, self.global_home)
        goal = np.array([int(goal[0]), int(goal[1]), 0])

        print('Drone will fly {} to {}'.format(start, goal))

        # get a pruned, a_starred path from the planner
        
        print('Finding a plan...')
        self.path2d_with_cost = self.planner.plan_route(start, goal)

        # fig = plt.figure()
        # grid = self.planner.create_grid()

        # plt.imshow(grid, cmap='Greys', origin='lower')
        # plt.scatter(start[1] - self.planner.east_min, start[0] - self.planner.east_min, color='blue')
        # plt.scatter(goal[1] - self.planner.east_min, goal[0] - self.planner.east_min, color='green')

        # plt.show()
        if len(self.path2d_with_cost) > 0:
            print('Plan found')

            self.all_waypoints = np.array(self.path2d_with_cost)[:, :3]
            self.send_waypoints()

            self.plan_status = PlanResult.PLAN_SUCCESS
        else:
            print("Could not plan a path for the given start and goal state")
            self.plan_status = PlanResult.PLAN_FAILED

        #self.generate_3dpath()
        # self.plan_status = PlanResult.PLAN_FAILED

        # debug
        # self.all_waypoints = np.array([start, goal])
        # self.send_waypoints()
        #self.plan_status = PlanResult.PLAN_SUCCESS
        
    def record_initial_gps(self):
        pos = super().global_position
        self.planner.set_initial_gps(GpsLocation(pos[1], pos[0], pos[2]))
        #print('setting initial gps location')
        #print(self.planner.init_gps)
        super().remove_callback(MsgID.GLOBAL_POSITION, self.record_initial_gps)


    def has_disarmed(self):
        # print('has drone disarmed', self.armed, self.guided)
        return not (self.armed or self.guided)

    def has_reached_altitude(self):
        # TODO check code in udacridrone to figure out how local_position is being computed.
        # There is a message on slack from @Krishna that says not to use local_position:
        # "Many people were confused about the above, Ryan later confirmed that we should not 
        # rely on `self.local_position` and rather calculate `local_pos` ourselves using the 
        # function to convert `self.global_position` (which is trustable)."

        altitude = -1.0 * self.local_position[2]
        return altitude > 0.90 * self.current_waypoint[2]

    def has_landed(self):
        print('has_landed')

        # check difference between desired altitude and our current altitude
        # return true in case the difference is < 0.5 and our velocity is < 0.1
        altitude = -self.local_position[2]
        desired = -self.current_waypoint[2]

        diff = np.abs(desired - altitude)
        return diff < 0.5 and self.local_velocity[2] < 0.1

    def has_waypoint_reached(self):
        if self.current_wp_index >= len(self.all_waypoints):
            print("Path is complete. Returning WayPointResult.PATH_COMPLETE")
            return WayPointResult.PATH_COMPLETE

        # resend command in case the waypoint has not been reached
        # self.cmd_position(int(self.current_waypoint[0]), int(self.current_waypoint[1]), self.min_height, 0.0)
        # print(int(self.current_waypoint[0]), int(self.current_waypoint[1]), self.min_height, 0.0)
    
        # print('Is close to current: {}. Local pos: {}. Current: {}'.format(
        #     self.is_close_to_current(), 
        #     self.local_position,
        #     self.current_waypoint))
        
        return WayPointResult.REACHED if  self.is_close_to_current() else WayPointResult.NOTREACHED

    def arming_transition(self):
        self.take_control()

        if not self.armed:
            self.arm()

        self.flight_state = States.ARMED

    def takeoff_transition(self):
        print('takeoff_transition to {} meters'.format(self.takeoff_altitude))
        # set current waypoint at a higher / lower altitude than we currently are
        self.current_waypoint = (self.local_position[0], 
                            self.local_position[1], 
                            self.takeoff_altitude)

        self.takeoff(self.current_waypoint[2])
        self.flight_state = States.TAKEOFF

    def calculate_heading(self, p1, p2):
        # consider both p1 and p2 as vectors and assuming drone
        # already has the heading going towards p1 so p1 is a vector in itself
        # print('p1:', p1)
        # print('p2:', p2)
        # print('p1 norm:', np.linalg.norm(p1))
        # print('p2 norm:', np.linalg.norm(p2))

        heading = np.arctan2((p2[1]-p1[1]), (p2[0]-p1[0]))
        print('Heading: ', heading)
        return heading

        # # add a very small number to the norm to not have a division by 0 error
        # p1_norm = np.linalg.norm(p1)
        # p2_norm = np.linalg.norm(p2)

        # if p1_norm == 0 or p2_norm == 0:
        #     return 0.0

        # p1_unit = np.array(p1) / p1_norm
        # p2_unit = np.array(p2) / p2_norm

        # # print('p1_unit:', p1_unit)
        # # print('p2_unit:', p2_unit)

        # dot = np.dot(p1_unit, p2_unit)
        # # print('dot:', dot)

        # # 0 heading means north, where as in degrees we would get 90
        # # so subtracting 90 from the degree
        # heading = 90.0 - np.degrees(np.arccos(dot))
        # # print('heading:', heading)
        # return heading
        
    def waypoint_transition(self):
        print('waypoint_transition')

        if self.current_wp_index < len(self.all_waypoints):
            owp = self.current_waypoint
            self.current_waypoint = self.all_waypoints[self.current_wp_index]
            self.current_wp_index += 1

            heading = self.calculate_heading(owp, self.current_waypoint)

            # (y2 - y1) / (x2 - x1) (conver to radians)
            #vector_other_pt = (self.current_waypoint[1] - owp[1])/(self.current_waypoint[0] - owp[0])

            self.cmd_position(self.current_waypoint[0], 
                            self.current_waypoint[1], 
                            -self.current_waypoint[2], 
                            heading)
            self.flight_state = States.WAYPOINT
            
            #print("transit to waypoint: ", self.current_waypoint)
        else:
            self.current_waypoint = None

    def landing_transition(self):
        print('landing_transition')
        # make sure the drone has stopped moving and then land
        if ((self.local_velocity[0] + self.local_velocity[1]) ** 2.0) ** 0.5 < .1:
            print('Velocity is ok. Going to land')
            self.land()
            self.flight_state = States.LANDING
        else:
            print('Waiting for drone to slow down')

    def disarming_transition(self):
        print('disarming')
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print('Manual transition')
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL
        print('Stop called and manual state set')
        
    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        wp  = self.all_waypoints.astype(int).tolist()
        print(wp)

        data = msgpack.dumps(wp)
        self.connection._master.write(data)

    def is_close_to_current(self):
        pos = self.local_position
        #pos = self.global_pos_ned

        distance = ((self.current_waypoint[0] - pos[0]) ** 2 + 
                    (self.current_waypoint[1] - pos[1]) ** 2 + 
                    (self.current_waypoint[2] - pos[2]) ** 2) ** 0.5

        # print('is_close_to_current: ', self.local_position, self.current_waypoint, distance)
        return distance < self.WAYPOINT_HIT_RADIUS

    def start(self):
        # no point in creating a log file, telemetry log is created by parent drone class
        # and that has every message in it
        # print("Creating log file, NavLog.txt for local position logging")
        # self.start_log("Logs", "NavLog.txt")
        # self.log.num_data = 3

        print("starting connection")
        self.connection.start()

        # print("Closing log file")
        # self.stop_log()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = MotionPlanning(conn)
    time.sleep(2)
    drone.start()
