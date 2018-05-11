import argparse
import time
from enum import Enum
from collections import deque

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

from planning_utils import StateDiagram, Plot, BoxPath, GpsLocation
from planner import Planner

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

        self.in_mission = True
        self.takeoff_altitude = 3.0

        # create state diagram and set the initial state
        self.flight_state, self.state_diagram = self.create_state_diagram()

        self.plot = Plot()
        self.plan_status = PlanResult.NOT_PLANNED
        self.planner = Planner()

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
                                BoxPath.WayPointResult.REACHED, self.waypoint_transition,
                                BoxPath.WayPointResult.PATH_COMPLETE, self.landing_transition)

        # when the drone has landed, go to disarm transition                                
        state_diagram.add(States.LANDING, MsgID.LOCAL_VELOCITY, self.has_landed, 
                                self.disarming_transition)

        # when drone has disarmed, go to manual mode                        
        state_diagram.add(States.DISARMING, MsgID.STATE, self.has_disarmed, 
                                self.manual_transition)

        return States.MANUAL, state_diagram

    def planning_transition(self):
        self.flight_state = States.PLANNING

        # planner = Planner()
        # planner.load_map()
        # planner.set_start_goal(start, goal)
        # self.path = planner.find_summary_path()
        # self.receding_path = planner.find_receding_path(self.RECEDING_RADIUS)
        # self.plan_status = PlanResult.PLAN_SUCCESS
        self.planner.load_map()
        pos = self.planner.home_gps_pos
        self.set_home_position(pos.lon, pos.lat, pos.altitude)

        print("Home location (from planner): ", pos)
        print("Home location (global_home): ", self.global_home)

        self.plan_status = PlanResult.PLAN_FAILED
        print("NO PLAN GENERATED")
        
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
        altitude = -1.0 * self.local_position[2]
        return altitude > 0.95 * self.takeoff_altitude

    def has_landed(self):
        altitude = -1.0 * self.local_position[2]
        return altitude < 0.5 and self.local_velocity[2] < 0.1

    def has_waypoint_reached(self):
        return self.path_planner.is_close_to_current(self.local_position)

    def arming_transition(self):
        self.take_control()

        if not self.armed:
            self.arm()

        self.flight_state = States.ARMED

    def takeoff_transition(self):
        self.takeoff(self.takeoff_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        next_waypoint = self.path_planner.get_next()
        if next_waypoint:
            # print('Setting command position to: ', *next_waypoint)
            self.cmd_position(*next_waypoint)
            self.flight_state = States.WAYPOINT
            
            print("transit to waypoint: ", next_waypoint)

    def landing_transition(self):
        # make sure the drone has stopped moving and then land
        if ((self.local_velocity[0] + self.local_velocity[1]) ** 2.0) ** 0.5 < .1:
            self.land()
            self.flight_state = States.LANDING

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
