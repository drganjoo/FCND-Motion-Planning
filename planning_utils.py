import numpy as np
import visdom

from enum import Enum
from udacidrone import Drone
from udacidrone.messaging import MsgID

class StateDiagram:
    """Represents a state diagram. Different flight states are handled behind
    different callbacks. e.g. when MsgID.State is called, MANUAL, ARMED, DISARMING
    are checked. Behind MsgID.LocalPosition WAYPOINT flight state is checked. This class
    helps to consolidate these checks in one place.
    A dictionary is maintained for each callback. Each key in the dictionary is the 
    possible flight state and the value holds (a) a condition function to call and (b)
    another dictionary that tells which transition function to call behind different
    return values of the condition function."""
    class StateNode:
        def __init__(self, condition_fn, result_map):
            self.condition_fn = condition_fn
            self.result_map = result_map

    def __init__(self, drone):
        self.drone = drone
        self.event_to_state = {}

        drone.register_callback(MsgID.STATE, self.state_callback)
        drone.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        drone.register_callback(MsgID.LOCAL_VELOCITY, self.local_velocity_callback)

    def state_callback(self):
        if self.drone.in_mission:
            if MsgID.STATE in self.event_to_state:
                state_diagram = self.event_to_state[MsgID.STATE]
                self.call_state_node(state_diagram)

    def local_position_callback(self):
        if MsgID.LOCAL_POSITION in self.event_to_state:
            state_diagram = self.event_to_state[MsgID.LOCAL_POSITION]
            self.call_state_node(state_diagram)

    def local_velocity_callback(self):
        if MsgID.LOCAL_VELOCITY in self.event_to_state:
            state_diagram = self.event_to_state[MsgID.LOCAL_VELOCITY]
            self.call_state_node(state_diagram)

    def call_state_node(self, state_diagram):
        # in case there is a state node that would like to work when the 'name' message
        # arrives, call the condition function, check the return value against the possible
        # transition functions and in case the return value matches one of the transition 
        # function, call it
        if self.drone.flight_state in state_diagram:
            state_node = state_diagram[self.drone.flight_state]

            # in case there is no condition function, just call the transition
            # function directly
            if state_node.condition_fn is None:
                fn = state_node.result_map[True]
                fn()
            else:
                result = state_node.condition_fn()
                if result in state_node.result_map:
                    state_node.result_map[result]()

    def add(self, flight_state, msg_id, condition_fn, *result_transition):
        if msg_id not in self.event_to_state:
            self.event_to_state[msg_id] = {}

        state_diagram = self.event_to_state[msg_id]

        # warn, in case a given event handler already has a state node for the 
        # particular flight_state e.g MsgID.State already has work defined for
        # Manual state, warn the user
        if flight_state in state_diagram:
            print("\x1b[32m;State {0} already has a node attached to it".format(flight_state))

        result_map = {}

        # in case transition function is to be called only on True / False of the
        # function then put an entry for True->Transition function
        if condition_fn is None or len(result_transition) == 1:
            result_map[True] = result_transition[0]
        else:
            result_map = {}
            for i in range(0, len(result_transition), 2):
                result_map[result_transition[i]] = result_transition[i+1]

        state_diagram[flight_state] = self.StateNode(condition_fn, result_map)


class Plot:
    def __init__(self, drone):
        """python -m visdom.server"""
        self.drone = drone
        self.viz = visdom.Visdom()

        if self.viz.check_connection():
            X = np.array([[self.drone.local_position[1], self.drone.local_position[0]]])

            self.local_plot = self.viz.scatter(
                X, opts=dict(
                    title="Local position (north, east)", 
                    xlabel='East', 
                    ylabel='North',
                    xtickmin=-5,
                    xtickmax=15,
                    xtickstep=1,
                    ytickmin=-5,
                    ytickmax=15,
                    ytickstep=1 ,
                    ))

            drone.register_callback(MsgID.LOCAL_POSITION, self.localpos_callback)
        else:
            print('Could not connect to visdom server. Please start server using python -m visdom.server')
            self.v = None

    def localpos_callback(self):
        X = np.array([[self.drone.local_position[1], self.drone.local_position[0]]])
        self.viz.scatter(X, win = self.local_plot, update = 'append')

    @property
    def is_connected(self):
        return self.v.check_connection()


class BoxPath:
    """The path drone is suppose to follow is represented by BoxPath. Additionally
    It is used by the BackyadFlier to get the next waypoint and to figure out if 
    the current waypoint has been reached or not"""
    class WayPointResult(Enum):
        NOTREACHED = 0
        REACHED = 1
        PATH_COMPLETE = 2

    def __init__(self):
        self.all_waypoints = self.calculate_box()
        self.current_target = []

    def calculate_box(self):
        # N, E, Alt, Heading
        distance = 10.0
        altitude = 3.0
        
        return [[distance, 0.0, altitude, 0.0], 
                [distance, distance, altitude, 0.0], 
                [0.0, distance, altitude, 0.0], 
                [0.0, 0.0, altitude, 0.0]]

    def get_next(self):
        if self.all_waypoints:
            next_waypoint = self.all_waypoints.pop(0)
        else:
            next_waypoint = []

        self.current_target = next_waypoint
        return next_waypoint

    def is_close_to_current(self, local_position):
        if not self.current_target:
            return BoxPath.WayPointResult.PATH_COMPLETE
        else:
            # distance = square root of (x2-x1) + (y2-y1)
            distance = ((self.current_target[0] - local_position[0]) ** 2 
                        + (self.current_target[1] - local_position[1]) ** 2) ** 0.5
            if distance < 1:
                return BoxPath.WayPointResult.REACHED

            return BoxPath.WayPointResult.NOTREACHED

class GpsLocation():
    def __init__(self, lat, lon, altitude):
        self._lat = lat
        self._lon = lon
        self._altitude = 0

    @property
    def lat(self):
        return self._lat

    @property
    def lon(self):
        return self._lon

    @property
    def altitude(self):
        return self._altitude

    def __repr__(self):
        return "Lat: {} Lon: {} Altitude {}".format(self.lat, self.lon, self.altitude)