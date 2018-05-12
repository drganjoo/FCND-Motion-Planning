import numpy as np
import visdom
from scipy.spatial import Voronoi
from bresenham import bresenham
from enum import Enum
from udacidrone import Drone
from udacidrone.messaging import MsgID
from PIL import Image
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re


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
    def __init__(self):
        """python -m visdom.server"""
        self.drone = None
        self.viz = visdom.Visdom()  

    def register_pos_callback(self, drone):
        self.drone = drone

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

            #drone.register_callback(MsgID.LOCAL_POSITION, self.localpos_callback)
        else:
            print('Could not connect to visdom server. Please start server using python -m visdom.server')
            self.v = None

    def localpos_callback(self):
        if self.drone == None:
            return
            
        X = np.array([[self.drone.local_position[1], self.drone.local_position[0]]])
        self.viz.scatter(X, win = self.local_plot, update = 'append')

    def show_grid(self, grid, drone_height):
        # Heighest buildings have dark color
        non_obstacle = grid == 0

        if grid.max() == 1:
            # 2d grid has been passed in 
            image = np.array(grid)

            # show obstacles as black and non-obstacles as white
            image[image == 1] = 0
            image[non_obstacle] = 1

            # flip image, equivalent to plt.show(origin=='lower')
            image = np.flip(image, 0)
        else:
            above_height = grid > drone_height
            below_height = ~above_height ^ non_obstacle

            image = np.zeros((grid.shape[0], grid.shape[1], 3)).astype(np.uint8)
            image[below_height] = [0, 0, 255]
            image[above_height] = [255, 0, 0]
            image[non_obstacle] = [255,255,255]

            # flip image, equivalent to plt.show(origin=='lower')
            image = np.flip(image, 0)

            # internally for some reason viz is doing a transformation before trying to plot
            # image. 
            image = np.transpose(image, (2, 0, 1))

        self.viz.image(image,
            opts=dict(title='Grid', caption='Grid'))

    def show_3dgrid(self, worldmap, voxel_size = 5):
        pass

    @property
    def is_connected(self):
        return self.v.check_connection()


# class BoxPath:
#     """The path drone is suppose to follow is represented by BoxPath. Additionally
#     It is used by the BackyadFlier to get the next waypoint and to figure out if 
#     the current waypoint has been reached or not"""
#     class WayPointResult(Enum):
#         NOTREACHED = 0
#         REACHED = 1
#         PATH_COMPLETE = 2

#     def __init__(self):
#         self.all_waypoints = self.calculate_box()
#         self.current_target = []

#     def calculate_box(self):
#         # N, E, Alt, Heading
#         distance = 10.0
#         altitude = 3.0
        
#         return [[distance, 0.0, altitude, 0.0], 
#                 [distance, distance, altitude, 0.0], 
#                 [0.0, distance, altitude, 0.0], 
#                 [0.0, 0.0, altitude, 0.0]]

#     def get_next(self):
#         if self.all_waypoints:
#             next_waypoint = self.all_waypoints.pop(0)
#         else:
#             next_waypoint = []

#         self.current_target = next_waypoint
#         return next_waypoint

#     def is_close_to_current(self, local_position):
#         if not self.current_target:
#             return BoxPath.WayPointResult.PATH_COMPLETE
#         else:
#             # distance = square root of (x2-x1) + (y2-y1)
#             distance = ((self.current_target[0] - local_position[0]) ** 2 
#                         + (self.current_target[1] - local_position[1]) ** 2) ** 0.5
#             if distance < 1:
#                 return BoxPath.WayPointResult.REACHED

#             return BoxPath.WayPointResult.NOTREACHED

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

    def __getitem__(self, index):
        if 0 == index:
            return self._lon
        elif 1 == index:
            return self._lat
        elif 2 == index:
            return self._altitude

class WorldMap():
    def __init__(self, filename = 'colliders.csv', safety_distance = 3):
        self.filename = filename
        self.home_gps_pos = GpsLocation(0,0,0)
        self.is_loaded = False
        self.safety_distance = safety_distance
        self.data = []
        self._grid25 = []
        # self.north = []
        # self.east = []
        self.height = []
        self.north_min_max = []             # obstacle's north min / max
        self.east_min_max = []
        self.grid_size = (0, 0)             # grid size in the north, east direction

    @property
    def loaded(self):
        return self.is_loaded

    @property
    def grid25(self):
        return self._grid25

    def create_grid_forheight(self, drone_altitude):
        grid = self.create_grid25_forheight(drone_altitude)
        grid[grid > 0] = 1
        return grid

    def load(self):
        # read the home location saved in the file
        with open(self.filename) as f:
            line = f.readline()
            m = re.match('lat0\s(.*),\slon0\s(.*)', line)
            if m:
                self.home_gps_pos = GpsLocation(float(m.group(1)), float(m.group(2)), 0)
            else:
                self.home_gps_pos = GpsLocation(0,0,0)

        # read complete data file
        data = np.loadtxt(self.filename, delimiter=',', dtype='Float64', skiprows=2)

        # top, bottom coordinates
        north = np.array([np.floor(data[:, 0] - data[:, 3]), 
                        np.ceil(data[:, 0] + data[:, 3])]).T

        # left, right coordinates
        east = np.array([np.floor(data[:, 1] - data[:, 4]),
                        np.ceil(data[:, 1] + data[:, 4])]).T

        self.height = np.array(np.ceil(data[:, 2] + data[:, 5]))
        self.data = data

        self.north_min_max = (int(np.floor(np.amin(north[:, 0]))),
                            int(np.ceil(np.amax(north[:, 1]))))

        self.east_min_max = (int(np.floor(np.amin(east[:, 0]))),
                            int(np.ceil(np.amax(east[:, 1]))))

        print("load_map: Data North Min: {}, Max: {}".format(self.north_min_max[0], self.north_min_max[1]))
        print("load_map: Data East Min: {}, Max: {}".format(self.east_min_max[0], self.east_min_max[1]))
        print("load_map: Home Position: {}".format(self.home_gps_pos))

        self.create_grid25()

    def create_grid25(self):
        """This is more for debugging as we are using graphs and not grid. But this helps in plotting"""
        # given the minimum and maximum coordinates we can
        # calculate the size of the grid.
        north_size = int(np.ceil(self.north_min_max[1] - self.north_min_max[0]))
        east_size = int(np.ceil(self.east_min_max[1] - self.east_min_max[0]))

        # compute max height of all obstacles, figure out the ones that 
        # will exceed drone's height (considering drone was about safe_distance lower)
        #collision_index = self.height > (drone_altitude - safety_distance)
        collision_index = self.height > 0

        # all obstacles that we can collide with
        obstacles = self.data[collision_index]
        
        # min computation: (obstalce_north - delta - safety) - north_min (to make it 0 based as per range)
        # max computation: (obstalce_north + delta + safety) - north_min (to make it 0 based as per range) BUT add 1 so that
        #   when we later on use [o_north_min[i] : o_north_max[i]] the range becomes inclusive
        o_north_min = np.clip(obstacles[:, 0] - obstacles[:, 3] - self.safety_distance - self.north_min_max[0], 
                        0, north_size - 1).astype(int)
        o_north_max = np.clip(obstacles[:, 0] + obstacles[:, 3] + self.safety_distance - self.north_min_max[0] + 1, 
                        0, north_size).astype(int)
        o_east_min = np.clip(obstacles[:, 1] - obstacles[:, 4] - self.safety_distance - self.east_min_max[0], 
                        0, east_size - 1).astype(int)
        o_east_max = np.clip(obstacles[:, 1] + obstacles[:, 4] + self.safety_distance - self.east_min_max[0] + 1, 0, 
                        east_size).astype(int)

        grid25 = np.zeros((north_size, east_size))

        # set grid to 1 for all places where this is an obstacle
        for i in range(0, obstacles.shape[0]):
            grid25[o_north_min[i] : o_north_max[i], o_east_min[i] : o_east_max[i]] = self.height[i]
        
        self._grid25 = grid25
        self.grid_size = (north_size, east_size)

    def create_grid25_forheight(self, drone_altitude):
        grid25_copy = np.array(self._grid25)

        # mark all obstacles that are below the drone's height as ok
        grid25_copy[grid25_copy < (drone_altitude - self.safety_distance)] = 0
        return grid25_copy

    def create_grid3d(self, voxel_size):
        """
        Returns a grid representation of a 3D configuration space
        based on given obstacle data.
        
        The `voxel_size` argument sets the resolution of the voxel map. 
        """
        grid25 = self._grid25
        north_min, north_max = self.north_min_max
        east_min, east_max = self.east_min_max
        alt_max = np.amax(self.height)
        
        # given the minimum and maximum coordinates we can
        # calculate the size of the grid.
        north_size = int(np.ceil((north_max - north_min))) // voxel_size
        east_size = int(np.ceil((east_max - east_min))) // voxel_size
        alt_size = int(alt_max) // voxel_size

        voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)

        # all obstacles that we can collide with
        collision_index = self.height > 0
        obstacles = self.data[collision_index]

        # min computation: (obstalce_north - delta - safety) - north_min (to make it 0 based as per range)
        # max computation: (obstalce_north + delta + safety) - north_min (to make it 0 based as per range) BUT add 1 so that
        #   when we later on use [o_north_min[i] : o_north_max[i]] the range becomes inclusive
        o_north_min = np.floor(obstacles[:, 0] - obstacles[:, 3] - self.safety_distance - north_min)
        o_north_max = np.ceil(obstacles[:, 0] + obstacles[:, 3] + self.safety_distance - north_min)
        o_east_min = np.floor(obstacles[:, 1] - obstacles[:, 4] - self.safety_distance - east_min)
        o_east_max = np.ceil(obstacles[:, 1] + obstacles[:, 4] + self.safety_distance - east_min)
        o_alt_max = np.ceil(self.height + self.safety_distance)
        
        o_north_min = np.clip(o_north_min // voxel_size, 0, north_size - 1).astype(int)
        o_north_max = np.clip(o_north_max // voxel_size, 0, north_size - 1).astype(int)
        o_east_min = np.clip(o_east_min // voxel_size, 0, east_size - 1).astype(int)
        o_east_max = np.clip(o_east_max // voxel_size, 0, east_size - 1).astype(int)
        o_alt = np.clip(o_alt_max // voxel_size, 0, alt_size - 1).astype(int)
        
        # set grid to 1 for all places where this is an obstacle
        for i in range(0, obstacles.shape[0]):
            voxmap[o_north_min[i] : o_north_max[i] + 1, 
                o_east_min[i] : o_east_max[i] + 1,
                0 : o_alt[i] + 1] = 1

        return voxmap

# def create_grid(data, drone_altitude, safety_distance):
#     """
#     Returns a grid representation of a 2D configuration space
#     based on given obstacle data, drone altitude and safety distance
#     arguments.
#     """

#     # minimum and maximum north coordinates
#     north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
#     north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

#     # minimum and maximum east coordinates
#     east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
#     east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

#     # given the minimum and maximum coordinates we can
#     # calculate the size of the grid.
#     north_size = int(np.ceil(north_max - north_min))
#     east_size = int(np.ceil(east_max - east_min))

#     # Initialize an empty grid
#     grid = np.zeros((north_size, east_size))

#     # Populate the grid with obstacles
#     for i in range(data.shape[0]):
#         north, east, alt, d_north, d_east, d_alt = data[i, :]
#         if alt + d_alt + safety_distance > drone_altitude:
#             obstacle = [
#                 int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
#                 int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
#                 int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
#                 int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
#             ]
#             grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

#     return grid

# def create_grid_and_edges(data, drone_altitude, safety_distance):
#     """
#     Returns a grid representation of a 2D configuration space
#     along with Voronoi graph edges given obstacle data and the
#     drone's altitude.
#     """
#     # minimum and maximum north coordinates
#     north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
#     north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

#     # minimum and maximum east coordinates
#     east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
#     east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

#     # given the minimum and maximum coordinates we can
#     # calculate the size of the grid.
#     north_size = int(np.ceil(north_max - north_min))
#     east_size = int(np.ceil(east_max - east_min))

#     # Initialize an empty grid
#     grid = np.zeros((north_size, east_size))
#     # Initialize an empty list for Voronoi points
#     points = []
#     # Populate the grid with obstacles
#     for i in range(data.shape[0]):
#         north, east, alt, d_north, d_east, d_alt = data[i, :]
#         if alt + d_alt + safety_distance > drone_altitude:
#             obstacle = [
#                 int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
#                 int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
#                 int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
#                 int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
#             ]
#             grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

#             # add center of obstacles to points list
#             points.append([north - north_min, east - east_min])

#     # TODO: create a voronoi graph based on
#     # location of obstacle centres
#     graph = Voronoi(points)

#     # TODO: check each edge from graph.ridge_vertices for collision
#     edges = []

#     # set all vertices that are out of bounds to -1
#     vertices = graph.vertices.astype(int)
#     n_z = vertices[:, 0] < 0
#     e_z = vertices[:, 1] < 0
#     out_of_bounds = np.where(n_z | e_z)
#     vertices[out_of_bounds] = np.array([-1, -1])
    
#     n_z = vertices[:, 0] >= grid.shape[0]
#     e_z = vertices[:, 1] >= grid.shape[0]
#     out_of_bounds = np.where(n_z | e_z)
#     vertices[out_of_bounds] = np.array([-1, -1])

#     for e in graph.ridge_vertices:
#         p1 = vertices[e[0]]
#         p2 = vertices[e[1]]
        
#         collision = False
#         if p1[0] == -1 or p2[0] == -1:
#             collision = True
#         else:
#             cells = bresenham(p1[0], p1[1], p2[0], p2[1])
#             for c in cells:
#                 if grid[c[0], c[1]] == 1:
#                     collision = True
#                     break

#         if not collision:
#             edges.append((p1, p2))
    
#     return grid, edges