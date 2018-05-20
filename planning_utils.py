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
            self.viz = None

    def scatter(self, points):
        X = np.array([[points[:, 1]], points[:, 0]])
        self.viz.scatter(X, win = self.local_plot, update = 'append')

    def matplot(self, plt):
        self.viz.matplot(plt)

    def line(self, points):
        X = np.column_stack((points[:, 1], points[:, 3]))
        Y = np.column_stack((points[:, 0], points[:, 2]))
        
        #print(X)
        #print(Y)

        self.viz.line(X=X, Y=Y)
        #self.viz.line(Y=np.random.rand(10), opts=dict(showlegend=True))

    def localpos_callback(self):
        if self.drone == None:
            return
            
        X = np.array([[self.drone.local_position[1], self.drone.local_position[0]]])
        self.viz.scatter(X, win = self.local_plot, update = 'append')

    def show_grid(self, grid):
        if not self.is_connected:
            return

        # Heighest buildings have dark color
        non_obstacle = grid == 0

        # 2d grid has been passed in 
        image = np.array(grid)

        # show obstacles as black and non-obstacles as white
        image[image == 1] = 0
        image[non_obstacle] = 1

        # flip image, equivalent to plt.show(origin=='lower')
        image = np.flip(image, 0)

        self.grid_plot = self.viz.image(image,
            opts=dict(title='Grid', caption='Grid'))

    def show_image(self, image, title = '', caption = ''):
        self.grid_plot = self.viz.image(image, opts=dict(title=title, caption=caption))

    def show_grid25(self, grid25, drone_height):
        if not self.is_connected:
            return

        # Heighest buildings have dark color
        non_obstacle = grid25 == 0

        above_height = grid25 > drone_height
        below_height = ~above_height ^ non_obstacle

        image = np.zeros((grid25.shape[0], grid25.shape[1], 3)).astype(np.uint8)
        image[below_height] = [0, 0, 255]
        image[above_height] = [255, 0, 0]
        image[non_obstacle] = [255,255,255]

        # flip image, equivalent to plt.show(origin=='lower')
        image = np.flip(image, 0)

        # internally for some reason viz is doing a transformation before trying to plot
        # image. 
        image = np.transpose(image, (2, 0, 1))

        self.grid_plot = self.viz.image(image,
            opts=dict(title='Grid', caption='Grid'))

    def show_3dgrid(self, worldmap, voxel_size = 5):
        pass

    @property
    def is_connected(self):
        return self.viz != None and self.viz.check_connection()


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

class Grid25:
    def __init__(self, data, safety_distance):
        self.data = data
        self.safety_distance = safety_distance
        self._grid25 = []
        self.shape = (0,0)

        # top, bottom coordinates
        north = np.array([np.floor(data[:, 0] - data[:, 3]), 
                        np.ceil(data[:, 0] + data[:, 3])]).T

        # left, right coordinates
        east = np.array([np.floor(data[:, 1] - data[:, 4]),
                        np.ceil(data[:, 1] + data[:, 4])]).T

        self.height = np.array(np.ceil(data[:, 2] + data[:, 5]))

        self.north_min_max = (int(np.floor(np.amin(north[:, 0]))),
                            int(np.ceil(np.amax(north[:, 1]))))

        self.east_min_max = (int(np.floor(np.amin(east[:, 0]))),
                            int(np.ceil(np.amax(east[:, 1]))))

        self.create()

    def __len__(self):
        return len(self._grid25)

    @property
    def grid25(self):
        return self._grid25

    def __getitem__(self, tup):
        x, y = tup
        return self._grid25[x, y]

    def get_map_coord(self, n, e):
        n = n - self.north_min_max[0]
        e = e - self.east_min_max[0]
        return self._grid25[n, e]

    def create(self):
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

        # initialize empty grid
        grid25 = np.zeros((north_size, east_size))

        # set each grid cell to the height of obstacle for all places where this is an obstacle
        for i in range(0, obstacles.shape[0]):
            grid25[o_north_min[i] : o_north_max[i], o_east_min[i] : o_east_max[i]] = self.height[i]
        
        # save grid for further use
        self._grid25 = grid25
        self.shape = (north_size, east_size)

        print("load_map: Data North Min: {}, Max: {}".format(self.north_min_max[0], self.north_min_max[1]))
        print("load_map: Data East Min: {}, Max: {}".format(self.east_min_max[0], self.east_min_max[1]))

    def create_binary(self, drone_altitude):
        grid25_copy = np.array(self._grid25)

        # mark all obstacles that are below the drone's height as 0
        ok = grid25_copy < (drone_altitude - self.safety_distance)
        grid25_copy[ok] = 0
        
        # mark all obstacles that are above the drone's height as 1
        grid25_copy[~ok] = 1 

        return grid25_copy

class Grid3d(Grid25):
    def __init__(self, data, safety_distance, voxel_size = 5):
        super.__init__(data, safety_distance)

        self.voxel_size = voxel_size
        self.create_3d()
    
    def create_3d(self):
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
        north_size = int(np.ceil((north_max - north_min))) // self.voxel_size
        east_size = int(np.ceil((east_max - east_min))) // self.voxel_size
        alt_size = int(alt_max) // self.voxel_size

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
        
        o_north_min = np.clip(o_north_min // self.voxel_size, 0, north_size - 1).astype(int)
        o_north_max = np.clip(o_north_max // self.voxel_size, 0, north_size - 1).astype(int)
        o_east_min = np.clip(o_east_min // self.voxel_size, 0, east_size - 1).astype(int)
        o_east_max = np.clip(o_east_max // self.voxel_size, 0, east_size - 1).astype(int)
        o_alt = np.clip(o_alt_max // self.voxel_size, 0, alt_size - 1).astype(int)
        
        # set grid to 1 for all places where this is an obstacle
        for i in range(0, obstacles.shape[0]):
            voxmap[o_north_min[i] : o_north_max[i] + 1, 
                o_east_min[i] : o_east_max[i] + 1,
                0 : o_alt[i] + 1] = 1

        return voxmap

class WorldMap():
    def __init__(self, filename = 'colliders.csv', safety_distance = 3):
        self.filename = filename
        self.safety_distance = safety_distance
        self.home_gps_pos = GpsLocation(0,0,0)
        self.is_loaded = False
        self.grid = None                    # Grid25 class object
        self.data = []
        self.north_min_max = (0,0)
        self.east_min_max = (0,0)

    @property
    def loaded(self):
        return self.is_loaded

    @property
    def grid25(self):
        return self.grid

    @property
    def home_gps(self):
        return self.home_gps_pos

    @property
    def north_min(self):
        return self.north_min_max[0]

    @property
    def east_min(self):
        return self.east_min_max[0]

    def get_binary_grid(self, drone_altitude):
        return self.grid.create_binary(drone_altitude)
        # grid = self.create_grid25_forheight(drone_altitude)
        # grid[grid > 0] = 1
        # return grid

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
        self.data = np.loadtxt(self.filename, delimiter=',', dtype='Float64', skiprows=2)

        self.grid = Grid25(self.data, self.safety_distance)

        self.north_min_max = self.grid.north_min_max
        self.east_min_max = self.grid.east_min_max

        self.is_loaded = True