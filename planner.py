import re
import numpy as np
import networkx as nx

from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree
from planning_utils import GpsLocation, WorldMap, Plot

class Planner():
    def __init__(self):
        self.grid = []
        self.data = []
        self.north = []
        self.east = []
        self.north_min_max = (0,0)
        self.east_min_max = (0,0)
        self.init_gps = GpsLocation(0,0,0)
        self.home_gps_pos = GpsLocation(0,0,0)
        self.world_map = WorldMap('colliders.csv')
        
    def set_initial_gps(self, pos):
        self.init_gps = pos

    def load_map(self):
        self.world_map.load()

    def create_grid(self, drone_altitude):
        return self.world_map.create_grid(drone_altitude)

    def create_graph(self):
        pass


if __name__ == "__main__":
    import time
    from planning_utils import Plot
    from PIL import Image
    import matplotlib.pyplot as plt

    planner = Planner()
    planner.load_map()

    drone_height = 3

    # im = Image.fromarray(image)
    # im.save('check.png')

    #im = Image.fromarray(np.uint8(np.flip(grid, 0) * 255))
    #im = Image.open('test.png')
    #print(np.array(im).shape)

    # grid = planner.create_grid(drone_height)

    # p = Plot()
    # p.show_grid(grid, drone_height)

    grid3d = planner.world_map.create_grid3d(5)
    print(grid3d.shape)

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.voxels(grid3d, edgecolor='k')
    ax.set_xlim(grid3d.shape[0], 0)
    ax.set_ylim(0, grid3d.shape[1])
    # add 100 to the height so the buildings aren't so tall
    ax.set_zlim(0, grid3d.shape[2]+20)

    plt.xlabel('North')
    plt.ylabel('East')

    plt.show()

    # x = np.array([0,10,100])
    # y = np.array([10,20,30])

    # plt.plot(x, y)
    # plt.show()

