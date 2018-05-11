import time
import re
import numpy as np
import networkx as nx

from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree
from planning_utils import GpsLocation, WorldMap, Plot
from scipy.spatial import Voronoi, voronoi_plot_2d
from bresenham import bresenham

class Planner():
    def __init__(self):
        self.init_gps = GpsLocation(0,0,0)
        self.home_gps_pos = GpsLocation(0,0,0)
        self.safety_distance = 3
        self.worldmap = WorldMap('colliders.csv', self.safety_distance)
        self.min_drone_altitude = 5
        self.max_drone_altitude = 20
        self.graph_nodes = None     # KDTree
        self.graph = None           # nx.Graph

    @property
    def north_min(self):
        return self.worldmap.north_min_max[0]

    @property
    def east_min(self):
        return self.worldmap.east_min_max[0]
        
    def set_initial_gps(self, pos):
        self.init_gps = pos

    def load_map(self):
        self.worldmap.load()
        self.home_gps_pos = self.worldmap.home_gps_pos
        self.create_obstacle_tree()

    def create_obstacle_tree(self):
        data = self.worldmap.data
        collision_index = self.worldmap.height > self.min_drone_altitude - self.safety_distance

        # all obstacles that we can collide with
        data_collidable = data[collision_index]
        centers = np.dstack((data_collidable[:, 0], data_collidable[:, 1])).squeeze()

        self.obstacle_tree = KDTree(centers)

    def create_grid(self):
        return self.worldmap.create_grid_forheight(self.min_drone_altitude)

    def create_voronoi_graph(self):
        edges = self.get_voronoi_edges()

        n_min = self.north_min
        e_min = self.east_min

        # Voronoi edges are 0 based as per the grid that was given to it
        # we need to convert it back to world coordinates in the map
        edges_w = []
        for p1, p2 in edges:
            p1_w = (p1[0] + n_min, p1[1] + e_min)
            p2_w = (p2[0] + n_min, p2[1] + e_min)
            edges_w.append((p1_w, p2_w))

        self.graph = nx.Graph()
        for p1, p2 in edges_w:
            self.graph.add_edge(tuple(p1), tuple(p2))

        self.graph_nodes = KDTree(self.graph.nodes)
        return self.graph

    # def prune_edges(self, edges):
    #     if path is None:
    #         return path

    #     pruned_path = []
        
    #     # TODO: this following should have been a one liner numpy multiply
    #     pp = make_3d(path)
    #     p1 = pp[0]
    #     p2 = pp[1]
    #     p3 = pp[2]

    #     print(p1)
    #     print(p2)
    #     print(p3)
    #     pruned_path.append(p1)

    #     for i in range(2, len(pp)):
    #         p3 = pp[i]
    #         if collinearity_check(p1, p2, p3):
    #             p2 = p3
    #         else:
    #             pruned_path.append(p2)
    #             p1 = p2
    #             p2 = p3
        
    #     pruned_path.append(p3)
    #     return np.delete(np.array(pruned_path), 2, axis=1)


    def generate_receding_plan(self):
        """This generates a plan using 2d map"""
        graph = self.create_graph()

    def get_voronoi_edges(self):
        # create 0 based north, east as voronoi can't work on < 0 indices
        north_min = self.north_min
        east_min = self.east_min

        north = self.worldmap.data[:, 0] - north_min
        east = self.worldmap.data[:, 1] - east_min
        
        points = np.dstack((north, east)).squeeze()
        
        # create a voronoi graph out of the center points of each obstacle given in the map
        vgraph = Voronoi(points)
        vertices = vgraph.vertices.astype(int)

        # figure out the vertices that are out of bounds (< 0) and remove them from
        # computation by setting them to -1, -1
        nob = vertices[:, 0] < 0
        eob = vertices[:, 1] < 0
        out_of_bounds = np.where(nob | eob)
        vertices[out_of_bounds] = np.array([-1, -1])
        
        # set all vertices that are > grid max to -1,-1
        nob = vertices[:, 0] >= self.worldmap.grid_size[0]
        eob = vertices[:, 1] >= self.worldmap.grid_size[1]
        out_of_bounds = np.where(nob | eob)
        vertices[out_of_bounds] = np.array([-1, -1])
        
        grid = self.worldmap.create_grid_forheight(self.min_drone_altitude)
        edges = []
        
        for e in vgraph.ridge_vertices:
            p1 = vertices[e[0]]
            p2 = vertices[e[1]]
            
            collision = False

            # do not consider a vertex that is out of bounds
            if p1[0] == -1 or p2[0] == -1:
                collision = True
            else:
                # check in case there is a collision between the two
                # points given by the vertices in the graph
                cells = bresenham(p1[0], p1[1], p2[0], p2[1])
                for c in cells:
                    if grid[c[0], c[1]] == 1:
                        collision = True
                        break

            if not collision:
                # change coordinates back to NED format
                p1_world = (p1[0] - north_min, p1[1] - east_min)
                p2_world = (p2[0] - north_min, p2[1] - east_min)
                edges.append((p1_world, p2_world))
        
        return edges


if __name__ == "__main__":
    import time
    from planning_utils import Plot
    from PIL import Image
    import matplotlib.pyplot as plt

    planner = Planner()
    planner.load_map()

    print("map loaded")

    def test_image(grid):
        im = Image.fromarray(np.uint8(np.flip(grid, 0) * 255))
        im = Image.open('test.png')
        print('Grid has been saved to test.png')

    def test_grid():
        p = Planner()
        
        print('Loading map...')
        p.load_map()

        print('Creating grid...')
        grid = p.create_grid()

        print('Plotting grid...')
        fig = plt.figure()
        plt.imshow(grid, origin='lower', cmap='Greys') 
        plt.show()

    def test_voronoi():
        edges = planner.get_voronoi_edges()
        print(len(edges))
        
        fig = plt.figure()

        grid = planner.create_grid()
        plt.imshow(grid, origin='lower', cmap='Greys') 

        for p1, p2 in edges:
            plt.plot([p1[1], p2[1]], 
                    [p1[0], p2[0]], 'b-')

        plt.show()

    def test_graph():
        g = planner.create_voronoi_graph()
        grid = planner.create_grid()

        north_min = planner.north_min
        east_min = planner.east_min

        fig = plt.figure()
        plt.imshow(grid, cmap='Greys', origin='lower')
        
        for (n1, n2) in g.edges:
            n1_i = (n1[0] - north_min, n1[1] - east_min)
            n2_i = (n2[0] - north_min, n2[1] - east_min)
            
            plt.plot([n1_i[1], n2_i[1]], [n1_i[0], n2_i[0]], color='blue')

        plt.show()

    test_voronoi()

    # --- show grid
    #p = Plot()
    #p.show_grid(grid, planner.min_drone_altitude)


    # -- 3d graph print check
    #grid3d = planner.worldmap.create_grid3d(5)
    #print(grid3d.shape)

    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax.voxels(grid3d, edgecolor='k')
    # ax.set_xlim(grid3d.shape[0], 0)
    # ax.set_ylim(0, grid3d.shape[1])
    # # add 100 to the height so the buildings aren't so tall
    # ax.set_zlim(0, grid3d.shape[2]+20)

    # plt.xlabel('North')
    # plt.ylabel('East')

    # plt.show()