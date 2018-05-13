import time
import re
import numpy as np
import networkx as nx

from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree
from planning_utils import GpsLocation, WorldMap, Plot
from scipy.spatial import Voronoi, voronoi_plot_2d
from bresenham import bresenham
from queue import PriorityQueue
from udacidrone.frame_utils import global_to_local

def distance_heuristic(n1, n2):
    return np.linalg.norm((n1, n2))

class Planner():
    def __init__(self):
        self.init_gps = GpsLocation(0,0,0)
        self.home_gps_pos = GpsLocation(0,0,0)
        self.safety_distance = 3
        self.worldmap = WorldMap('colliders.csv', self.safety_distance)
        self.min_drone_altitude = 5
        self.max_drone_altitude = 20
        self.graph_nodes_kd = None     # KDTree
        self.graph = None           # nx.Graph
        self.start3d = (0, 0, 0)
        self.goal3d = (0, 0, 0)

        self.STATE_RADIUS = 10.0

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
        centers = centers.astype(int)

        self.obstacle_tree = KDTree(centers)

    def plan_route(self, start3d, goal3d):
        self.start3d = start3d
        self.goal3d = goal3d

        # self.graph will contain the graph
        # self.graph_nodes_kd is a KDTree to find the nearest nodes
        self.create_voronoi_graph()

        # find the closes node to the start and goal
        # indices = self.graph_nodes_kd.query_radius([start, goal], 
        #                         r = self.STATE_RADIUS, 
        #                         return_distance = False)

        indices = self.graph_nodes_kd.query([start3d, goal3d], 
                                k = 1, 
                                return_distance = False)

        # print('closest to goal indices:', indices)
        # print('index 0:', indices[0])
        # print('index 1:', indices[1])

        # find the closest node to the start and goal states

        graph_list = list(self.graph)

        c_start = graph_list[indices[0][0]]
        c_goal = graph_list[indices[1][0]]

        # print('Closest Start: ', c_start)
        # print('Closest Goal: ', c_goal)

        # print('Running a_star')
        path, path_cost = self.a_star_graph(self.graph, distance_heuristic, c_start, c_goal)
        print('a_star returned {} nodes'.format(len(path)))

        # prune path
        print('Pruning path...')
        pruned_path = self.prune_path(path)

        print('Path pruned. Length: ', len(pruned_path))

        return pruned_path, path_cost


    def create_grid(self):
        return self.worldmap.create_grid_forheight(self.min_drone_altitude)

    def convert_edges_tomap(self, edges):
        n_min = self.north_min
        e_min = self.east_min

        # Voronoi edges are 0 based as per the grid that was given to it
        # we need to convert it back to world coordinates in the map
        edges_w = []
        for p1, p2 in edges:
            p1_w = (p1[0] + n_min, p1[1] + e_min)
            p2_w = (p2[0] + n_min, p2[1] + e_min)
            edges_w.append((p1_w, p2_w))
        
        return edges_w

    def create_voronoi_graph(self):
        edges = self.get_voronoi_edges()
        edges_w = self.convert_edges_tomap(edges)

        self.graph = nx.Graph()

        # add altitude to each 2d edge returned by voronoi edges
        for p1, p2 in edges_w:
            p1_3d = (p1[0], p1[1], -self.min_drone_altitude)
            p2_3d = (p2[0], p2[1], -self.min_drone_altitude)
            
            self.graph.add_edge(p1_3d, p2_3d, weight=distance_heuristic(p1_3d, p2_3d))

        self.graph_nodes_kd = KDTree(self.graph.nodes)
        return self.graph

    # def point(self, p):
    #     return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity_check(self, p1, p2, p3, epsilon=1e-6):   
        m = np.vstack((p1, p2, p3))
        det = np.linalg.det(m)
        return abs(det) < epsilon

    def a_star_graph(self, graph, heuristic, start, goal):
        """Modified A* to work with NetworkX graphs."""
        path = []
        path_cost = 0

        queue = PriorityQueue()
        queue.put((0, start))
        visited = set(start)

        branch = {}
        found = False
        
        while not queue.empty():
            item = queue.get()
            current_node = item[1]
            
            if current_node == start:
                current_cost = 0.0
            else:              
                current_cost = branch[current_node][0]
                
            if current_node == goal:        
                print('Found a path.')
                found = True
                break
            else:
                #for action in valid_actions(grid, current_node):
                graph_node = graph[current_node]
                
                for next_node in graph_node:
                    if next_node not in visited:                
                        weight = graph_node[next_node]['weight']

                        branch_cost = current_cost + weight
                        queue_cost = branch_cost + heuristic(next_node, goal)
                    
                        visited.add(next_node)               
                        branch[next_node] = (branch_cost, current_node)
                        queue.put((queue_cost, next_node))
                
        if found:
            # retrace steps
            n = goal
            path_cost = branch[n][0]
            path.append(goal)

            while branch[n][1] != start:
                path.append(branch[n][1])
                n = branch[n][1]

            path.append(branch[n][1])
        else:
            print('**********************')
            print('Failed to find a path!')
            print('**********************') 
        return path[::-1], path_cost

    def prune_path(self, pp):
        if pp is None:
            return pp

        pruned_path = []
        
        p1 = pp[0]
        p2 = pp[1]
        p3 = pp[2]

        print(p1)
        print(p2)
        print(p3)
        pruned_path.append(p1)

        for i in range(2, len(pp)):
            p3 = pp[i]
            if self.collinearity_check(p1, p2, p3):
                p2 = p3
            else:
                pruned_path.append(p2)
                p1 = p2
                p2 = p3
        
        pruned_path.append(p3)
        # delete the 3rd dimension
        #return np.delete(np.array(pruned_path), 2, axis=1)
        return pruned_path

    def generate_receding_plan(self):
        """This generates a plan using 2d map"""
        pass

    def get_voronoi_edges(self):
        """returns voronoi edges in the image coordinates"""
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
                edges.append((p1, p2))
        
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
        print('test_voronoi')
        
        grid = planner.create_grid()
        edges = planner.get_voronoi_edges()

        print('Voronoi edges computed. Length: ', len(edges))

        n_min = planner.north_min
        e_min = planner.east_min
        
        fig = plt.figure()
        plt.subplot(121)
        plt.imshow(grid, origin='lower', cmap='Greys') 

        print('Plotting points...')

        for p1, p2 in edges:
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
    
        print('Converting into map coordinates..')
        edges_w = planner.convert_edges_tomap(edges)

        print('Plotting map points...')

        plt.subplot(122)
        plt.imshow(grid, origin='lower', cmap='Greys') 
        
        for p1, p2 in edges_w:
            p1_i = p1[0] - n_min, p1[1] - e_min
            p2_i = p2[0] - n_min, p2[1] - e_min
            plt.plot([p1_i[1], p2_i[1]], [p1_i[0], p2_i[0]], color='red')
        
        plt.show()

    def test_graph():
        print('test_graph')
        grid = planner.create_grid()

        print('Creating graph...')
        g = planner.create_voronoi_graph()

        n_min = planner.north_min
        e_min = planner.east_min

        # print('Plotting graph...')

        # fig = plt.figure()
        # plt.imshow(grid, cmap='Greys', origin='lower')

        # for (n1, n2) in g.edges:
        #     n1_i = (n1[0] - n_min, n1[1] - e_min)
        #     n2_i = (n2[0] - n_min, n2[1] - e_min)
            
        #     plt.plot([n1_i[1], n2_i[1]], [n1_i[0], n2_i[0]], color='blue')

        # plt.show()
        s = []
        for n in g.nodes:
            s = n
            break

        print('Looking for ', s)
        # print(type(g.nodes))
        t = KDTree(g.nodes)
        print(t.query(s, k = 2))
        
        # s = (-n_min, -e_min)
        # print('Trying to find nearest of start state: ', s)
        # print(planner.graph_nodes_kd.query(s, k = 5))

    def test_plan_route():
        print('test_plan_route')

        start = planner.home_gps_pos
        goal = (-planner.north_min + 10, -planner.east_min + 10)

        ls = global_to_local(start)
        #self.planner.create_plan(start, goal)
        print("Start:")
        print(start)
        print(ls)

        print('Goal')
        print(goal)

    
    test_graph()
    #est_plan_route()

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