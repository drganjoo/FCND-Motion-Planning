import heapq
import numpy as np
import time
from enum import Enum
from queue import Queue
from queue import PriorityQueue

class CubicAction(Enum):
    GO_NORTH = (-1, 0, 0, 1)
    GO_SOUTH = (1, 0, 0, 1)
    GO_EAST = (0, 1, 0, 1)
    GO_WEST = (0, -1, 0, 1)
    # GO_NE = (-1, 1, 0, np.sqrt(2))
    # GO_NW = (-1, -1, 0, np.sqrt(2))
    # GO_SW = (1, -1, 0, np.sqrt(2))
    # GO_SE = (1, 1, 0, np.sqrt(2))
    GO_UP = (0, 0, -1, 1)
    GO_DOWN = (0, 0, 1, 1)
        
    def __str__(self):
        if self == CubicAction.GO_NORTH:
            return '↑'
        elif self == CubicAction.GO_SOUTH:
            return '↓'
        elif self == CubicAction.GO_EAST:
            return '→'
        elif self == CubicAction.GO_WEST:
            return '←'
        # elif self == CubicAction.GO_NW:
        #     return '↖'
        # elif self == CubicAction.GO_NE:
        #     return '↗'
        # elif self == CubicAction.GO_SE:
        #     return '↘'
        # elif self == CubicAction.GO_SW:
        #     return '↙'
        elif self == CubicAction.GO_UP:
            return 'U'
        else:
            return 'D'
    
    @property
    def cost(self):
        return self.value[3]

    @property
    def delta(self):
        return (self.value[0], self.value[1], self.value[2])


def distance_euclidean(x, y):
    return np.linalg.norm((x,y))

def distance_manhattan(x, y):
    return np.abs(y[2] - x[2]) + np.abs(y[1] - x[1]) + np.abs(y[0] - x[0])


class ActionPlanner:
    def __init__(self, grid25, max_altitude):
        self.max_altitude = max_altitude
        self.grid25 = grid25

    # def average(self, north, east):
    #     cells = [(-1,0), (-1,-1), (0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1)]
    #     sum = self.grid25[north, east]
    #     for c in cells:
    #         sum += self.grid25[north + c[0], east + c[1]]
    #     return sum / 9

    def find_path(self, start, goal):
        start = (int(start[0]), int(start[1]), int(start[2]))
        goal = (int(goal[0]), int(goal[1]), int(goal[2]))

        # in case the goal is an obstacle, we will try to land on top of it
        height = self.grid25.get_map_coord(goal[0], goal[1])
        if height != 0.0:
            print('Goal state is an obstacle. Will land on top of it')

            # make sure that we are not at the edge of the building
            # if average(goal[0], goal[1]) - height > 0.1:
            #     pass
            goal = (goal[0], goal[1], -int(height))

        path = self.a_star(distance_manhattan, goal, start)

        # since we looked from goal to start, lets switch around the path
        path = path[::-1]
        return path

    def valid_actions(self, node):
        last_row, last_col = self.grid25.shape
        cells = []

        # print('Valid Actions for {}'.format(node))

        for a in list(CubicAction):
            temp = (node[0] + a.value[0], node[1] + a.value[1], node[2] + a.value[2])
            
            # print('Possible Action {} -> {}'.format(a, temp))

            # check grid cell is valid or not. Any cell within the bounds of the grid and
            # maximum altitude within allowed range is considered to be ok cell

            if 0 <= temp[0] < last_row and 0 <= temp[1] < last_col and temp[2] <= self.max_altitude:
                # print('Within limits')

                obs_height = self.grid25.get_map_coord(temp[0],temp[1])

                # node altitude is -ve
                if obs_height == 0.0 or obs_height < -temp[2]:
                    # print('\t\tWithin limits {}'.format(a))
                    cells.append(((temp), a))
                # else:
                    # print('grid25 has higher height {}', self.grid25[temp[0]][temp[1]])
                
        return cells

    def a_star(self, h, start, goal):
        # make sure that start and goal are int
        start = (int(start[0]), int(start[1]), int(start[2]))
        goal = (int(goal[0]), int(goal[1]), int(goal[2]))

        queue = PriorityQueue()
        visited = set()
        branch = {}
        
        queue.put((0, start))
        visited.add(start)
        current_cost = 0
        
        # print('*' * 50)
        print("Start: {}, Goal: {}".format(start, goal))

        s = time.time()

        while queue.qsize() > 0:
            if time.time() - s > 5:
                print("Cubic Actions timing out!!")
                break

            _, current_node = queue.get()

            if current_node == start:
                current_cost = 0.0
            else:              
                current_cost = branch[current_node][0]

            # print("Current Cost: {}, Current: {}".format(current_cost, current_node))

            if current_node == goal:
                print("Current equals goal, going out")
                break
            else:
                for next_node, action in self.valid_actions(current_node):
                    if next_node not in visited:
                        visited.add(next_node)
                        
                        branch_cost = current_cost + action.cost
                        branch[next_node] = (branch_cost, current_node, action)

                        h_cost = h(next_node, goal)
                        queue_cost = branch_cost + h_cost
                        queue.put((queue_cost, next_node))

                        #print("\t{} {} to {} BC: {} HC: {} QC: {}".format(action, current_node, next_node, branch_cost, h_cost, queue_cost))

            # # TODO debugging
            # if queue.qsize() > 100:
            #     break

        path = []

        if current_node != goal:
            print("!!!!Could not find path using CubicActions!!!")
            return path
        
        path_action = []
        path_cost = 0

        path.append(goal + (path_cost, ))

        while current_node != start:
            branch_cost, parent_node, action = branch[current_node]
            path_action.append(action)

            path_cost += branch_cost

            path.append(parent_node + (path_cost, ))
            current_node = parent_node

        return path[::-1]

# def visualize_path(grid, path, start, goal):
#     """
#     Given a grid, path and start position
#     return visual of the path to the goal.
    
#     'S' -> start 
#     'G' -> goal
#     'O' -> obstacle
#     ' ' -> empty
#     """
#     x = np.zeros(grid.shape, dtype=str)
#     x[:] = ' '
#     x[grid == 1] = 'O'
#     x[start] = 'S'
#     x[goal] = 'G'

#     #print(x)

#     n = start
#     for a in path:
#         x[n] = str(a)
#         n = (n[0] + a.value[0], n[1] + a.value[1])

#     x[start] = 'S'
#     x[goal] = 'G'
    
#     return x

# def uniform_cost(grid, start, goal):
#     queue = []
#     visited = set()
#     branch = {}
#     found = False
    
#     queue.append((0, start))
#     visited.add(start)
    
#     while len(queue) > 0:
#         cost, current = heapq.heappop(queue)

#         if current == goal:
#             break
#         else:
#             for next_node, action in valid_actions(grid, current):
#                 new_cost = cost + action.cost
                
#                 if next_node not in visited:
#                     visited.add(next_node)
#                     branch[next_node] = (new_cost, current, action)
#                     heapq.heappush(queue, (new_cost, next_node))

#     path = []
#     path_cost = 0

#     if current != goal:
#         return []
    
#     path_action = []
#     path_cost = branch[current][0]
    
#     while current != start:
#         node_cost, parent_node, action = branch[current]
#         path_action.append(action)
#         current = parent_node

#     return path_action[::-1], cost

# def breadth_first_search(grid, start, goal):
#     visited = set()
#     branch = {}     # keeps track of how we reach a state (parent, action)
#     q = Queue()

#     visited.add(start)
#     q.put(start)

#     while q.qsize() > 0:
#         current = q.get()
#         if current == goal:
#             break

#         possible = valid_actions(grid, current)

#         for p in possible:
#             next_node = p[0]
#             action = p[1]

#             if next_node not in visited:
#                 visited.add(next_node)
#                 branch[next_node] = (current, action)
#                 q.put(next_node)

#     if current != goal:
#         return []
#     else:
#         path_action = []

#         while current != start:
#             arrived_from = branch[current]
#             path_action.append(arrived_from[1])
#             current = arrived_from[0]

#         return path_action[::-1]    