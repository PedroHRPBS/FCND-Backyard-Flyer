from queue import PriorityQueue, Queue
import numpy as np
from enum import Enum
from math import sqrt, inf


class Action(Enum):
    """
    An action is represented by a 3 element tuple.
    
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    LEFTUP = (-1, -1, sqrt(2))
    LEFTDOWN = (1, -1, sqrt(2))
    RIGHTUP = (-1, 1, sqrt(2))
    RIGHTDOWN = (1, 1, sqrt(2))
    
    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
        elif self == self.LEFTUP:
            return 'X'
        elif self == self.LEFTDOWN:
            return 'X'
        elif self == self.RIGHTUP:
            return 'X'
        elif self == self.RIGHTDOWN:
            return 'X'
    
    @property
    def cost(self):
        return self.value[2]
    
    @property
    def delta(self):
        return (self.value[0], self.value[1])
            
    
def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN, Action.LEFTUP, Action.LEFTDOWN, Action.RIGHTUP, Action.RIGHTDOWN]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node
    
    # check if the node is off the grid or
    # it's an obstacle
    
    if x - 1 < 0 or grid[x-1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x+1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y-1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y+1] == 1:
        valid.remove(Action.RIGHT)
    if (x - 1 < 0 or y - 1 < 0) or grid[x-1, y-1] == 1:
        valid.remove(Action.LEFTUP)
    if (x + 1 > n or y - 1 < 0) or grid[x+1, y-1] == 1:
        valid.remove(Action.LEFTDOWN)
    if (x - 1 < 0 or y + 1 > m) or grid[x-1, y+1] == 1:
        valid.remove(Action.RIGHTUP)
    if (x + 1 > n or y + 1 > m) or grid[x+1, y+1] == 1:
        valid.remove(Action.RIGHTDOWN)
        
    return valid

def valid_actions_bfs(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN, Action.LEFTUP, Action.LEFTDOWN, Action.RIGHTUP, Action.RIGHTDOWN]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node
    
    # check if the node is off the grid or
    # it's an obstacle
    
    if grid[x-1, y] == 1:
        valid.remove(Action.UP)
    if grid[x+1, y] == 1:
        valid.remove(Action.DOWN)
    if grid[x, y-1] == 1:
        valid.remove(Action.LEFT)
    if grid[x, y+1] == 1:
        valid.remove(Action.RIGHT)
    if grid[x-1, y-1] == 1:
        valid.remove(Action.LEFTUP)
    if grid[x+1, y-1] == 1:
        valid.remove(Action.LEFTDOWN)
    if grid[x-1, y+1] == 1:
        valid.remove(Action.RIGHTUP)
    if grid[x+1, y+1] == 1:
        valid.remove(Action.RIGHTDOWN)
        
    return valid

def visualize_path(grid, path, start):
    sgrid = np.zeros(np.shape(grid), dtype=np.str)
    sgrid[:] = ' '
    sgrid[grid[:] == 1] = 'O'
    
    pos = start
    
    for a in path:
        da = a.value
        sgrid[pos[0], pos[1]] = str(a)
        pos = (pos[0] + da[0], pos[1] + da[1])
    sgrid[pos[0], pos[1]] = 'G'
    sgrid[start[0], start[1]] = 'S'  
    return sgrid

# TODO: implement a heuristic function. This may be one of the
# functions described above or feel free to think of something
# else.
def heuristic_manhattan(position, goal_position):
    h = abs(position[0] - goal_position[0]) + abs(position[1] - goal_position[1])
    return h

def heuristic_euclidean(position, goal_position):
    h = sqrt(pow((position[0] - goal_position[0]),2) + pow((position[1] - goal_position[1]),2))
    return h

def a_star(grid, h, start, goal):

    path = []
    steps = []
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
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                # TODO: calculate branch cost (action.cost + g)
                # TODO: calculate queue cost (action.cost + g + h)
                branch_cost = action.cost + current_cost
                queue_cost = branch_cost + h(next_node,goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))    
                    
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            steps.append(branch[n][2])
            n = branch[n][1]
        path.append(branch[n][1])
        steps.append(branch[n][2])
 
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
        
    return path[::-1], path_cost, steps

def a_star_bfs(grid, h, start, goal):

    path = []
    steps = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)
    width = np.shape(grid)[1]

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
            for action in valid_actions_bfs(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                # TODO: calculate branch cost (action.cost + g)
                # TODO: calculate queue cost (action.cost + g + h)
                branch_cost = action.cost + current_cost
                queue_cost = branch_cost + h[next_node[0] + next_node[1] * width]
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))    
                    
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            steps.append(branch[n][2])
            n = branch[n][1]
        path.append(branch[n][1])
        steps.append(branch[n][2])
 
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
        
    return path[::-1], path_cost, steps


def breadth_first(grid, start, goal):
    q = Queue()
    q.put((0, start))
    visited = set(start)
    branch = {}
    found = False
    width = np.shape(grid)[1]
    mapa = np.zeros(np.shape(grid)[0]-1 + (np.shape(grid)[1]-1) * width + 1)
    mapa[mapa > -1] = inf 

    while not q.empty(): 
        item = q.get()
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
            valid = valid_actions_bfs(grid, current_node)
            for action in valid:
                da = action.value
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = action.cost + current_cost
                mapa[current_node[0] + current_node[1] * width] = current_cost
                if next_node not in visited:
                    visited.add(next_node)
                    q.put((branch_cost, next_node))    
                    branch[next_node] = (branch_cost, current_node, action)
   
    print(current_cost)
    return mapa

start = (4, 5)
goal = (0, 2)

grid = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 1, 0],
    [0, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0],
])

path, cost, steps = a_star(grid, heuristic_euclidean, start, goal)
#print(cost, path)

# S -> start, G -> goal, O -> obstacle
visualize_path(grid, steps, start)

path, cost, steps = a_star(grid, heuristic_manhattan, start, goal)
#print(cost, path)

# S -> start, G -> goal, O -> obstacle
visualize_path(grid, steps, start)