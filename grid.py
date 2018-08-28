import numpy as np 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import Voronoi, voronoi_plot_2d
from bresenham import bresenham


#%matplotlib inline

plt.rcParams["figure.figsize"] = [12, 12]

filename = 'colliders.csv'
# Read in the data skipping the first two lines.  
# Note: the first line contains the latitude and longitude of map center
# Where is this??
data = np.loadtxt(filename,delimiter=',',dtype='Float64',skiprows=2)
print(data)

# Static drone altitude (metres)
drone_altitude = 5

# Minimum distance required to stay away from an obstacle (metres)
# Think of this as padding around the obstacles.
safe_distance = 3


def create_grid_bfs(data, drone_altitude, safety_distance):
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))
    
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

  
    north_size = int(np.ceil(north_max - north_min)) + 2
    east_size = int(np.ceil(east_max - east_min)) + 2
 
    grid = np.zeros((north_size, east_size))

    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])

    for i in range(north_size):
        grid[i,0] = 1
        grid[i,-1] = 1

    for i in range(east_size):
        grid[0,i] = 1
        grid[-1,i] = 1


    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obj_size = [int(np.ceil(d_north * 2)) + safety_distance * 2, 
                        int(np.ceil(d_east * 2)) + safety_distance * 2]

            for n in range(0, obj_size[0]):
                for m in range(0, obj_size[1]): 
                    x = np.clip(int(np.ceil(north - d_north - safety_distance - north_min)) + m, 0, north_size - 1)
                    y = np.clip(int(np.ceil(east - d_east - safety_distance - east_min)) + n, 0, east_size - 1)
                    grid[x,y] = 1
        

    return grid



def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    #Delimita o mínimo e o máximo da vertical considerando a presença do objeto
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))
    
    # minimum and maximum east coordinates
    #Delimita o mínimo e o máximo da horizontal considerndo a presença do objeto
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))
    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Center offset for grid
    # Computa o centro do obstaculo mais ao norte e leste
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    # Populate the grid with obstacles
    # data.shape = (n,m) onde n = numero de linhas e m = numero de colunas
    # data.shape[0] retorna n, ou seja, o numero de linhas
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obj_size = [int(np.ceil(d_north * 2)) + safety_distance * 2, 
                        int(np.ceil(d_east * 2)) + safety_distance * 2]

            for n in range(0, obj_size[0]):
                for m in range(0, obj_size[1]): 
                    x = np.clip(int(np.ceil(north - d_north - safety_distance - north_min)) + m, 0, north_size - 1)
                    y = np.clip(int(np.ceil(east - d_east - safety_distance - east_min)) + n, 0, east_size - 1)
                    grid[x,y] = 1
        

    return grid

# Here you'll modify the `create_grid()` method from a previous exercise
# In this new function you'll record obstacle centres and
# create a Voronoi graph around those points
def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min)))
    east_size = int(np.ceil((east_max - east_min)))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    
    # Define a list to hold Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(north - d_north - safety_distance - north_min_center),
                int(north + d_north + safety_distance - north_min_center),
                int(east - d_east - safety_distance - east_min_center),
                int(east + d_east + safety_distance - east_min_center),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
            
            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # TODO: create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)
    edges = []
    # TODO: check each edge from graph.ridge_vertices for collision
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        hit = False
        
        
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        
        for c in cells:
            if np.min(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            if grid[c] == 1:
                hit = True
                break
            
        if not hit:
            #Converter array para tuple
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))
            

    return grid, edges

grid = create_grid(data, drone_altitude, safe_distance)

# equivalent to
# plt.imshow(np.flip(grid, 0))
# NOTE: we're placing the origin in the lower lefthand corner here
# so that north is up, if you didn't do this north would be positive down
fig1 = plt.imshow(grid, origin='lower') 

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()
