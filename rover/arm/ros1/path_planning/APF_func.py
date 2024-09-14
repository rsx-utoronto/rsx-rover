import numpy as np
import random

def shortest_vector(point, point_cloud):
    #find the shortest distance between a point and a boundary

    # Calculate the distances from the single point to all points in the point cloud
    distances = np.linalg.norm(point_cloud - point, axis=1)

    # Find the shortest distance and the corresponding point
    closest_point = point_cloud[np.argmin(distances)]
    shortest_vector = closest_point - point

    return shortest_vector, closest_point

def brownian(cur_pos, vi, num_steps):
    '''
    This function simulates brownian motion,
    it will be used for getting out of local minima

    curpos: 1X3 numpy array
    vi: int
    num_steps: int

    returns a new cur_pos
    '''

    for i in range(num_steps):
        for a in range(len(cur_pos)):
            operation = random.choice([-1, 1])
            cur_pos[a] = cur_pos[a] + (operation*vi)
    
    return cur_pos