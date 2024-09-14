import numpy as np
import APF
import APF_func as af

start = np.array([0, 0, 0])
goal = np.array([0, 0, 0])
obstacles = np.array([0,0,0],
                     [0,0,0])
min_dist = 0.5 #minimum distance between the tool tip and the goal to stop the arm
apf = APF(start, goal, obstacles)

while np.linalg.norm(goal - start) <= min_dist:

    apf.move()
