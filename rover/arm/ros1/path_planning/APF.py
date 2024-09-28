import math
import numpy as np
import APF_func as af


class APF():

    def _init_(self, start, goal, obstacles):
        
        '''
        start: 3X1 np.array
        goal: 3X1 np.array
        obstacles: list of 3X1 np.arrays?
        '''

        #APF parameters
        '''
        zeta & d: APF parameters for calculating F_att and F_rep
        time_step: used for calculating new position
        '''
        self.zeta = 1.0
        self.time_step = 0.1 #sec
        self.d = 1.0                      #min distance for F_att
        self.v = np.array([0.0, 0.0, 0.0])
        self.k = 1.0
        self.cur_pos = start
        self.goal = goal
        self.obstacles = obstacles
        self.eta = 1.0                    #F_rep scalar
        self.rho0 = 5.0                   #min rho for F_rep
        #parameters for random motions:
        self.vi = 0.005                   #size of random steps
        self.ran_steps = 5                #number of random steps


    def change_zeta(self, zeta):
        self.zeta = zeta
    
    def change_time_step(self, time_step):
        self.time_step = time_step
    
    def change_d(self, d):
        self.d = d
    
    def change_eta(self, eta):
        self.eta = eta

    def calc_dist(self):
        #calculate current distance from goal

        distance = np.linalg.norm(self.cur_pos - self.goal)

        return distance
    
    def calc_F_att(self):
        #calculate attractive force
        distance = self.calc_dist()

        if distance <= self.d:
            F_att = -self.zeta(self.cur_pos - self.goal)

        elif distance >= self.d:
            F_att = -self.d*self.zeta*(self.cur_pos - self.goal)

        return F_att
    
    def calc_rho(self):
        #rho is the distance from the point to the boundary
        rho, _ = af.shortest_vector(self.cur_pos, self.obstacles)

        return rho
    
    def calc_F_rep(self):
        #calculate repulsive force
        distance = np.linalg.norm(self.cur_pos, self.goal)
        rho = self.calc_rho()

        if np.linalg.norm(rho) >= np.linalg.norm(self.rho0):
            F_rep = 0

        elif np.linalg.norm(rho) <= np.linalg.norm(self.rho0):
            F_rep = self.eta*((1/rho)-(1/self.rho0))*(1/(rho**2))\
                (rho/np.linalg.norm(rho))
            
        return F_rep
    
    def calc_Ft(self):
        #calculate current total force

        Ft = self.calc_F_att + self.calc_F_rep

        return Ft
    
    def calc_v(self):
        #calculate v for the next time step

        self.v = self.k*self.calc_Ft()
    
    def move(self):
        self.cur_pos = self.cur_pos + self.v*self.time_step

    
    def escape_trap(self):
        self.cur_pos = af.brownian(self.cur_pos, self.vi, self.ran_steps)
        self.move()
    
