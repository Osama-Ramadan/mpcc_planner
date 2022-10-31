from sys import path
import numpy as np

#Import CasADi 
path.append(r"/home/casadi-linux-py36-v3.5.5-64bit")
from casadi import *

class IGONEO_DW():
    
    def __init__(self):
        
        self.max_speed = 0
        self.min_speed = 0
        self.max_omega = 0
        self.min_omega = 0
        self.max_acc = 0
        self.max_alpha = 0
        self.v_resolution = 0
        self.omega_resolution = 0
        self.dt = 0
        self.pred_horizon = 0
        
        self.heading_cost_gain = 0
        self.to_goal_cost_gain = 0
        self.speed_cost_gain = 0
        self.obstacle_cost_gain = 0
        self.dynamic_obstacle_cost_gain = 0
        
        self.igoneo_raduis = 0
        self.igoneo_length = 0
        
        self.n_states = 6
        self.n_inputs = 2
        
        self.driving_direction = "b"
        
    def set_driving_direction(self,direction):
        self.driving_direction = direction
                
    def set_dw_configuration(self,max_speed,min_speed,max_omega,min_omega,max_acc,max_alpha,v_resolution,omega_resolution,dt,pred_horizon):
        
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.max_omega = max_omega
        self.min_omega = min_omega
        self.max_acc = max_acc
        self.max_alpha = max_alpha
        self.v_resolution = v_resolution
        self.omega_resolution = omega_resolution
        self.dt = dt
        self.pred_horizon = pred_horizon
    
    def set_gains(self,heading_gain,obstacle_gain,speed_gain,to_goal_gain, dynamic_obst_gain):
        
        self.heading_cost_gain = heading_gain
        self.to_goal_cost_gain = to_goal_gain
        self.speed_cost_gain = speed_gain
        self.obstacle_cost_gain = obstacle_gain
        self.dynamic_obstacle_cost_gain = dynamic_obst_gain
    
    def set_igoneo_parameters(self,raduis,length, wheel_r):
        
        self.igoneo_raduis = raduis
        self.igoneo_length = length
        self.wheel_r = wheel_r
    
    def get_dw_configuration(self):
        pass
    
    def forward_step(self,x,u):
        next = np.zeros((self.n_states,1))
        theta = float(x[2])
        gamma = float(x[3])
        A = np.array([[np.cos(gamma+theta) , 0],
                     [np.sin(gamma+theta) , 0],
                     [-np.sin(gamma)/self.igoneo_length, 0],
                     [0 , 1]])
        d_x = (A@u)*self.dt
        next[0:4] = x[0:4] + d_x
        next[4] = next[0]-(self.igoneo_length*np.cos(next[2]))
        next[5] = next[1]-(self.igoneo_length*np.sin(next[2]))
        return next
    
    def get_dw(self,u):
        Vs = [self.min_speed, self.max_speed, self.min_omega, self.max_omega]
        Vd = [u[0]-self.max_acc*self.dt, u[0]+self.max_acc*self.dt, u[1]-self.max_alpha*self.dt, u[1]+self.max_alpha*self.dt]
        
        Dw = [float(max(Vs[0],Vd[0])), float(min(Vs[1],Vd[1])), float(max(Vs[2],Vd[2])), float(min(Vs[3],Vd[3]))]
        
        return Dw
    
    def compute_trajectory(self,x0,u):
        traj = [x0]
        t = 0
        x = x0
        while t <= self.pred_horizon:
            x = self.forward_step(x,u)
            traj.append(x)
            t = t + self.dt
        return traj
    
    def compute_obstacle_cost(self,trajectory,map):
        cost_value = 0
        min_r = 1000
        min_r_b = 1000
        for traj in trajectory:
            obst_list_f = map.get_obst_around_pose(traj[0:2],1,1,1,1)
            obst_list_b = map.get_obst_around_pose(traj[4:6],1,1,1,1)

            if len(obst_list_f)==0 and len(obst_list_b)==0:
                return cost_value
            
            # Forward point
            if len(obst_list_f)!=0:
                ox = obst_list_f[:,0]
                oy = obst_list_f[:,1]
                dx = np.subtract(traj[0],ox)
                dy = np.subtract(traj[1],oy)
                r = np.sqrt(dx**2+dy**2)
                min_r = np.min(r)
            #backward point
            if len(obst_list_b)!=0:
                ox = obst_list_b[:,0]
                oy = obst_list_b[:,1]
                dx_b = np.subtract(traj[4],ox)
                dy_b = np.subtract(traj[5],oy)
                r_b = np.sqrt(dx_b**2+dy_b**2)
                min_r_b = np.min(r_b)

            if min_r < 0.3 or min_r_b < 0.3:
                return np.inf

        return 1/min_r + 1/min_r_b
    
    def compute_dynamic_obstacle_cost(self,trajectory,obst_trajectory):
        cost_value = 0
        obst_pose = obst_trajectory[0]
        for traj in trajectory:
            if self.driving_direction == "f":
                dist = np.sqrt((traj[0]-obst_pose[0])**2 +(traj[1]-obst_pose[1])**2)
            else :
                dist = np.sqrt((traj[4]-obst_pose[0])**2 +(traj[5]-obst_pose[1])**2)

            if dist < 2 :
                for i in range (len(obst_trajectory)):
                    obst_traj = obst_trajectory[i]
                    dist = np.sqrt((traj[0]-obst_traj[0])**2 +(traj[1]-obst_traj[1])**2)
                    cost_value = cost_value + (len(obst_trajectory)-i)/(dist+0.001)
        return cost_value
    
    def compute_to_goal_cost(self, trajectory, goal):
        cost = 0
        for traj in trajectory:
            traj= traj.reshape(6,1)
            goal = goal.reshape(2,1)
            if self.driving_direction =="f":
                cost = cost + np.sqrt(np.sum((goal[0:2] - traj[0:2]) ** 2, axis=0))
            else:
                cost = cost + np.sqrt(np.sum((goal[0:2] - traj[4:6]) ** 2, axis=0))
        return cost
    
    def compute_heading_cost(self,trajectory, goal):
        
        for traj in trajectory:
            if self.driving_direction =="f":
                dx = goal[0] - traj[0]
                dy = goal[1] - traj[1]
            else:
                dx = goal[0] - traj[4]
                dy = goal[1] - traj[5]
        
        error_angle = np.arctan2(dy,dx) - traj[2]
        cost = abs(np.arctan2(np.sin(error_angle),np.cos(error_angle)))
        
        return cost
      
    def compute_control_trajectory(self,x,dw,goal,map, obst_traj_f, obst_traj_b):
        x_init = x
        min_cost = np.inf
        opt_u = np.zeros((self.n_inputs,1))
        opt_traj = x
                
        for v in np.round(np.arange(dw[0],dw[1]+self.v_resolution,self.v_resolution),4):
            for omega in np.round(np.arange(dw[2],dw[3]+self.omega_resolution,self.omega_resolution),4):
                trajectory = self.compute_trajectory(x_init,np.array([[v],[omega]]))
                
                heading_cost = float(self.heading_cost_gain*self.compute_heading_cost(trajectory,goal))
                speed_cost = self.speed_cost_gain*(abs(self.max_speed-v))
                obst_cost = self.obstacle_cost_gain*self.compute_obstacle_cost(trajectory,map)
                to_goal_cost = self.to_goal_cost_gain*self.compute_to_goal_cost(trajectory,goal)
                
    
                dynamic_obst_cost_f = self.dynamic_obstacle_cost_gain*self.compute_dynamic_obstacle_cost(trajectory,obst_traj_f)
                dynamic_obst_cost_b = self.dynamic_obstacle_cost_gain*self.compute_dynamic_obstacle_cost(trajectory,obst_traj_b)
                

                final_cost = heading_cost + speed_cost + obst_cost + to_goal_cost + dynamic_obst_cost_f + dynamic_obst_cost_b
            
                if min_cost >= final_cost:
                    min_cost = final_cost
                    opt_u[0] = v
                    opt_u[1] = omega
                    opt_traj = trajectory
        if min_cost == np.inf:
            return[[float(0.0),float(0.0)],[],np.inf]
        return [opt_u,opt_traj,min_cost]