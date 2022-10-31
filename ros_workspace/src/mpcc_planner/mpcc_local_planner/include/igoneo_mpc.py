from cmath import sqrt
from math import gamma
from sys import path
import numpy as np

# Import CasADi 
path.append(r"/home/casadi-linux-py36-v3.5.5-64bit")
from casadi import *


# This Class define the dynmaic equation for the IgoNeo to be used in the optimizer to generate the predicted trajectory

class IGONEO():
    def __init__(self):
        self.f = self.define_model()
        self.predicted_trajectory = []

    # getter function to return the forward function
    def get_forward_function(self):
        return self.f
    
    def get_n_states(self):
        return self.n_states

    def get_n_controls(self):
        return self.n_controls

    def get_mpcc_solver(self):
        return self.solver

    def update_mpcc_solver(self,solver):
        self.solver = solver

    def get_mpcc_nlp_problem(self):
        return self.nlp_prob

    def update_mpcc_nlp_problem(self,nlp_prob):
        self.nlp_prob = nlp_prob
    
    def get_mpcc_args(self):
        return self.args

    def update_mpcc_args(self, args):
        self.args = args

    def get_wheel_raduis(self):
        return self.wheel_raduis
    
    def set_predicted_trajectory(self,traj):
        self.predicted_trajectory = traj
    
    def get_predicted_trajctory(self):
        return self.predicted_trajectory
    
    def set_last_control_sequence(self,u):
        self.prev_u = u
    
    def get_last_control_sequence(self):
        return self.prev_u

    def define_model(self):
        mB= 20                          # mass of the chassis (kg)
        self.D= 1.9                        # distance between the front wheels and the rear wheel (m)
        self.d= 0                    # distance from the center of the front wheels to the center of mass of chassis (m)
        self.B = self.D
        IB= mB*self.D**2                     # moment of inertia of the chassis (kg.m²)
        IT= IB + mB*(self.d)**2         # Total Moment of inirtia (kg.m²)
        self.wheel_raduis = 0.127       # Raduis of the driving wheel (m)
        
        x = SX.sym('x')                 # global x coordinate of body point
        y = SX.sym('y')                 # global y coordinate of body point
        theta = SX.sym('theta')         # angle between the vehicle heading and the x axis of the global frame
        gamma = SX.sym('gamma')         # steering angle between the steering angle direction and the heading of the vehicle
        x_dot = SX.sym('x_dot')         # change in x coordinate of the body point in the global frame
        y_dot = SX.sym('y_dot')         # change in y coordinate of the body point in the global frame
        theta_dot = SX.sym('theta_dot') # change in the heading angle
        gamma_dot = SX.sym('gamma_dot') # change in the steering angle
        w = SX.sym('w')                 # parameter to represent the prograss on the path
        x_b = SX.sym('x_b')                 # global x coordinate of the back point
        y_b = SX.sym('y_b')                 # global y coordinate of the back point

        
        tau = SX.sym('tau')             # Tourque input on the driving wheel 
        omega = SX.sym('omega')         # steering input 
        wd = SX.sym('wd')               # speed on the path progress 
            
        self.states = vertcat(x,y,theta,gamma,x_dot,y_dot,theta_dot,gamma_dot,w, x_b, y_b)                             # system states
        q_dot =vertcat(x_dot, y_dot, theta_dot, gamma_dot)                                              
        T = vertcat(tau*cos(gamma+theta), tau*sin(gamma+theta), -tau*sin(gamma)*(self.B), omega)      # system control inputs
        self.Input = vertcat(tau,omega,wd)

        M = SX.zeros(4,4)
        M[0,0] = mB
        M[0,2] = mB*self.d*sin(theta)
        M[1,1] = mB
        M[1,2] = -mB*self.d*cos(theta)
        M[2,0] = mB*self.d*sin(theta)
        M[2,1] = -mB*self.d*cos(theta)
        M[2,2] = IT
        M[3,3] = 3
        M_inv = inv(M)

        B = SX.zeros(4,1)
        B[0,0] = -cos(theta)*mB*self.d*(theta_dot*theta_dot)
        B[1,0] = -sin(theta)*mB*self.d*(theta_dot*theta_dot)

        C = SX.zeros(2,4)
        C[0,0] = sin(theta+gamma)
        C[0,1] = -cos(theta+gamma)
        C[0,2] = (self.B)*cos(gamma)
        C[0,3] = 0
        C[1,0] = sin(theta)
        C[1,1] = -cos(theta)
        C[1,2] = -self.d
        C[1,3] = 0
        

        Cdot = SX.zeros(2,4)
        Cdot[0,0] = cos(theta+gamma)*(theta_dot+gamma_dot)
        Cdot[0,1] = sin(theta+gamma)*(theta_dot+gamma_dot)
        Cdot[0,2] = -(self.B)*sin(gamma)*gamma_dot
        Cdot[0,3] = 0
        Cdot[1,0] = cos(theta)*theta_dot
        Cdot[1,1] = sin(theta)*theta_dot
        


        lambdas=-inv(C@M_inv@C.T)@(C@M_inv@(T-B)+Cdot@q_dot)            # lagrange multipliers to insure motion constraints are satisfied
        q_dd= M_inv@(T-B+C.T@lambdas)
        q_dd = vertcat(q_dd,0,0,0,0,0)
        f = Function('f', [self.states,self.Input], [q_dd])                  # Forward Function
        return f

    def set_MPCC_parameters(self,dt,N):
        
        n_states   = (self.states).shape
        self.n_states = n_states[0]
        n_controls = (self.Input).shape
        self.n_controls = n_controls[0]
        self.dt = dt;                       # sampling time (s) 
        self.N = N                          # prediction Horizon length
        
        self.Q = diag([200, 500,0.3])          # To DO (Decide Where to pass this paramter)---
        self.Q_terminal = diag([00, 00]) 
        self.R = diag([10,100, 0.1])         # To DO (Decide Where to pass this paramter)---
        self.q_t = 800                      # To DO (Decide Where to pass this paramter)---
        
        self.Q_col = diag([300, 300, 0.3])          # To DO (Decide Where to pass this paramter)---
        self.R_col = diag([80,200, 10])         # To DO (Decide Where to pass this paramter)---
        self.q_t_col = 100                     # To DO (Decide Where to pass this paramter)---

    def set_steering_constraints(self,max_gamma,min_gamma,args):
        
        if len(max_gamma)!=len(min_gamma):
            return args
        n_constraints = len(max_gamma)
        for i in range(3,self.n_states*(self.N+1),self.n_states):     # Gamma
            if i >= n_constraints:
                break
            max_g = max_gamma[i]
            min_g = min_gamma[i]
            args.lbx[i] = min_g*(np.pi/180)    
            args.ubx[i] = max_g*(np.pi/180)  
            
        solver = self.go_to_waypoint(max_gamma)
        return [args,solver]
     
    def go_to_waypoint(self,waypoint):
        object = 0
        Q = 100 
        for k in range(self.N):
            st  =  self.X[:,k]
            ex = st[0]-waypoint[0]
            ey = st[1]-waypoint[1]
            e = vertcat(ex,ey)
            object = object + (Q/e.T@e)
            
        nlp_prob = {'f':object , 'x':self.OPT_variables, 'g':self.g, 'p': self.P}
        opts = {}
        opts['ipopt.print_level'] = 0
        opts['print_time'] = 0
        #opts['expand'] = True
        opts['ipopt.max_iter'] = 100
        solver_obst = nlpsol('solver', 'ipopt' , nlp_prob, opts)
        return solver_obst
            
    def Init_MPCC(self,dt,N,type):
        
        self.set_MPCC_parameters(dt,N)

        ## Spline Functions ----------------------------------------------------------------------------------------------
        Qx = SX.sym('Qx',4,1)
        Qy = SX.sym('Qy',4,1)
        t = SX.sym('t')

        x_t=1.0/6.0*(((1-t)**3)*Qx[0]+(3*t**3-6*t**2+4)*Qx[1]+(-3*t**3+3*t**2+3*t+1)*Qx[2]+(t**3)*Qx[3])
        y_t=1.0/6.0*(((1-t)**3)*Qy[0]+(3*t**3-6*t**2+4)*Qy[1]+(-3*t**3+3*t**2+3*t+1)*Qy[2]+(t**3)*Qy[3]) 

        dx_t = (1/6)*((-3*(1-t)**2*Qx[0])+((9*t**2-12*t)*Qx[1])+((-9*t**2+6*t+3)*Qx[2])+(3*t**2*Qx[3]))
        dy_t = (1/6)*((-3*(1-t)**2*Qy[0])+((9*t**2-12*t)*Qy[1])+((-9*t**2+6*t+3)*Qy[2])+(3*t**2*Qy[3]))

        xd = vertcat(x_t,y_t)
        theta_d = np.arctan(dy_t,dx_t)

        spline = Function('s',[Qx,Qy,t],[xd], ['Qx','Qy', 't'],['xd'])
        self.drivative = Function('deriv',[Qx,Qy,t],[theta_d], ['Qx','Qy', 't'],['theta_d'])

        ## ------------------------------------------------------------------------------------------------------------------
        
        self.prev_u = np.zeros([self.N,self.n_controls])
        
        self.U = SX.sym('U',self.n_controls,self.N);                                      # Decision variables (controls)
        self.P = SX.sym('P',self.n_states + 8 + 2,1)                                       # parameters (which include at the initial state of the robot and the reference state)
        self.X = SX.sym('X',self.n_states,(self.N+1))                                # A vector that represents the states over the optimization problem.

        # Setting the MPC Problem -----------------------------------------------------------------------
        self.obj = 0                                                                        # Objective function
        self.obj_avoidance = 0
        self.g = []                                                                         # constraints vector
        
        st  = self.X[:,0]                                                                   # initial state
        self.g = vertcat(self.g,st-self.P[0:self.n_states])
        
        ##################################
        
        for k in range(self.N):
            st = self.X[:,k]
            con = self.U[:,k]
            st_next = self.X[:,k+1]
            st_next_euler = st
            forward = self.get_forward_function()
        ####### Euler ##############################################################
            x_dd = forward(st,con)
            x_d = st[4:(st.size()[0]-4)] + (self.dt*x_dd[0:3])          # get the next state by  euler method
            st_next_euler[4:st_next_euler.size()[0]-4] = x_d[0:3]
            st_next_euler[st_next_euler.size()[0]-4] = x_dd[3]
            x = st[0:3] + (self.dt*x_d)
            st_next_euler[0:3]= x
            st_next_euler[3] = st[3] + (self.dt*x_dd[3])
            st_next_euler[8] = st[8] + con[2]
            st_next_euler[9] = st_next_euler[0]-self.D*cos(st_next_euler[2])
            st_next_euler[10] = st_next_euler[1]-self.D*sin(st_next_euler[2])
            
            self.g = vertcat(self.g,st_next-st_next_euler)  
                       
        X_flat = reshape(self.X,self.n_states*(self.N+1),1)
        U_flat = reshape(self.U,self.n_controls*self.N,1)
        self.OPT_variables = vertcat(X_flat,U_flat)
        
        for k in range(self.N):
            st  =  self.X[:,k]
            con =  self.U[:,k]
            t_t =  self.X[8,k]
            n_waypoint_x = self.P[-2]
            n_waypoint_y = self.P[-1]
            
            if k >= self.N-1:
                delta_u = con
            else:
                con_next = self.U[:,k+1]
                delta_u = con_next-con


            s = spline(self.P[self.n_states:self.n_states+4],self.P[self.n_states+4:self.n_states+8],t_t)
            heading = self.drivative(self.P[self.n_states:self.n_states+4],self.P[self.n_states+4:self.n_states+8],t_t)

            if type == "f":
                e_c = sin(heading)*(st[0]-s[0])  - cos(heading)*(st[1]-s[1])
                e_l = -cos(heading)*(st[0]-s[0]) - sin(heading)*(st[1]-s[1])
            elif type == "b":
                e_c = sin(heading)*(st[9]-s[0])  - cos(heading)*(st[10]-s[1])
                e_l = -cos(heading)*(st[9]-s[0]) - sin(heading)*(st[10]-s[1])
                
            e_h = sqrt((heading-st[2])**2)
            
            e = vertcat(e_c, e_l,e_h)

            self.obj = self.obj + e.T@self.Q@e + delta_u.T@self.R@delta_u - self.q_t*t_t
            
            self.obj_avoidance = self.obj_avoidance + e.T@self.Q_col@e + delta_u.T@self.R_col@delta_u - self.q_t_col*t_t
            
            if k == self.N-1:
                st = self.X[:,k+1]
                t_t = self.X[8,k+1]
                s = spline(self.P[self.n_states:self.n_states+4],self.P[self.n_states+4:self.n_states+8],t_t)
                heading = self.drivative(self.P[self.n_states:self.n_states+4],self.P[self.n_states+4:self.n_states+8],t_t)
                if type == "f":
                    e_c = sin(heading)*(st[0]-s[0])  - cos(heading)*(st[1]-s[1])
                    e_l = -cos(heading)*(st[0]-s[0]) - sin(heading)*(st[1]-s[1])
                    ex = st[0]-n_waypoint_x
                    ey = st[1]-n_waypoint_y
                elif type == "b":
                    e_c = sin(heading)*(st[9]-s[0])  - cos(heading)*(st[10]-s[1])
                    e_l = -cos(heading)*(st[9]-s[0]) - sin(heading)*(st[10]-s[1])
                    ex = st[9]-n_waypoint_x
                    ey = st[10]-n_waypoint_y
                    
                e_h = sqrt((heading-st[2])**2)
                e = vertcat(e_c, e_l,e_h)
                self.obj = self.obj + e.T@self.Q@e - self.q_t*t_t

                e_p = vertcat(ex,ey)
                self.obj = self.obj + (e_p.T@self.Q_terminal@e_p)

        self.nlp_prob = {'f':self.obj , 'x':self.OPT_variables, 'g':self.g, 'p': self.P}
        
        self.opts = {}
        self.opts['ipopt.print_level'] = 0
        self.opts['print_time'] = 0
        self.opts['ipopt.max_iter'] = 60
        self.solver = nlpsol('solver', 'ipopt' , self.nlp_prob, self.opts)
        #self.solver.print_options()
        
        self.set_constraints(type)

    def set_constraints(self,type):

        self.args = self.Constraints()
        
        self.args.lbg = np.zeros((self.n_states*(self.N+1),1)) #[0 for i in range(self.n_states*(self.N+1)+ self.num_obs*(self.N+1))]
        self.args.ubg = np.zeros((self.n_states*(self.N+1),1)) #[0 for i in range(self.n_states*(self.N+1)+ self.num_obs*(self.N+1))]
        
        self.args.lbx = np.zeros((self.n_states*(self.N+1)+self.n_controls*self.N,1)) #[0 for i in range(self.n_states*(self.N+1)+self.n_controls*self.N)]
        self.args.ubx = np.zeros((self.n_states*(self.N+1)+self.n_controls*self.N,1)) #[0 for i in range(self.n_states*(self.N+1)+self.n_controls*self.N)]
        
        for i in range(self.n_states*(self.N+1)):
            self.args.lbg[i] = 0
            self.args.ubg[i] = 0
                            
        for i in range(0,self.n_states*(self.N+1),self.n_states):     # X
            self.args.lbx [i] = -inf
            self.args.ubx [i] = inf
        for i in range(1,self.n_states*(self.N+1),self.n_states):     # Y
            self.args.lbx [i] = -inf
            self.args.ubx [i] = inf
        for i in range(2,self.n_states*(self.N+1),self.n_states):     # Theta
            self.args.lbx [i] = -inf
            self.args.ubx [i] = inf     
        for i in range(3,self.n_states*(self.N+1),self.n_states):     # Gamma
            self.args.lbx [i] = -85*(pi/180)    
            self.args.ubx [i] =  85*(pi/180)      
        if type == "b":
            for i in range(4,self.n_states*(self.N+1),self.n_states):     # X_dot
                self.args.lbx [i] = -1.7   
                self.args.ubx [i] =  1.7
            for i in range(5,self.n_states*(self.N+1),self.n_states):     # Y_dot
                self.args.lbx [i] = -1.7
                self.args.ubx [i] =  1.7
        elif type == "f":  
            for i in range(4,self.n_states*(self.N+1),self.n_states):     # X_dot
                self.args.lbx [i] = -0.3   
                self.args.ubx [i] =  0.3
            for i in range(5,self.n_states*(self.N+1),self.n_states):     # Y_dot
                self.args.lbx [i] = -0.3
                self.args.ubx [i] =  0.3  
        for i in range(6,self.n_states*(self.N+1),self.n_states):     # Theta_dot
            self.args.lbx [i] = -inf
            self.args.ubx [i] =  inf           
        for i in range(7,self.n_states*(self.N+1),self.n_states):    # Gamma_dot
            self.args.lbx [i] = -pi/2  
            self.args.ubx [i] =  pi/2      
        for i in range(8,self.n_states*(self.N+1),self.n_states):    # s
            self.args.lbx [i] = 0  
            self.args.ubx [i] = 1        
        for i in range(9,self.n_states*(self.N+1),self.n_states):    # X backward
            self.args.lbx [i] = -inf  
            self.args.ubx [i] = inf
        for i in range(10,self.n_states*(self.N+1),self.n_states):   # Y backward
            self.args.lbx [i] = -inf  
            self.args.ubx [i] = inf      
        
        for i in range(self.n_states*(self.N+1),self.n_states*(self.N+1)+self.n_controls*self.N,self.n_controls):
            self.args.lbx [i] = -100   
            self.args.ubx [i] =  100    
        for i in range(self.n_states*(self.N+1)+1,self.n_states*(self.N+1)+self.n_controls*self.N,self.n_controls):
            self.args.lbx [i] = -1.5    # the steeting limit
            self.args.ubx [i] =  1.5
        for i in range(self.n_states*(self.N+1)+2,self.n_states*(self.N+1)+self.n_controls*self.N,self.n_controls):
            self.args.lbx [i] =  -0.0001     # the s_dot limit
            self.args.ubx [i] =  0.05

    def handle_dynamic_obstacle(self,obst_traj,N):

        objective = self.obj_avoidance
        for i in range(min(15,N)):
            st = self.X[:,i]
            for j in range(min(5,len(obst_traj))):
                obst_pose = obst_traj[j,:]
                dist_v_f = sqrt((st[0]-obst_pose[0])**2 + (st[1]-obst_pose[1])**2)
                dist_v_b = sqrt((st[9]-obst_pose[0])**2 + (st[10]-obst_pose[1])**2) 
                #v_v = (st[4]*np.cos(st[2])) + (st[5]*np.sin(st[2]))
                #delta_t = dist_v_b/(v_v+0.01)
                objective = objective + 200/(dist_v_f+0.0001) + 200/(dist_v_b+0.0001)
        
        nlp_prob = {'f':objective , 'x':self.OPT_variables, 'g':self.g, 'p': self.P}
        opts = {}
        opts['ipopt.print_level'] = 0
        opts['print_time'] = 0
        #opts['expand'] = True
        opts['ipopt.max_iter'] = 100
        solver_obst = nlpsol('solver', 'ipopt' , nlp_prob, opts)
        return solver_obst
    
    class Constraints:
        lbg = []
        ubg = []
        lbx = []
        ubx = []
        p   = []
        x0  = []