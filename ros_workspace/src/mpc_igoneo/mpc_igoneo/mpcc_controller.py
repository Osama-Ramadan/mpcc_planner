# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

#sudo apt update
#sudo apt install python3-pip
#sudo apt-get install python3-scipy python3-pandas
#sudo apt install ros-foxy-navigation2
#sudo apt install ros-foxy-nav2-bringup
#sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
#sudo apt install ros-foxy-turtlebot3*
#rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
import numpy as np
from interfaces.srv import Interpolation
from interfaces.msg import FloatList
from interfaces.msg import FloatArray
from mpc_igoneo.helper_functions import *
from mpc_igoneo.igoneo_model import IGONEO
import time
from sys import path
import pandas as pd 

 #Import CasADi 
path.append(r"/home/casadi-linux-py36-v3.5.5-64bit")
from casadi import *



class MPC_CONTROLLER(Node):
    def __init__(self):

        super().__init__("mpcc_controller")

        group = MutuallyExclusiveCallbackGroup()
        group1 = MutuallyExclusiveCallbackGroup()
        group2 = MutuallyExclusiveCallbackGroup()

        # Create the publisher. This publisher will publish a JointState message to the /joint_command topic.
        self.Cmdpublisher_ =  self.create_publisher(JointState, "joint_command", 10)
        self.Jointsubscriper_ = self.create_subscription(JointState, "joint_states", self.joint_linstner, 10,callback_group = group)
        self.Velsubscriper_ = self.create_subscription(Odometry, "odom", self.vel_linstner, 10,callback_group = group)
        self.obstsubscriper_ = self.create_subscription(PoseArray, "predicted_traj", self.obst_pred_traj_listner, 10,callback_group = group1)

        self.interpolate_client = self.create_client(Interpolation,'interpolation_service')
        while not self.interpolate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = Interpolation.Request() 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a JointState message
        self.joint_state = JointState()

        self.joint_state.name = [
            "chassis_drehschemel",
            "drehschemel_antriebsrad"
        ]

        num_joints = len(self.joint_state.name)

        self.joint_state.velocity = np.array([0.0] * num_joints, dtype=np.float64).tolist()
                
        self.timer_period = 0.2  # seconds        

        self.igoneo = IGONEO()
        self.Init_MPC()

        self.timer = self.create_timer(self.timer_period, self.mpcc_timer_callback, callback_group = group2)

        self.current_poses = []
        self.path = []
        self.prev_path = []
        self.current_path = []
        self.current_waypoints = []
        self.prev_waypoints = []
        self.current_seg = 1

    def obst_pred_traj_listner(self, msg):
        obst_predicted_trajectory = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            point = [x,y]
            obst_predicted_trajectory.append(point)
        self.predicted_obst_trajec = list(obst_predicted_trajectory)

    def send_waypoints(self, waypoints, accuracy_parameter):
        
        global_points = FloatArray()
        global_points_list = []
        global_points.size = len(waypoints)
        for i in range(len(waypoints)):
            list = FloatList()
            list.elements = waypoints[i]
            global_points_list.append(list)

        global_points.lists = global_points_list
        self.req.globalwaypoints = global_points
        self.req.accuracyparameter = int(accuracy_parameter)
        future = self.interpolate_client.call(self.req)
        return future
    
    def mpcc_timer_callback(self):
        state_update = self.update_state()

        goals = np.array([[float(-5),float(0)],[float(10),float(0)], [float(15),float(0)]])
        num_goals = len(goals)
        lookahead = 2
        self.currentx = float(state_update[0])
        self.currenty = float(state_update[1])
        self.current_pose = [self.currentx ,self.currenty]
        accuracy_parameter = 100
        
        self.current_waypoints = get_next_waypoints(lookahead, goals, self.current_poses, self.current_waypoints,self.current_pose)
        if not(self.current_waypoints == self.prev_waypoints):
            self.response = self.send_waypoints(self.current_waypoints, accuracy_parameter)
            self.current_poses = np.dstack([self.response.localpath_x,self.response.localpath_y])[0]
            self.current_path = np.dstack([self.response.localpath_x,self.response.localpath_y])[0]
            if len(self.path)==0:
                self.path = np.array(self.current_path)
                self.prev_path = list(self.current_path)
        Qx = self.response.qx
        Qy = self.response.qy
        self.prev_waypoints = list(self.current_waypoints)
        Trajectory_obst = np.reshape(self.predicted_obst_trajec,[len(self.predicted_obst_trajec),2])
        self.MPC_run(state_update, np.array(self.current_path), Qx, Qy, Trajectory_obst)

    def update_state(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('opx_l12','chassis',now)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
            return


        qw = trans.transform.rotation.w
        qz = trans.transform.rotation.z
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        t3 = 2*(qw*qz + qx*qy)
        t4 = 1-2*(qy*qy + qz*qz)
        angle = atan2(t3,t4)
        self.states[0] = trans.transform.translation.x
        self.states[1] = trans.transform.translation.y
        self.states[2] = angle


        self.feedback[0] = float(self.states[0])
        self.feedback[1] = float(self.states[1])
        self.feedback[2] = float(self.states[2])
        self.feedback[3] = float(self.joint_buffer[0])
        self.feedback[4] = float(self.odom_buffer[0]*cos(angle) - self.odom_buffer[1]*sin(angle))
        self.feedback[5] = float(self.odom_buffer[0]*sin(angle) + self.odom_buffer[1]*cos(angle))
        self.feedback[6] = float(self.odom_buffer[2])
        self.feedback[7] = float(self.joint_buffer[1])

        return np.round(self.feedback,3)

    def joint_linstner(self, msg):

        #print('Gamma_delta =  ' + str(msg.position[0]-self.joint_buffer[3]))
        self.joint_buffer[0] = msg.position[0]            # gamma
        self.joint_buffer[1] = msg.velocity[0]            # gamma_dot

    def vel_linstner(self, msg):

       self.odom_buffer[0] = msg.twist.twist.linear.x                # x_dot
       self.odom_buffer[1] = msg.twist.twist.linear.y                # y_dot
       self.odom_buffer[2] = msg.twist.twist.angular.z               # theta_dot

    def set_constraints(self):

        self.args = self.Constraints()
        
        self.args.lbg = np.zeros(self.n_states*(self.N+1)+ self.num_obs*(self.N+1),1) #[0 for i in range(self.n_states*(self.N+1)+ self.num_obs*(self.N+1))]
        self.args.ubg = np.zeros(self.n_states*(self.N+1)+ self.num_obs*(self.N+1),1) #[0 for i in range(self.n_states*(self.N+1)+ self.num_obs*(self.N+1))]
        
        self.args.lbx = np.zeros(self.n_states*(self.N+1)+self.n_controls*self.N,1) #[0 for i in range(self.n_states*(self.N+1)+self.n_controls*self.N)]
        self.args.ubx = np.zeros(self.n_states*(self.N+1)+self.n_controls*self.N,1) #[0 for i in range(self.n_states*(self.N+1)+self.n_controls*self.N)]
        
        for i in range(self.n_states*(self.N+1)):
            self.args.lbg[i] = 0
            self.args.ubg[i] = 0
            
        for i in range(self.n_states*(self.N+1),self.n_states*(self.N+1)+ self.num_obs*(self.N+1)):
            self.args.lbg[i] = -1000
            self.args.ubg[i] = -0.1     # The acceptable distance between the vehicle and the obstacles
                            
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
        for i in range(4,self.n_states*(self.N+1),self.n_states):     # X_dot
            self.args.lbx [i] = -1.0   
            self.args.ubx [i] =  1.0
        for i in range(5,self.n_states*(self.N+1),self.n_states):     # Y_dot
            self.args.lbx [i] = -1.0
            self.args.ubx [i] =  1.0    
        for i in range(6,self.n_states*(self.N+1),self.n_states):     # Theta_dot
            self.args.lbx [i] = -inf
            self.args.ubx [i] =  inf           
        for i in range(7,self.n_states*(self.N+1),self.n_states):    # Gamma_dot
            self.args.lbx [i] = -pi/2  
            self.args.ubx [i] =  pi/2      
        for i in range(8,self.n_states*(self.N+1),self.n_states):    # s
            self.args.lbx [i] = 0  
            self.args.ubx [i] = 1        

        for i in range(self.n_states*(self.N+1),self.n_states*(self.N+1)+self.n_controls*self.N,self.n_controls):
            self.args.lbx [i] = -30   
            self.args.ubx [i] =  10     
        for i in range(self.n_states*(self.N+1)+1,self.n_states*(self.N+1)+self.n_controls*self.N,self.n_controls):
            self.args.lbx [i] = -1.5    # the steeting limit
            self.args.ubx [i] =  1.5
        for i in range(self.n_states*(self.N+1)+2,self.n_states*(self.N+1)+self.n_controls*self.N,self.n_controls):
            self.args.lbx [i] =  -0.001     # the s_dot limit
            self.args.ubx [i] =  0.008
  
    def Init_MPC(self):
        
        self.n_states   = 9
        self.n_controls = 3

        self.states = np.zeros([self.n_states,1])
        self.joint_buffer = np.zeros([2,1])
        self.odom_buffer = np.zeros([3,1])

        self.dt = self.timer_period;                    #[s]  sampling time 
        self.N = 20

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
        drivative = Function('deriv',[Qx,Qy,t],[theta_d], ['Qx','Qy', 't'],['theta_d'])

        ## ------------------------------------------------------------------------------------------------------------------


        U = SX.sym('U',self.n_controls,self.N);                                             # Decision variables (controls)
        self.P = SX.sym('P',self.n_states + 8)                                       # parameters (which include at the initial state of the robot and the reference state)
        self.X = SX.sym('X',self.n_states,(self.N+1))                                            # A vector that represents the states over the optimization problem.

        # Setting the MPC Problem -----------------------------------------------------------------------
        self.obj = 0                                                                        # Objective function
        self.g = []                                                                         # constraints vector
        
        Q = diag([200, 1000]);                                                    # weighing matrices (states)
        R = diag([10,100, 0.1])                                                       # weighing matrices (controls)
        
        st  = self.X[:,0]                                                                   # initial state
        self.g = vertcat(self.g,st-self.P[0:self.n_states])
        
        ##################################
        obs_x =    []         
        obs_y =    []  
        #################################
        
        self.num_obs  = len(obs_x)
        self.obs_diam = 1
        self.rob_diam = 1.2

        
        for k in range(self.N):
            st = self.X[:,k]
            con = U[:,k]
            st_next = self.X[:,k+1]
            st_next_euler = st
            forward = self.igoneo.get_forward_function()
        ####### Euler ##############################################################
            x_dd = forward(st,con)
            x_d = st[4:(st.size()[0]-2)] + (self.dt*x_dd[0:3])          # get the next state by  euler method
            st_next_euler[4:st_next_euler.size()[0]-2] = x_d[0:3]
            st_next_euler[st_next_euler.size()[0]-2] = x_dd[3]
            x = st[0:3] + (self.dt*x_d)
            st_next_euler[0:3]= x
            st_next_euler[3] = st[3] + (self.dt*x_dd[3])
            st_next_euler[8] = st[8] + con[2]
            self.g = vertcat(self.g,st_next-st_next_euler)
        ####### RK4 #####################################################
            #k1 = self.f(st,con)
            #k2 = self.f(st+self.dt/2*k1,con)
            #k3 = self.f(st+self.dt/2*k2,con)
            #k4 = self.f(st+self.dt*k3,con)
            #next_st = self.dt/6*(k1+2*k2+2*k3+k4)

            #x_d = st[4:(st.size()[0]-1)] + next_st[0:3]
            #st_next_euler[4:st_next_euler.size()[0]-1] = x_d[0:3]
            #st_next_euler[st_next_euler.size()[0]-1] = k1[3]
            #x = st[0:3] + (self.dt*x_d)
            #st_next_euler[0:3]= x
            #st_next_euler[3] = st[3] + next_st[3]
            #g = vertcat(g,st_next-st_next_euler)
        
        for k in range(self.N+1):
            for j in range(self.num_obs):
                self.g = vertcat(self.g,-sqrt(((self.X[0,k]-obs_x[j])**2)+((self.X[1,k]-obs_y[j])**2)) + (self.rob_diam/2 + self.obs_diam/2))
                       
        X_flat = reshape(self.X,self.n_states*(self.N+1),1)
        U_flat = reshape(U,self.n_controls*self.N,1)
        self.OPT_variables = vertcat(X_flat,U_flat)
        
        for k in range(self.N):
            st  =  self.X[:,k]
            con =  U[:,k]
            t_t =  self.X[8,k]

            if k == self.N-1:
                delta_u = con
            else:
                con_next = U[:,k+1]
                delta_u = con_next-con


            q_t = 200

            s = spline(self.P[self.n_states:self.n_states+4],self.P[self.n_states+4:self.n_states+8],t_t)
            heading = drivative(self.P[self.n_states:self.n_states+4],self.P[self.n_states+4:self.n_states+8],t_t)

            e_c = sin(heading)*(st[0]-s[0])  - cos(heading)*(st[1]-s[1])
            e_l = -cos(heading)*(st[0]-s[0]) - sin(heading)*(st[1]-s[1])

            e = vertcat(e_c, e_l)

            self.obj = self.obj + e.T@Q@e + delta_u.T@R@delta_u - q_t*t_t

        self.nlp_prob = {'f':self.obj , 'x':self.OPT_variables, 'g':self.g, 'p': self.P}
        
        self.opts = {}
        self.opts['ipopt.print_level'] = 0
        self.opts['print_time'] = 0
        self.opts['expand'] = True
        #opts['ipopt.max_iter'] = 200
        #opts['ipopt'] = dict(mu_strategy = 'adaptive')

        self.solver = nlpsol('solver', 'ipopt' , self.nlp_prob, self.opts)
        #self.solver.print_options()
        
        self.set_constraints()

        self.u0 = np.zeros([self.N,self.n_controls])

        self.feedback = np.zeros((self.n_states,1))

        self.tick_num = 0 
        self.actual_state = []
        self.predicted_state = []
        self.solver_time = []
        self.obstacle_traj = []
        self.log_Ticks = 500

        self.g_static = self.g
        self.lbg_static = list(self.args.lbg)
        self.ubg_static = list(self.args.ubg)

    def MPC_run(self, feedback_state, local_path, Qx, Qy, obst_trajectory):
        
        self.g = self.g_static
        self.args.lbg = list(self.lbg_static)
        self.args.ubg = list(self.ubg_static)
        
        self.tick_num = self.tick_num+1
        self.x0 = feedback_state

        closest_point_index = find_closest_point(local_path,[float(self.x0[0]),float(self.x0[1])])
        if closest_point_index > (self.current_seg-1)*100 + 90:
            self.current_seg = self.current_seg+1
            if self.current_seg > 2:
                self.current_seg = 2
        i = (closest_point_index)-(self.current_seg-1)*100
        if(i < 0):
            i = 1
        t_t0 = i*0.01

        self.x0[8] = t_t0
        self.args.p[0:self.n_states] = self.x0
        self.args.p[self.n_states:self.n_states+4]   = Qx[self.current_seg-1:self.current_seg+3]
        self.args.p[self.n_states+4:self.n_states+8] = Qy[self.current_seg-1:self.current_seg+3]

        if np.sqrt((self.x0[0]-obst_trajectory[0,0])**2+(self.x0[1]-obst_trajectory[0,1])**2) < 5:
            for k in range(20):
                for l in range(len(obst_trajectory)):
                    self.g = vertcat(self.g,-sqrt(((self.X[0,k]-obst_trajectory[l,0])**2)+((self.X[1,k]-obst_trajectory[l,1])**2)) + (self.rob_diam/2 + self.obs_diam/2))
                    self.args.lbg.append(-100)
                    self.args.ubg.append(-0.2)
            self.nlp_prob = {'f':self.obj , 'x':self.OPT_variables, 'g':self.g, 'p': self.P}
            self.solver = nlpsol('solver', 'ipopt' , self.nlp_prob, self.opts)
                
        else:
            self.nlp_prob = {'f':self.obj , 'x':self.OPT_variables, 'g':self.g, 'p': self.P}
            self.solver = nlpsol('solver', 'ipopt' , self.nlp_prob, self.opts)

        self.X0 = repmat(self.x0,1,self.N+1).T
        
        self.args.x0 = vertcat(reshape(self.X0.T,self.n_states*(self.N+1),1),(reshape(self.u0.T,self.n_controls*self.N,1)))
  
        # get the start time
        st = time.time()
        sol = self.solver(x0 = self.args.x0, lbx = self.args.lbx, ubx = self.args.ubx, lbg = self.args.lbg, ubg = self.args.ubg, p = self.args.p)
        et = time.time()
        # get the execution time
        elapsed_time = et - st
        print('Execution time:', round(elapsed_time*1000,3), 'milliseconds')
        
        status = self.solver.stats()  
        #print(status)   

        if(not((status['return_status'])=='Solve_Succeeded')):
            print(status['return_status'])
            #self.joint_state.velocity = [float(0), float(0)]
            #now1 = self.get_clock().now()
            #self.joint_state.header.stamp = now1.to_msg()
            #self.Cmdpublisher_.publish(self.joint_state)
            #return   

        sol = sol['x'].full().flatten()
        u = reshape(sol[self.n_states*(self.N+1):].T,self.n_controls,self.N).T


        #state_opt = reshape(sol[:self.n_states*(self.N+1)].T,self.n_states,self.N+1).T
        #state_opt = np.array(state_opt)
        #self.predicted_state.append(state_opt)
        #self.actual_state.append(np.array(self.x0).T)
        #self.solver_time.append(elapsed_time)
        #self.obstacle_traj.append(obst_trajectory)
        if (not(float(self.prev_path[0][0])==float(local_path[0][0]))):
            self.path = np.concatenate((self.path,np.array(local_path)))
        self.prev_path = list(local_path)
        #print(self.actual_state)

        if self.tick_num == self.log_Ticks:
            state_array = np.array(self.predicted_state)
            state_array = state_array.reshape((self.log_Ticks*(self.N+1),self.n_states))
            actual_array = np.array(self.actual_state)
            actual_array = actual_array.reshape((self.log_Ticks,self.n_states))
            obstacle_trajc = np.reshape(self.obstacle_traj,[len(self.obstacle_traj)*len(self.obstacle_traj[0]),2])
            pd.DataFrame(actual_array).to_csv("/home/logs/real_state.csv")
            pd.DataFrame(state_array).to_csv("/home/logs/states.csv")    
            pd.DataFrame(self.solver_time).to_csv("/home/logs/solver_time.csv") 
            pd.DataFrame(self.path).to_csv("/home/logs/path.csv")    
            pd.DataFrame(obstacle_trajc).to_csv("/home/logs/obstacle.csv") 
            print('Data is logged')       

        final_state = reshape(sol[self.n_states*(4):self.n_states*(5)],self.n_states,1)
       
        self.u0 = np.zeros([self.N,self.n_controls])
       
        tx_u = u[1,:]
        x_dot_body = final_state[4]*cos(final_state[2]) + final_state[5]*sin(final_state[2])
        #self.v = sqrt(final_state[4]**2+final_state[5]**2)*sign(x_dot_body)
        self.v = x_dot_body
        if(abs(self.x0[3])>=(70*pi/180) and abs(tx_u[0])>=10):
            self.v=0.15
            pass

        self.joint_state.velocity = [float(tx_u[1]), float(self.v/0.127)]

        #self.print_feedback(tx_u)


        # Publish the message to the topic
        now1 = self.get_clock().now()
        self.joint_state.header.stamp = now1.to_msg()
        self.Cmdpublisher_.publish(self.joint_state)
    
    def print_feedback(self, tx_u):
        print('X =  ' + str(self.x0[0]))
        print('Y =  ' + str(self.x0[1]))
        print('Theta =  ' + str(self.x0[2]*180/pi))
        print('Gamma =  ' + str(self.x0[3]))
        print('X_dot =  ' + str(self.x0[4]))
        print('Y_dot =  ' + str(self.x0[5]))
        print('Theta_dot =  ' + str(self.x0[6]))
        print('gamma_dot =  ' + str(self.x0[7]))
        print('Omega = ' + str(tx_u[1]))
        print('Torque = ' + str(tx_u[0]))
        print('-------------------------------------')
        
    class Constraints:
        lbg = []
        ubg = []
        lbx = []
        ubx = []
        p   = []
        x0  = []

def main(args=None):
    rclpy.init(args=args)

    try:
        mpc_node = MPC_CONTROLLER()
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(mpc_node)


        try:
            executor.spin()
        finally:
            executor.shutdown()
            mpc_node.destroy_node()
    # Destroy the node explicitly  
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()


def check_collision(self,obstacle_traj_f,obstacle_traj_b,local_path):

        # First check if the line segments of the obstacle trajectory and the path segments intersect
        #result_f = self.check_intersection(obstacle_traj_f,local_path)
        #result_b = self.check_intersection(obstacle_traj_b,local_path)
        #if result_f[0]== False and result_b[0] == False:
            #return [False,[],[]]

        # Get the segment each point trajectory intersect with (Mostly it will be the same segment)
        #collision_seg_f = result_f[1]
        #collision_seg_b = result_b[1]


        # Check if any point on the segment will collide with the trajectory of the forward point
        collision_t_f = -1
        if collision_seg_f !=0:
            seg_start_index = (collision_seg_f-1)*local_path.path_sampling_para + int((local_path.path_sampling_para/2)*result_f[2])
            seg_end_index = (collision_seg_f)*local_path.path_sampling_para
        
            for i in range(seg_start_index,seg_end_index):
                path_pt = local_path.localpath[i]
                for j in range(len(obstacle_traj_f)-1,-1,-1):
                    dist = np.sqrt(np.sum((path_pt - obstacle_traj_f[j]) ** 2, axis=0))
                    if dist < (self.obst.r + 1.5):
                        collision_t_f = i
                        break
                else:
                    continue
                break
        
        # True if there is a collision, Then just examin 20 point before the collision point and 10 point after for backward point
        if collision_t_f != -1:    
            for i in range(max(collision_t_f-20,seg_start_index),min(collision_t_f+20,seg_end_index)):
                path_pt = local_path.localpath[i]
                for j in range(len(obstacle_traj_b)-1,-1,-1):
                    dist = np.sqrt(np.sum((path_pt - obstacle_traj_b[j]) ** 2, axis=0))
                    if dist < (self.obst.r + 1.5):
                        return [True,local_path.localpath[collision_t_f],local_path.localpath[i]]               # Both points collide with the path
            return [True,local_path.localpath[collision_t_f],[]]                      # Only forward point collide
        elif collision_seg_b != 0:
            for i in range((collision_seg_b-1)*local_path.path_sampling_para,(collision_seg_b)*local_path.path_sampling_para):
                path_pt = local_path.localpath[i]
                for j in range(len(obstacle_traj_b)-1,-1,-1):
                    dist = np.sqrt(np.sum((path_pt - obstacle_traj_b[j]) ** 2, axis=0))
                    if dist < (self.obst.r + 1.5):
                        return [True,[],local_path.localpath[i]]                          # Only backward point collide
        return[False,[],[]]