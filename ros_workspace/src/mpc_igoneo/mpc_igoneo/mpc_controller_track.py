# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

#sudo apt update
#sudo apt install python3-pip
#sudo apt-get install python3-scipy python3-matplotlib ipython3 python3-pandas
#sudo apt install ros-foxy-navigation2
#sudo apt install ros-foxy-nav2-bringup


from threading import local
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
import numpy as np
from interfaces.srv import Interpolation
from interfaces.msg import FloatList
from interfaces.msg import FloatArray
import time
from sys import path
from scipy import spatial
import pandas as pd 

# Import CasADi 
path.append(r"/home/casadi-linux-py36-v3.5.5-64bit")
from casadi import *



class MPC_CONTROLLER(Node):
    def __init__(self):

        super().__init__("mpc_controller")

        group = MutuallyExclusiveCallbackGroup()
        group1 = MutuallyExclusiveCallbackGroup()
        group2 = MutuallyExclusiveCallbackGroup()

        # Create the publisher. This publisher will publish a JointState message to the /joint_command topic.
        self.Cmdpublisher_ =  self.create_publisher(JointState, "joint_command", 10)
        self.Jointsubscriper_ = self.create_subscription(JointState, "joint_states", self.joint_linstner, 10,callback_group = group)
        self.Velsubscriper_ = self.create_subscription(Odometry, "odom", self.vel_linstner, 10,callback_group = group1)
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

        self.Init_MPC()

        self.timer = self.create_timer(self.timer_period, self.timer_callback, callback_group = group2)

        self.current_poses = []
        self.current_path = []
        self.current_waypoints = []

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

    def find_closest_point(self, list, point):
        mytree = spatial.cKDTree(list)
        dist, indexes = mytree.query(point)
        return indexes

    def get_next_waypoints(self, lookahead, goals, current_path, current_waypoints, current_pose):
        if len(current_path)==0 and len(current_waypoints)==0:      # first time to call no path or waypoint calculated before
            return goals[0:lookahead]
        waypoint_index = self.find_closest_point(current_path,[current_waypoints[1],current_pose])
        if waypoint_index[0] > waypoint_index[1]:         # vehciel did not complete the first segmetn of the path yet
            return current_waypoints
        else:
            current_waypoint_index_in_goal = goals.index(current_waypoints[0])  # vehicle already finished the firts segment
            print('First waypoint loc in goals: ', current_waypoint_index_in_goal)
            if current_waypoint_index_in_goal == (len(goals)-lookahead):        # if the current path reaches the final goal point
                print('I am in the final segment !!')
                return current_waypoints
            else:
                current_waypoints = goals[current_waypoint_index_in_goal+1:current_waypoint_index_in_goal+lookahead+1]
                return current_waypoints
    
    def timer_callback(self):
        state_update = self.update_state()

        goals = [[float(-5),float(0)],[float(0),float(0)], [float(2), float(0)], [float(6), float(0)], [float(8), float(0)]]
        lookahead = 3
        self.currentx = float(state_update[0])
        self.currenty = float(state_update[1])
        self.current_pose = [self.currentx ,self.currenty]
        accuracy_parameter = 15
        
        self.current_waypoints = self.get_next_waypoints(lookahead, goals, self.current_poses, self.current_waypoints,self.current_pose)
        response = self.send_waypoints(self.current_waypoints, accuracy_parameter)
        self.current_poses = np.dstack([response.localpath_x,response.localpath_y])[0]
        self.current_path = np.dstack([response.localpath_x,response.localpath_y, response.localpath_theta])[0]
        self.MPC_run(state_update, np.array(self.current_path))

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
        
        self.args.lbg = [0 for i in range(self.n_states*(self.N+1)+ self.num_obs*(self.N+1))]
        self.args.ubg = [0 for i in range(self.n_states*(self.N+1)+ self.num_obs*(self.N+1))]
        
        self.args.lbx = [0 for i in range(self.n_states*(self.N+1)+self.n_controls*self.N)]
        self.args.ubx = [0 for i in range(self.n_states*(self.N+1)+self.n_controls*self.N)]
        
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
            self.args.lbx [i] = -80*(pi/180)    
            self.args.ubx [i] =  80*(pi/180)      
        for i in range(4,self.n_states*(self.N+1),self.n_states):     # X_dot
            self.args.lbx [i] = -1.7   
            self.args.ubx [i] =  1.5
        for i in range(5,self.n_states*(self.N+1),self.n_states):     # Y_dot
            self.args.lbx [i] = -1.7
            self.args.ubx [i] =  1.5    
        for i in range(6,self.n_states*(self.N+1),self.n_states):     # Theta_dot
            self.args.lbx [i] = -inf
            self.args.ubx [i] =  inf           
        for i in range(7,self.n_states*(self.N+1),self.n_states):    # Gamma_dot
            self.args.lbx [i] = -pi/2  
            self.args.ubx [i] =  pi/2         

        for i in range(self.n_states*(self.N+1),self.n_states*(self.N+1)+self.n_controls*self.N,self.n_controls):
            self.args.lbx [i] = -20   
            self.args.ubx [i] =  20     
        for i in range(self.n_states*(self.N+1)+1,self.n_states*(self.N+1)+self.n_controls*self.N,self.n_controls):
            self.args.lbx [i] = -1.5    # the steeting limit
            self.args.ubx [i] =  1.5
  
    def Init_MPC(self):
        self.states = np.zeros([8,1])
        self.joint_buffer = np.zeros([2,1])
        self.odom_buffer = np.zeros([3,1])

        self.dt = self.timer_period;                    #[s]  sampling time 
        self.N = 10
                
        self.define_model()

        self.n_states   = 8
        self.n_controls = 2

        U = SX.sym('U',self.n_controls,self.N);                                             # Decision variables (controls)
        P = SX.sym('P',self.n_states + self.N*(self.n_states))                                       # parameters (which include at the initial state of the robot and the reference state)
        X = SX.sym('X',self.n_states,(self.N+1))                                            # A vector that represents the states over the optimization problem.

        # Setting the MPC Problem -----------------------------------------------------------------------
        obj = 0                                                                        # Objective function
        g = []                                                                         # constraints vector
        
        Q = diag([20, 20, 2, 0, 0, 0, 0, 0]);                                                    # weighing matrices (states)
        Q_terminal = diag([20,20, 2, 0, 0.0, 0, 0, 0]);                                                      # weighing matrices (states)
        R = diag([0.1,0.01])                                                               # weighing matrices (controls)
        
        st  = X[:,0]                                                                   # initial state
        g = vertcat(g,st-P[0:self.n_states])
        
        ##################################
        obs_x =    []         
        obs_y =    []  
        #################################
        
        self.num_obs  = len(obs_x)
        obs_diam = 1
        rob_diam = 1.2

        
        for k in range(self.N):
            st = X[:,k]
            con = U[:,k]
            st_next = X[:,k+1]
            st_next_euler = st
        ####### Euler ##############################################################
            x_dd = self.f(st,con)
            x_d = st[4:(st.size()[0]-1)] + (self.dt*x_dd[0:3])          # get the next state by  euler method
            st_next_euler[4:st_next_euler.size()[0]-1] = x_d[0:3]
            st_next_euler[st_next_euler.size()[0]-1] = x_dd[3]
            x = st[0:3] + (self.dt*x_d)
            st_next_euler[0:3]= x
            st_next_euler[3] = st[3] + (self.dt*x_dd[3])
            g = vertcat(g,st_next-st_next_euler)
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
                g = vertcat(g,-sqrt(((X[0,k]-obs_x[j])**2)+((X[1,k]-obs_y[j])**2)) + (rob_diam/2 + obs_diam/2))
                       
        X_flat = reshape(X,self.n_states*(self.N+1),1)
        U_flat = reshape(U,self.n_controls*self.N,1)
        OPT_variables = vertcat(X_flat,U_flat)
        
        for k in range(self.N):
            if k!= self.N-1:                                        
                obj = obj+(st-P[k*(self.n_states):(k+1)*self.n_states]).T@Q@(st-P[k*(self.n_states):(k+1)*self.n_states]) + con.T@R@con
            else:
                obj = obj+(st-P[k*(self.n_states):(k+1)*self.n_states]).T@Q_terminal@(st-P[k*(self.n_states):(k+1)*self.n_states]) + con.T@R@con

        nlp_prob = {'f':obj , 'x':OPT_variables, 'g':g, 'p': P}
        
        opts = {}
        opts['ipopt.print_level'] = 0
        opts['print_time'] = 0
        opts['expand'] = True
        #opts['ipopt.max_iter'] = 200
        #opts['ipopt'] = dict(mu_strategy = 'adaptive')

        self.solver = nlpsol('solver', 'ipopt' , nlp_prob, opts)
        #self.solver.print_options()
        
        self.set_constraints()

        self.u0 = np.zeros([self.N,2])

        self.feedback = np.zeros((8,1))

        self.tick_num = 0 
        self.actual_state = []
        self.predicted_state = []
        self.solver_time = []
        self.log_Ticks = 500

    def MPC_run(self, feedback_state, local_path):
        
        self.tick_num = self.tick_num+1
        self.x0 = feedback_state

        closest_point_index = self.find_closest_point(local_path,[float(self.x0[0]),float(self.x0[1]), float(self.x0[2])])
        print(local_path)
        print('X =' + str(self.x0[0]) + ' Y = ' + str(self.x0[1]))
        print(local_path[closest_point_index][0])
        print('-----------------------')
        self.X0 = repmat(self.x0,1,self.N+1).T
        if closest_point_index+self.N+1 < np.size(local_path,0):
            for k in range(self.N+1):
                self.args.p[k*(self.n_states):(k+1)*self.n_states] = [local_path[k+closest_point_index][0],local_path[k+closest_point_index][1],local_path[k+closest_point_index][2],0,0,0,0,0]
        self.args.x0 = vertcat(reshape(self.X0.T,self.n_states*(self.N+1),1),(reshape(self.u0.T,self.n_controls*self.N,1)))
  
        # get the start time
        st = time.time()
        sol = self.solver(x0 = self.args.x0, lbx = self.args.lbx, ubx = self.args.ubx, lbg = self.args.lbg, ubg = self.args.ubg, p = self.args.p)
        et = time.time()
        # get the execution time
        elapsed_time = et - st
        #print('Execution time:', round(elapsed_time*1000,3), 'milliseconds')
        
        status = self.solver.stats()  
        #print(status)   

        if(not((status['return_status'])=='Solve_Succeeded')):
            pass
            #print(status['return_status'])
            #self.joint_state.velocity = [float(0), float(0)]
            #now1 = self.get_clock().now()
            #self.joint_state.header.stamp = now1.to_msg()
            #self.Cmdpublisher_.publish(self.joint_state)
            #return   

        sol = sol['x'].full().flatten()
        u = reshape(sol[self.n_states*(self.N+1):].T,self.n_controls,self.N).T


        state_opt = reshape(sol[:self.n_states*(self.N+1)].T,self.n_states,self.N+1).T
        state_opt = np.array(state_opt)
        self.predicted_state.append(state_opt)
        self.actual_state.append(np.array(self.x0).T)
        self.solver_time.append(elapsed_time)
        #print(self.actual_state)

        if self.tick_num == self.log_Ticks:
            state_array = np.array(self.predicted_state)
            state_array = state_array.reshape((self.log_Ticks*(self.N+1),self.n_states))
            actual_array = np.array(self.actual_state)
            actual_array = actual_array.reshape((self.log_Ticks,self.n_states))
            pd.DataFrame(actual_array).to_csv("/home/logs/real_state.csv")
            pd.DataFrame(state_array).to_csv("/home/logs/states.csv")    
            pd.DataFrame(self.solver_time).to_csv("/home/logs/solver_time.csv")   
            print('Data is logged')       

        final_state = reshape(sol[self.n_states*(4):self.n_states*(5)],self.n_states,1)
       
        self.u0 = np.zeros([self.N,2])
       
        tx_u = u[1,:]
        x_dot_body = final_state[4]*cos(final_state[2]) + final_state[5]*sin(final_state[2])
        self.v = sqrt(final_state[4]**2+final_state[5]**2)*sign(x_dot_body)
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

    def define_model(self):

        mB= 100                     # mass of the chassis
        self.d= 1.0                    # distance from the center of the wheels to the center of mass of chassis
        D= 1.827             
        IB= mB*D**2            # moment of inertia of the chassis
        IT=IB + mB*(self.d)**2 
            
        x = SX.sym('x')
        y = SX.sym('y')
        theta = SX.sym('theta')
        gamma = SX.sym('gamma')
        x_dot = SX.sym('x_dot')
        y_dot = SX.sym('y_dot')
        theta_dot = SX.sym('theta_dot')
        gamma_dot = SX.sym('gamma_dot')

        
        tau = SX.sym('tau') 
        omega = SX.sym('omega')
            
        states = vertcat(x,y,theta,gamma,x_dot,y_dot,theta_dot,gamma_dot)
        q_dot =vertcat(x_dot, y_dot, theta_dot, gamma_dot)       # system states
        T = vertcat(tau*cos(gamma+theta), tau*sin(gamma+theta), -tau*sin(gamma)*(D-self.d), omega)                                               # system control inputs
        Input = vertcat(tau,omega)



        M = SX.zeros(4,4)
        M[0,0] = mB
        M[0,2] = mB*self.d*sin(theta)
        M[1,1] = mB
        M[1,2] = -mB*self.d*cos(theta)
        M[2,0] = mB*self.d*sin(theta)
        M[2,1] = -mB*self.d*cos(theta)
        M[2,2] = IT
        M[3,3] = 2
        M_inv = inv(M)

        B = SX.zeros(4,1)
        B[0,0] = -cos(theta)*mB*self.d*(theta_dot*theta_dot)
        B[1,0] = -sin(theta)*mB*self.d*(theta_dot*theta_dot)

        C = SX.zeros(2,4)
        C[0,0] = sin(theta+gamma)
        C[0,1] = -cos(theta+gamma)
        C[0,2] = (D)*cos(gamma)
        C[0,3] = 0
        C[1,0] = sin(theta)
        C[1,1] = -cos(theta)
        C[1,2] = 0
        C[1,3] = 0


        Cdot = SX.zeros(2,4)
        Cdot[0,0] = cos(theta+gamma)*(theta_dot+gamma_dot)
        Cdot[0,1] = sin(theta+gamma)*(theta_dot+gamma_dot)
        Cdot[0,2] = -(D)*sin(gamma)*gamma_dot
        Cdot[0,3] = 0
        Cdot[1,0] = cos(theta)*theta_dot
        Cdot[1,1] = sin(theta)*theta_dot


        lambdas=-inv(C@M_inv@C.T)@(C@M_inv@(T-B)+Cdot@q_dot)
        q_dd= M_inv@(T-B+C.T@lambdas)
        q_dd = vertcat(q_dd,0,0,0,0)
        self.f = Function('f', [states,Input], [q_dd])
        
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
        executor = MultiThreadedExecutor(num_threads=4)
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
