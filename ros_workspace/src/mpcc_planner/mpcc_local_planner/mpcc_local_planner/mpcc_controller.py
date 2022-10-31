from sys import path
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from include.igoneo_mpc import IGONEO
from include.static_cost_map import StaticCostMap
from include.bounding_boxes import BOUNDING_BOXES

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray,Pose 
from std_msgs.msg import Float32, Float32MultiArray
from nav2_msgs.msg import Costmap
from mpcc_interfaces.msg import LocalPath
from mpcc_interfaces.msg import StaticConstraints
from mpcc_interfaces.srv import MapRequest
from nav_msgs.msg import Odometry
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

import numpy as np
from scipy.spatial import KDTree
import time


#Import CasADi 
path.append(r"/home/casadi-linux-py36-v3.5.5-64bit")
from casadi import *


class MPCC_CONTROLLER(Node):
    
    def __init__(self):
        super().__init__("mpcc_controller")


        ## Create Callback groups 1) for Feedback ,,, 
        group1 = MutuallyExclusiveCallbackGroup()
        group2 = MutuallyExclusiveCallbackGroup()

        ## Define Client Definition ##
        self.req_map_cli = self.create_client(MapRequest, 'global_map_request')
        while not self.req_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...') 
            
        ## Create Publishers and Subscribers ##
        self.cmdpublisher_ =  self.create_publisher(JointState, "joint_command", 1)                                    # Publish velocity commands to the simulator
        self.mpcc_pred_traj_pub_ = self.create_publisher(PoseArray, "mpcc_predicted_traj", 1)                          # Publish the predicted state generated from the solver
        
        
        ## Create Publishers for data to be logged ## 
        self.mpcc_time_pub_ = self.create_publisher(Float32, "mpcc_time", 1)  
        self.step_time_pub_ = self.create_publisher(Float32, "step_time", 1) 
        self.mpcc_feedback_pub_ = self.create_publisher(Float32MultiArray, "mpcc_feedback", 1)  
        self.path_boundry_pub_ = self.create_publisher(Float32MultiArray, "path_boundary", 1)  
        self.collision_time_pub_ = self.create_publisher(Float32, "collision_time", 1)  
        self.mpcc_torque_pub_ = self.create_publisher(Float32,"mpcc_torque",1)
        
        
        self.steering_subscriber_ = self.create_subscription(JointState, "joint_states", self.steering_listener, 2, callback_group= group1)    # Subscribe to the steering angle values from simulation
        self.odom_subscriber_ = self.create_subscription(Odometry, "vehicle_odom", self.vehicle_odom_listener, 2, callback_group= group1)      # Subscribe to the odom values of the vehicle
        self.obst_traj_subscriber_ = self.create_subscription(PoseArray, "obs_predicted_traj", self.obst_traj_listener, 2,callback_group= group1)      # Subscribe to the predicted obstalce trajectory
        self.local_path_subscriber_ = self.create_subscription(LocalPath, "local_path", self.local_path_listener, 5,callback_group=group1)
        #self.static_constraints_subscriber_ = self.create_subscription(StaticConstraints, "static_constraints", self.static_constraints_listener,5,callback_group=group2)
        self.collision_points_subscriber_ = self.create_subscription(PoseArray, "collision_pts", self.collision_pts_listener,5,callback_group=group1)
        
        
        ## Flag Variables
        self.ispath_boundary_created = False  
        
        
        ## Create tf listner ## 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        ## Create Controller Timer callback ##
        self.mpcc_dt_ = 0.15        #(s)
        self.mpcc_timer = self.create_timer(self.mpcc_dt_, self.mpcc_timer_callback)

        ## Initialize MPC ## 
        self.N = 25
        self.igoneo = IGONEO()
        self.driving_direction = "b"
        self.igoneo.Init_MPCC(self.mpcc_dt_,self.N,self.driving_direction)

        ## Create Cmd Message ##
        self.joint_state = JointState()                                                 # Initialize the Joint State Message
        self.joint_state.name = ["chassis_drehschemel","drehschemel_antriebsrad"]       # Set the joint names in the simulator model

        ## Feedback Buffers ##
        self.joint_buffer = np.zeros((2,1))
        self.odom_buffer = np.zeros((3,1))
        self.feedback = np.zeros((self.igoneo.get_n_states(),1))

        ## Create Initial Local Path Object ##
        self.local_path = self.Local_Path()
        
        ## Create Dynamic Obstacle Object and Set Avoidance Paramters##
        self.dynamic_obst1 = self.Dynamic_Obstacle()
        self.dynamic_obst1.id = 1
        self.collision_counter = 0
        self.threshold_time = 15    #(s)

        self.min_coll_dist = 3
        self.min_prev_dist = 1000
        self.min_prev_dist = 1000
        
        ## Get the Global Map from the service ## 
        self.costmap = 0
        costmap_serv = Costmap()
        while len(costmap_serv.data) < 1:
            req = MapRequest.Request()
            future = self.req_map_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            costmap_serv = result.costmap

        self.costmap = StaticCostMap(costmap_serv) 
        
        ## Create Bounding Boxes Object ##
        self.bounding_boxes = BOUNDING_BOXES(self.costmap)
        self.boundary_table = {}
        ## Create Bounding Boxes Timer callback ##
             
    def mpcc_timer_callback(self):
        
        if self.costmap == 0 or not(self.ispath_boundary_created):
            return
        
        # Update the State of the IgoNeo and returns a copy of the state
        feedback = self.update_state()
        st = time.time()
        [mpcc_flag , predicted_traj] = self.mpcc_run(feedback,self.local_path,self.boundary_table)
        et = time.time()
        elapsed_time = et - st
        
        # Publish step time
        step_time = Float32()
        step_time.data = float(elapsed_time)
        self.step_time_pub_.publish(step_time)
        
        # Publish the Predicted Trajectory on MPCC run success
        if mpcc_flag and len(predicted_traj)!=0:
            [predicted_traj_msg_f,predicted_traj_msg_b] = self.create_ros_pose_array(predicted_traj)
            self.mpcc_pred_traj_pub_.publish(predicted_traj_msg_f)
            self.mpcc_pred_traj_pub_.publish(predicted_traj_msg_b)
            #self.get_logger().info("MPCC Run Time: %f" %(round(elapsed_time*1000,3)))

    def mpcc_run(self,feedback,local_path,path_boundary_table):
        # Check if the feedback is available for the IgoNeo
        if (len(feedback)<1) or (len(local_path.Qx)<1) or (len(local_path.Qy)<1):
            #self.get_logger().error("The Feedback of the IgoNeo or the Path is not Available")
            return [False,[]]
        
        solver = self.igoneo.get_mpcc_solver()
        args = self.igoneo.get_mpcc_args()
        n_states = self.igoneo.get_n_states()
        n_controls = self.igoneo.get_n_controls()
        
        # Get Collision Points with Dynamic Obstacles if exists and initialize collision distance
        collision_pt_f = list(self.dynamic_obst1.collision_pt_f)
        collision_pt_b = list(self.dynamic_obst1.collision_pt_b)
        
        # Get the closest point on the path to the current vehicle pose and current seg
        if self.driving_direction == "f":
            d , closest_path_point_index = local_path.localpath_tree.query((float(feedback[0]),float(feedback[1])))
            current_pose = np.array([float(feedback[0]),float(feedback[1])])
        elif self.driving_direction == "b":
            d , closest_path_point_index = local_path.localpath_tree.query((float(feedback[9]),float(feedback[10])))
            current_pose = np.array([float(feedback[9]),float(feedback[10])])

        # Get the current segment of the path
        current_seg = int(closest_path_point_index / local_path.path_sampling_para)+1

        # Get the index of the next waypoint in the waypoints list
        next_waypoint_index = min(current_seg,len(local_path.waypoints)-1)
        next_waypoint = np.array(local_path.waypoints[next_waypoint_index])

        # Get the path progress parameter
        t_t0 = (closest_path_point_index % local_path.path_sampling_para)*(1/local_path.path_sampling_para)
        
        # Get the colser collision Point to IgoNeo
        if len(self.dynamic_obst1.trajectory_f)!=0 or len(self.dynamic_obst1.trajectory_b)!=0:
            obstacle_trajectory = []
            if collision_pt_b[0]!=-1 or collision_pt_f[0]!=-1:
                self.collision_counter = self.collision_counter+1
            else:
                self.collision_counter = np.max(self.collision_counter-1,0)
        
            if self.collision_counter > 5:
                
                if collision_pt_b[0]!=-1 or collision_pt_f[0]!=-1:
                    self.collision_counter = 10 
                
                coll_f_dist = np.sqrt(np.sum((current_pose - self.dynamic_obst1.trajectory_f[0]) ** 2, axis=0))
                coll_b_dist = np.sqrt(np.sum((current_pose - self.dynamic_obst1.trajectory_b[0]) ** 2, axis=0))
                
                if coll_f_dist <=coll_b_dist:
                    #d_o, closest_path_point_index_obs = local_path.localpath_tree.query((float(self.dynamic_obst1.trajectory_f[0][0]),float(self.dynamic_obst1.trajectory_f[0][1])))
                    coll_dist = coll_f_dist
                    obstacle_trajectory = self.dynamic_obst1.trajectory_f
                else:
                    #d_o, closest_path_point_index_obs = local_path.localpath_tree.query((float(self.dynamic_obst1.trajectory_b[0][0]),float(self.dynamic_obst1.trajectory_b[0][1])))
                    coll_dist = coll_b_dist
                    obstacle_trajectory = self.dynamic_obst1.trajectory_b
                    
                # Handle Dynamic Obstacle
                time_to_collision = coll_dist/abs(self.v_body)
                collision_time = Float32()
                collision_time.data = float(time_to_collision)
                self.collision_time_pub_.publish(collision_time)
                #self.get_logger().info("Time to Collision: %f" %(time_to_collision))
            
                if (time_to_collision < self.threshold_time) or (coll_dist <= self.min_coll_dist):
                    self.min_prev_dist = float(coll_dist)
                    solver = self.igoneo.handle_dynamic_obstacle(obstacle_trajectory,self.N)
                elif((coll_dist <= self.min_prev_dist) and (self.min_prev_dist!=1000)):
                    self.min_prev_dist = coll_dist
                    solver = self.igoneo.handle_dynamic_obstacle(obstacle_trajectory,self.N)
                else:
                    self.min_prev_dist = 1000
        
        # If you are close to the waypoint switch to the next one
        if np.sqrt(np.sum((next_waypoint - current_pose) ** 2, axis=0)) < 1:
            if current_seg+1 <= local_path.n_path_segs:
                current_seg = current_seg+1
                t_t0 = (1/local_path.path_sampling_para)

        feedback[8] =  float(t_t0)
        
        # Publish the feedback msg
        feedback_msg = Float32MultiArray()
        feedback_msg.data = list(np.array(feedback,dtype=float).flatten())
        self.mpcc_feedback_pub_.publish(feedback_msg)
        
        # Update the args and P parameters
        args.p = np.zeros((n_states+10,1))
        args.p[0:n_states] = feedback
        args.p[n_states:n_states+4]   = np.array(local_path.Qx[(current_seg-1)*4:(current_seg*4)]).reshape(4,1)
        args.p[n_states+4:n_states+8] = np.array(local_path.Qy[(current_seg-1)*4:(current_seg*4)]).reshape(4,1)
        args.p[n_states+8] = next_waypoint[0]
        args.p[n_states+9] = next_waypoint[1]

        # Set the Boundary constraints
        predicted_vehicle_traj = self.igoneo.get_predicted_trajctory()
        if len(predicted_vehicle_traj)>1:

            for i in range(len(predicted_vehicle_traj)-2):
                traj_pt = predicted_vehicle_traj[i]
                d_f , closest_path_point_index_f = local_path.localpath_tree.query((float(traj_pt[0]),float(traj_pt[1])))
                d_b , closest_path_point_index_b = local_path.localpath_tree.query((float(traj_pt[9]),float(traj_pt[10])))
                
                if d_f < 0.8:  
                    static_constraints_f = path_boundary_table[closest_path_point_index_f]
                                           
                    args.lbx[i*n_states] = static_constraints_f[0]+0.1
                    args.ubx[i*n_states] = static_constraints_f[2]-0.1
                    args.lbx[(i*n_states)+1] = static_constraints_f[3]+0.1
                    args.ubx[(i*n_states)+1] = static_constraints_f[1]-0.1
                else:
                    pass
                    # Apply the Online Search
                if d_b < 0.8:
                    static_constraints_b = path_boundary_table[closest_path_point_index_b]
   
                    args.lbx[(i*n_states)+9] = static_constraints_b[0]+0.1
                    args.ubx[(i*n_states)+9] = static_constraints_b[2]-0.1
                    args.lbx[(i*n_states)+10] = static_constraints_b[3]+0.1
                    args.ubx[(i*n_states)+10] = static_constraints_b[1]-0.1
                else:
                    pass
                    # Apply Online Search
                
            #n_boxes = 4
            #n_states_per_box = 2
            #for k in range(0,n_boxes*n_states_per_box,n_states_per_box):
                #traj_pt = predicted_vehicle_traj[k]
                #constraints_f = self.bounding_boxes.compute_bounding_boxes(traj_pt[0:3],"p")
                #constraints_b = self.bounding_boxes.compute_bounding_boxes([traj_pt[9],traj_pt[10],traj_pt[2]],"p")
                #for l in range(n_states_per_box):
                    #args.lbx[(k)*n_states] = constraints_f[0]
                    #args.ubx[(k)*n_states] = constraints_f[2]
                
                    #args.lbx[(k)*n_states+1] = constraints_f[3]
                    #args.ubx[(k)*n_states+1] = constraints_f[1]
                
            
                #for l in range(n_states_per_box):

                    #args.lbx[(k)*n_states+9] = constraints_b[0]
                    #args.ubx[(k)*n_states+9] = constraints_b[2]
                
                    #args.lbx[(k)*n_states+10] = constraints_b[3]
                    #args.ubx[(k)*n_states+10] = constraints_b[1]


        # Update MPCC Initial variables
        X0 = repmat(feedback,1,self.N+1).T
        u0 = self.igoneo.get_last_control_sequence()
        args.x0 = vertcat(reshape(X0.T,n_states*(self.N+1),1),(reshape(u0,n_controls*self.N,1)))

        # Solve the Optimization Problem
        st = time.time()
        sol = solver(x0 = args.x0, lbx = args.lbx, ubx = args.ubx, lbg = args.lbg, ubg = args.ubg, p = args.p)
        et = time.time()

        # Output Time needed to solve the problem
        elapsed_time = et - st
        mpcc_time = Float32()
        mpcc_time.data = float(elapsed_time)
        self.mpcc_time_pub_.publish(mpcc_time)
        #self.get_logger().info("Solver Time: %f" %(round(elapsed_time*1000,3)))

        # Extract The Solution from the sol object
        sol = sol['x'].full().flatten()
        u = reshape(sol[n_states*(self.N+1):].T,n_controls,self.N).T
        self.igoneo.set_last_control_sequence(u)
        
        # Extract the Predicted Trajectory
        state_opt = reshape(sol[:n_states*(self.N+1)].T,n_states,self.N+1).T
        state_opt = state_opt.full()
        self.igoneo.set_predicted_trajectory(state_opt)
        
        # Get Solver Status
        status = solver.stats()
        if(not((status['return_status'])=='Solve_Succeeded')):
            if status['return_status'] == 'Infeasible_Problem_Detected':
                self.get_logger().info(status['return_status'])
                return[False, []]
            elif status['return_status'] == 'Maximum_Iterations_Exceeded':
                self.get_logger().info(status['return_status'])
            #return [False, []] 
        
        # Specify the predicted state in the horizon to apply velocity to joint
        mpcc_state_index = 5                               
        mpcc_state = reshape(sol[n_states*(mpcc_state_index):n_states*(mpcc_state_index+1)],n_states,1)

        if self.driving_direction == "f":
            X_dot = mpcc_state[4]
            Y_dot = mpcc_state[5]
        elif self.driving_direction == "b":
            X_dot = mpcc_state[4]-(self.igoneo.D*mpcc_state[6])
            Y_dot = mpcc_state[5]-(self.igoneo.D*mpcc_state[6])
        
        # Calculate the velocity in body frame and omega
        self.v_body = (X_dot*np.cos(mpcc_state[2])) + (Y_dot*np.sin(mpcc_state[2]))
        tx_u = u[0,:]                                       # Take first input from the solver
        
        # If The Tourqe is High but the velocity in the body frame is very small, give small constant velocity (Corner Case)
        if((abs(self.v_body)< 0.12 and abs(tx_u[0])>=3)):
                self.v_body = (0.2)*np.sign(tx_u[0])

        # Create the Joint Command msg
        self.joint_state.velocity = [float(tx_u[1]), float(self.v_body/self.igoneo.get_wheel_raduis())]

        # Publish the Torque Value
        torque = Float32()
        torque.data = float(tx_u[0])
        self.mpcc_torque_pub_.publish(torque)
        # Logging Velocity Commands
        #self.get_logger().info("V = %f, Omega = %f, Tourque = %f" %(float(self.v_body),float(tx_u[1]),float(tx_u[0])))

        # Logging FeedBack
        #self.get_logger().info("X = %f, Y = %f, Theta = %f, Gamma = %f" %(float(feedback[0]),float(feedback[1]),float(feedback[2]),float(feedback[3]*180/np.pi)))

        # Publish the Joint msg 
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.cmdpublisher_.publish(self.joint_state)

        return [True, state_opt]
    
    
    def check_invalid_constraints(self, static_constraints, prev_const,next_waypoint):
        constraint_isfit = True
        
        if static_constraints[0]>prev_const[2]:             # if Xmin > Xmax
            static_constraints[0]=prev_const[0]
            constraint_isfit = False
        
        if static_constraints[3] > prev_const[1]:           # if Ymin > Ymax
            static_constraints[3] = prev_const[3]
            constraint_isfit = False
                            
        if np.abs(static_constraints[3]-prev_const[3])>1:   # if Ymin - Ymin > 1
            static_constraints[3] = prev_const[3]
            constraint_isfit = False
        
        #if np.abs(static_constraints[0]-prev_const[0])>2:   # if Xmin - Xmin > 1
            #if (abs(next_waypoint[0]-prev_const[0])) < (abs(next_waypoint[0]-static_constraints[0])):
                #static_constraints[0] = prev_const[0]
            #constraint_isfit = False
        
        if np.abs(static_constraints[2]-prev_const[2])>2:   # if Xmax - Xmax > 1
            static_constraints[2] = prev_const[2]
            constraint_isfit = False
        
        if np.abs(static_constraints[1]-prev_const[1])>1:   # if Ymax - Ymax > 1
            static_constraints[1] = prev_const[1]
            constraint_isfit = False

        return [static_constraints,constraint_isfit]
        
    def steering_listener(self,msg):
        self.joint_buffer[0] = msg.position[0]            # gamma
        self.joint_buffer[1] = msg.velocity[0]            # gamma_dot

    def vehicle_odom_listener(self,msg):
        self.odom_buffer[0] = msg.twist.twist.linear.x                # x_dot
        self.odom_buffer[1] = msg.twist.twist.linear.y                # y_dot
        self.odom_buffer[2] = msg.twist.twist.angular.z               # theta_dot

    def obst_traj_listener(self,msg):
        obst_id = int([int(s) for s in (msg.header.frame_id).split("/") if s.isdigit()][0])
        if msg.header.frame_id[0] == "f":
            traj_f = np.zeros([len(msg.poses),2])
            for i in range(len(msg.poses)):
                pose = msg.poses[i]
                traj_f[i] = np.array([pose.position.x, pose.position.y])
            if self.dynamic_obst1.id == obst_id:
                self.dynamic_obst1.trajectory_f = traj_f
        else:
            traj_b = np.zeros([len(msg.poses),2])
            for i in range(len(msg.poses)):
                pose = msg.poses[i]
                traj_b[i] = np.array([pose.position.x, pose.position.y])
            if self.dynamic_obst1.id == obst_id:
                self.dynamic_obst1.trajectory_b = traj_b
            
    def local_path_listener(self,msg):
        if len(self.local_path.localpath) > 0:
            return
        waypoint_list = []
        for i in range(len(msg.waypoints_x)):
            waypoint_list.append([msg.waypoints_x[i],msg.waypoints_y[i]])
        self.local_path.waypoints = waypoint_list
        self.local_path.Qx = msg.qx
        self.local_path.Qy = msg.qy
        self.local_path.n_path_segs = msg.nseg
        self.local_path.localpath = np.dstack([msg.sampled_pt_x,msg.sampled_pt_y,msg.sampled_pt_th])[0]
        self.local_path.localpath_tree = KDTree(self.local_path.localpath[:,0:2])
        self.local_path.path_sampling_para = msg.resolution
        
        if not self.ispath_boundary_created:
            try:
                self.bounding_boxes.create_path_bounding_area(self.local_path)
            except AttributeError:
                return
            self.boundary_table = self.bounding_boxes.get_path_boundary()
            path_bound = Float32MultiArray()
            bound_table = []
            for i in range(len(self.boundary_table)):
                bound = self.boundary_table[i]
                bound_table.append(bound)
            path_bound.data = list(np.array(bound_table).flatten())
            self.path_boundry_pub_.publish(path_bound)
            self.ispath_boundary_created = True
            self.get_logger().info("Computing Path Safe Boundary Finished !!")

    def update_state(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('Map_orig','chassis',now)      # Always get the latest transform
            #trans = self.tf_buffer.lookup_transform('Map_orig','antriebsrad',now)      # Always get the latest transform
        except TransformException as ex:
            #self.get_logger().info(f'Could not transform: {ex}')
            return []
        
        qw = trans.transform.rotation.w
        qz = trans.transform.rotation.z
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        t3 = 2*(qw*qz + qx*qy)
        t4 = 1-2*(qy*qy + qz*qz)
        angle = np.arctan2(t3,t4)
        
        x_body_dot = self.odom_buffer[0]
        y_body_dot = self.odom_buffer[1]
        theta_dot =  self.odom_buffer[2]
        
        self.feedback[0]  = float(trans.transform.translation.x)
        self.feedback[1]  = float(trans.transform.translation.y)
        self.feedback[2]  = float(angle)
        self.feedback[3]  = float(self.joint_buffer[0])
        self.feedback[4]  = float(-x_body_dot*np.cos(angle) - y_body_dot *np.sin(angle))    # float(-x_body_dot*np.cos(angle) - y_body_dot *np.sin(angle))
        self.feedback[5]  = float(-x_body_dot*np.sin(angle) + y_body_dot *np.cos(angle))    # float(-x_body_dot*np.sin(angle) + y_body_dot *np.cos(angle))
        self.feedback[6]  = float(theta_dot)
        self.feedback[7]  = float(self.joint_buffer[1])
        self.feedback[9]  = float(self.feedback[0]-self.igoneo.D*np.cos(angle))
        self.feedback[10] = float(self.feedback[1]-self.igoneo.D*np.sin(angle))
        
        #self.get_logger().info("X_dot = %f, Y_dot = %f, Theta_dot = %f" %(float(self.feedback[4]),float(self.feedback[5]),float(self.feedback[6])))

        return self.feedback.copy()

    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    def create_ros_pose_array(self,traj):
        predicted_traj_msg_f = PoseArray()
        predicted_traj_msg_b = PoseArray()
        traj_list_f = []
        traj_list_b = []
        for i in range(len(traj)):
            current_pose_f = Pose()
            current_pose_b = Pose()
            current_point = traj[i]
            current_angle = self.euler_to_quaternion(current_point[2],0,0)

            current_pose_f.position.x = float(current_point[0])
            current_pose_f.position.y = float(current_point[1])
            
            current_pose_b.position.x = float(current_point[9])
            current_pose_b.position.y = float(current_point[10])
            current_pose_f.orientation.x = float(current_angle[0])
            current_pose_f.orientation.y = float(current_angle[1])
            current_pose_f.orientation.z = float(current_angle[2])
            current_pose_f.orientation.w = float(current_angle[3]) 

            traj_list_f.append(current_pose_f)
            traj_list_b.append(current_pose_b)

        now1 = self.get_clock().now()
        
        predicted_traj_msg_f.header.stamp = now1.to_msg()
        predicted_traj_msg_f.header.frame_id = str("f")
        predicted_traj_msg_f.poses = traj_list_f
        
        predicted_traj_msg_b.header.stamp = now1.to_msg()
        predicted_traj_msg_b.header.frame_id = str("b")
        predicted_traj_msg_b.poses = traj_list_b
        
        return [predicted_traj_msg_f,predicted_traj_msg_b]
        
    def static_constraints_listener(self,msg):
        ul_x = np.array(msg.ul_x).reshape(len(msg.ul_x),1)
        ul_y = np.array(msg.ul_y).reshape(len(msg.ul_y),1)
        lr_x = np.array(msg.lr_x).reshape(len(msg.lr_x),1)
        lr_y = np.array(msg.lr_y).reshape(len(msg.lr_y),1)

        static_constraints = np.dstack([ul_x,ul_y,lr_x,lr_y])
        self.igoneo.set_static_constraints(list(static_constraints),str(msg.point_type))
        
    def collision_pts_listener(self,msg):
        obst_id = int([int(s) for s in (msg.header.frame_id).split("/") if s.isdigit()][0])
        pose_f = msg.poses[0]
        pose_b = msg.poses[1]
        collision_pt_f = np.array([pose_f.position.x, pose_f.position.y])
        collision_pt_b = np.array([pose_b.position.x, pose_b.position.y])
        if self.dynamic_obst1.id == obst_id:
            self.dynamic_obst1.collision_pt_f = collision_pt_f
            self.dynamic_obst1.collision_pt_b = collision_pt_b
              
    class Local_Path:
        path_sampling_para = 0
        waypoints  = []
        n_path_segs = 0
        Qx = []
        Qy = []
        localpath = []
        localpath_tree = []
        
    class Dynamic_Obstacle:
        id = -1
        trajectory_f =  []
        trajectory_b =  []
        collision_pt_f = []
        collision_pt_b = []
