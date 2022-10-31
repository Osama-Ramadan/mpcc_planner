from rclpy.node import Node
import rclpy

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from dynamic_window.igoneo_dw import IGONEO_DW
from dw_costmap.dw_costmap import DW_COSTMAP
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray,Pose 
from std_msgs.msg import Float32, Float32MultiArray

from mpcc_interfaces.msg import LocalPath
from nav_msgs.msg import Odometry
from mpcc_interfaces.srv import MapRequest

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

import numpy as np
from scipy.spatial import KDTree
import time


class DYNAMIC_WINDOW(Node):
    def __init__(self):
        super().__init__("dynamic_window")
        
        ## Feedback Buffers ##
        self.joint_buffer = np.zeros((2,1))
       
        ## Create Callback groups 1) for Feedback ,,, 
        group1 = MutuallyExclusiveCallbackGroup()
        
        ## Create Publishers and Subscribers ##
        self.cmdpublisher_ =  self.create_publisher(JointState, "joint_command", 1)                                    # Publish velocity commands to the simulator
        
        ## Create Publishers for logging
        self.dw_time_pub_ = self.create_publisher(Float32, "dw_time", 1)  
        self.dw_pred_traj_pub_ = self.create_publisher(PoseArray, "dw_predicted_traj", 1)
        self.dw_cost = self.create_publisher(Float32,"dw_cost",1)
        self.dw_feedback_pub_ = self.create_publisher(Float32MultiArray, "dw_feedback", 1)  



        self.steering_subscriber_ = self.create_subscription(JointState, "joint_states", self.steering_listener, 2, callback_group= group1)    # Subscribe to the steering angle values from simulation
        self.obst_traj_subscriber_ = self.create_subscription(PoseArray, "obs_predicted_traj", self.obst_traj_listener, 2,callback_group= group1)      # Subscribe to the predicted obstalce trajectory
        self.local_path_subscriber_ = self.create_subscription(LocalPath, "local_path", self.local_path_listener, 5,callback_group=group1)
        
        ## Create Client to get the Map ##
        self.req_map_cli = self.create_client(MapRequest, 'global_map_request')
        while not self.req_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...') 
            
        ## Get the Map and Create the Map Object ##
        result = None
        while result == None:
            req = MapRequest.Request()
            future = self.req_map_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()

        self.costmap = DW_COSTMAP(result.costmap)
            
            
        ## Create tf listner ## 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        ## Create Controller Timer callback ##
        self.dw_dt_ = 0.15        #(s)
        self.dw_timer = self.create_timer(self.dw_dt_, self.dw_timer_callback)
        
        ## Initialize DW ## 
        self.driving_direction = "b"
        self.N = 3              #(s)
        self.dt = 0.15           #(s)
        self.dw = IGONEO_DW()
        self.dw.set_dw_configuration(max_speed=0.3,min_speed=-1.7,
                                     max_omega=80*np.pi/180, min_omega=-80*np.pi/180,
                                     max_acc=1,max_alpha=20*np.pi/180,
                                     v_resolution=0.1,omega_resolution=(np.pi/180),
                                     dt=self.dt,pred_horizon=self.N)
        self.dw.set_gains(heading_gain = 5.0, obstacle_gain = 1.0, speed_gain = 3.0, to_goal_gain = 5, dynamic_obst_gain = 2.0)
        self.dw.set_igoneo_parameters(raduis = 0.5, length = 1.827, wheel_r = 0.127)
        
        ## Create Cmd Message ##
        self.joint_state = JointState()                                                 # Initialize the Joint State Message
        self.joint_state.name = ["chassis_drehschemel","drehschemel_antriebsrad"]       # Set the joint names in the simulator model
        
        ## Create Initial Local Path Object ##
        self.local_path = self.Local_Path()
        
        ## Initial Control Parameters
        self.u = np.zeros(((self.dw).n_inputs,1))
        self.feedback = np.zeros((self.dw.n_states,1))
        
        ## Create Dynamic Obstacle Object and Set Avoidance Paramters##
        self.dynamic_obst1 = self.Dynamic_Obstacle()
        self.dynamic_obst1.id = 1
        self.collision_counter = 0
        
    def dw_timer_callback(self):
        
        # Update State
        feedback = self.update_state()
        # Check if the feedback is available for the IgoNeo
        if (len(feedback)<1) or (len(self.local_path.Qx)<1) or (len(self.local_path.Qy)<1):
            return
        
        # Update Dynamic Window
        Dw = self.dw.get_dw(self.u)

        # Get Colsest Point on the Path
        if self.driving_direction == "f":
            d , closest_path_point_index = self.local_path.localpath_tree.query((float(feedback[0]),float(feedback[1])))
        else:
            d , closest_path_point_index = self.local_path.localpath_tree.query((float(feedback[4]),float(feedback[5])))

        # Get the current segment of the path
        next_point_index = min(closest_path_point_index+15,len(self.local_path.localpath)-1)
        next_point_on_path = self.local_path.localpath[next_point_index]
        current_seg = int(closest_path_point_index / self.local_path.path_sampling_para)+1
        
        # Get Dynamic Obstacle Trajectory
        trajectory_f = list(self.dynamic_obst1.trajectory_f)
        trajectroy_b = list(self.dynamic_obst1.trajectory_b)
        
        # Next Waypoint
        #next_waypoint = np.array(self.local_path.waypoints[current_seg]).reshape(2,1)
        #if np.sqrt(np.sum((next_waypoint - feedback[0:2]) ** 2, axis=0)) < 1 and current_seg < len(self.local_path.waypoints):
            #current_seg = current_seg + 1
            #next_waypoint = self.local_path.waypoints[current_seg]

        st = time.time()
        [cmd,traj,cost] = self.dw.compute_control_trajectory(feedback,Dw,np.array(next_point_on_path[0:2]),self.costmap, trajectory_f, trajectroy_b)
        self.u = cmd
        et = time.time()
        self.publish_cmd(cmd)
        
        if cost == np.inf:
            self.get_logger().error("Could not Find A Valid Trajectory")
        [predicted_traj_msg_f,predicted_traj_msg_b] = self.create_ros_pose_array(traj)
        self.dw_pred_traj_pub_.publish(predicted_traj_msg_f)
        self.dw_pred_traj_pub_.publish(predicted_traj_msg_b)
        
        # Output Time needed to solve the problem
        elapsed_time = et - st
        dw_time = Float32()
        dw_time.data = float(elapsed_time)
        self.dw_time_pub_.publish(dw_time)
        
        # Publish cost of optimal trajectory
        min_cost = Float32()
        min_cost.data = float(cost)
        self.dw_cost.publish(min_cost)
        
        # Publish the feedback msg
        feedback_msg = Float32MultiArray()
        feedback_msg.data = list(np.array(feedback,dtype=float).flatten())
        self.dw_feedback_pub_.publish(feedback_msg)
        
        #self.get_logger().info("Solver Time: %f" %(round(elapsed_time*1000,3)))
       
        
    def publish_cmd(self,cmd):
        
        self.joint_state.velocity = [float(cmd[1]), float(cmd[0]/self.dw.wheel_r)]
        
        # Publish the Joint msg 
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.cmdpublisher_.publish(self.joint_state)
    
    
    def steering_listener(self,msg):
        self.joint_buffer[0] = msg.position[0]            # gamma
        self.joint_buffer[1] = msg.velocity[0]            # gamma_dot
        
        
    def local_path_listener(self,msg):
        #if len(self.local_path.localpath) > 0:
         #   return
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
        
        
        self.feedback[0] = float(trans.transform.translation.x)
        self.feedback[1] = float(trans.transform.translation.y)
        self.feedback[2] = float(angle)
        self.feedback[3] = float(self.joint_buffer[0])
        self.feedback[4] = float(self.feedback[0]-(self.dw.igoneo_length*np.cos(angle)))
        self.feedback[5] = float(self.feedback[1]-(self.dw.igoneo_length*np.sin(angle)))
        
        return self.feedback.copy()


    def obst_traj_listener(self,msg):
        obst_id = int([int(s) for s in (msg.header.frame_id).split("/") if s.isdigit()][0])
        if msg.header.frame_id[0] == "f":
            traj_f = np.zeros([len(msg.poses),2])
            for i in range(len(msg.poses)):
                pose = msg.poses[i]
                traj_f[i] = np.array([pose.position.x, pose.position.y])
            try:
                if self.dynamic_obst1.id == obst_id:
                    self.dynamic_obst1.trajectory_f = traj_f
            except AttributeError:
                return
        else:
            traj_b = np.zeros([len(msg.poses),2])
            for i in range(len(msg.poses)):
                pose = msg.poses[i]
                traj_b[i] = np.array([pose.position.x, pose.position.y])
            try:
                if self.dynamic_obst1.id == obst_id:
                    self.dynamic_obst1.trajectory_b = traj_b
            except AttributeError:
                return
    
    
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
            
            current_pose_b.position.x = float(current_point[4])
            current_pose_b.position.y = float(current_point[5])
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
    
    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
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