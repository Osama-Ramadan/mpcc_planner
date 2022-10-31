from rclpy.node import Node
import rclpy

from dynamic_obstacle.kalman_filter import KALMAN_FILTER 
from dynamic_obstacle.dynamic_obstacle import DYNAMIC_OBSTACLE
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

from mpcc_interfaces.msg import LocalPath


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import time


class DynamicCollisionAvoidance(Node):
    def __init__(self):
        super().__init__("dynamic_collision_avoidance")

        # Create Publishers and subscribers
        self.predicted_traj_publisher_ =  self.create_publisher(PoseArray, "obs_predicted_traj", 1)
        self.collision_pt_publisher_ =  self.create_publisher(PoseArray, "collision_pts", 1)
        self.local_path_subscriber_ = self.create_subscription(LocalPath, "local_path", self.local_path_listener, 5)

        # Buffer to hold the tf transformation information
        self.tf_buffer = Buffer() 

        # listener for retrieving the tf info
        self.tf_listener = TransformListener(self.tf_buffer, self) 

        # Parameters
        self.prediction_horizon = 30
        self.timer_period = 0.15             #(s)  

        # Create Timer 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize Dynamic Obstacle and Kalman Filter Objects
        self.obst = DYNAMIC_OBSTACLE('antriebsrad', 'obs1_f',1)
        self.kalman_filter = KALMAN_FILTER(self.timer_period, self.prediction_horizon)

        # Create Local Path Object 
        self.local_path = self.Local_Path()

    # Function to retrieve the Current Position of a target prim
    def get_pose(self, reference, target):

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(reference,target,now)
        except TransformException as ex:
            #self.get_logger().info(f'Could not transform: {ex}')
            return -1
        except Exception as ex1:
            self.get_logger().error("Could not Transform the Prim")
            return -1

        pose = [0,0]
        pose[0] = trans.transform.translation.x
        pose[1] = trans.transform.translation.y

        return pose

    # Get the Current obstacle Position, Update the Filter model and Publishes the new Trajectory
    def timer_callback(self):    # Takes maximum 20ms
        st = time.time()
        if self.local_path.n_path_segs == 0:
            #self.get_logger().info("Path is not received yet")
            return
        
        targets = self.obst.get_target_prim()
        obstacle_pose_f = self.get_pose('Map_orig', targets[0])
        obstacle_pose_b = self.get_pose('Map_orig', targets[1])

        if obstacle_pose_f == -1 or obstacle_pose_b == -1:
            #self.get_logger().error("Obstacle Current Position (Failed to Retrieve)")
            return

        self.obst.update_pose(obstacle_pose_f,obstacle_pose_b)
        [state_estimate_f,state_estimate_b,cov_estimate_f,cov_estimate_b] = self.kalman_filter.update_estimation(self.obst)
        self.obst.update_estimated_state(state_estimate_f,state_estimate_b)
        self.obst.update_estimated_cov(cov_estimate_f,cov_estimate_b)

        # Generate Predicted Trajectory for both forward and backward point on the obstacle
        predicted_trajectory_f = self.kalman_filter.predict_trajectory(state_estimate_f)
        predicted_trajectory_b = self.kalman_filter.predict_trajectory(state_estimate_b)
                
        predicted_trajectory_f = predicted_trajectory_f[:,0:2]
        predicted_trajectory_b = predicted_trajectory_b[:,0:2]


        # check if there is a collision with the path
        result = self.check_collision(predicted_trajectory_f,predicted_trajectory_b,self.local_path)
        coll_pt = np.array([result[1],result[2]]).reshape(2,2)
        coll_pt_msg = self.build_ros_msg(coll_pt,"collision",self.obst)
        
        trajectory_f = self.build_ros_msg(predicted_trajectory_f,"forward",self.obst)
        trajectory_b = self.build_ros_msg(predicted_trajectory_b,"backward",self.obst)
        
        self.collision_pt_publisher_.publish(coll_pt_msg)
        self.predicted_traj_publisher_.publish(trajectory_f)
        self.predicted_traj_publisher_.publish(trajectory_b)
        
        et = time.time()
        elapsed_time = et - st
        #self.get_logger().info("Run Time: %f" %(round(elapsed_time*1000,3)))

    def check_collision(self,obstacle_traj_f,obstacle_traj_b,local_path):

        # Check if any point on the segment will collide with the trajectory of the forward point
        collision_t_f = -1
        min_dist = 1000
        for j in range(len(obstacle_traj_f)-1,-1,-1):
            for i in range(len(local_path.localpath)):
                path_pt = local_path.localpath[i]
                dist = np.sqrt(np.sum((path_pt - obstacle_traj_f[j]) ** 2, axis=0))
                if min_dist > dist:
                    min_dist = dist
                if dist < (self.obst.r + 0.5):
                    collision_t_f = i
                    break
            else:
                continue
            break
            
        # True if there is a collision, Then just examin 20 point before the collision point and 10 point after for backward point
        if collision_t_f != -1 and min_dist < 3:
            collision_seg = int(collision_t_f/local_path.path_sampling_para)+1    
            for j in range(len(obstacle_traj_b)-1,-1,-1):
                for i in range(max(collision_t_f-20,(collision_seg-1)*local_path.path_sampling_para),min(collision_t_f+20,(collision_seg)*local_path.path_sampling_para)):
                    path_pt = local_path.localpath[i]
                    dist = np.sqrt(np.sum((path_pt - obstacle_traj_b[j]) ** 2, axis=0))
                    if dist < (self.obst.r + 0.5):
                        return [True,local_path.localpath[collision_t_f],local_path.localpath[i]]               # Both points collide with the path
            return [True,local_path.localpath[collision_t_f],[-1,-1]]                                                # Only forward point collide
        elif min_dist < 3:
            for j in range(len(obstacle_traj_b)-1,-1,-1):
                for i in range(len(local_path.localpath)):
                    path_pt = local_path.localpath[i]
                    dist = np.sqrt(np.sum((path_pt - obstacle_traj_b[j]) ** 2, axis=0))
                    if dist < (self.obst.r + 0.5):
                        return [True,[-1,-1],local_path.localpath[i]]                          # Only backward point collide
        return[False,[-1,-1],[-1,-1]]
        
    def check_intersection(self,obstacle_traj, local_path):
        
        def ccw(A,B,C):
            return (C[1]-B[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-B[0])

        obstacle_fp = obstacle_traj[0]
        obstacle_lp  = obstacle_traj[len(obstacle_traj)-1]
        path_res = local_path.path_sampling_para

        for i in range(local_path.n_path_segs):
            seg_fp = local_path.localpath[(i*path_res),:]
            seg_lp = local_path.localpath[((i+1)*path_res)-1,:]
            seg_mp_min = local_path.localpath[(i*int(path_res/2))-5,:]
            seg_mp_max = local_path.localpath[(i*int(path_res/2))+5,:]

            if ((ccw(seg_fp,obstacle_fp,obstacle_lp)  != ccw(seg_mp_max,obstacle_fp,obstacle_lp)) and (ccw(seg_fp,seg_mp_max,obstacle_fp) != ccw(seg_fp,seg_mp_max,obstacle_lp))):
                return [True,i+1,0]
            if ((ccw(seg_mp_min,obstacle_fp,obstacle_lp)  != ccw(seg_lp,obstacle_fp,obstacle_lp)) and (ccw(seg_mp_min,seg_lp,obstacle_fp) != ccw(seg_mp_min,seg_lp,obstacle_lp))):
                return [True,i+1,1]

        return [False,0,0]

    # Extract the Local Path from the ros msg
    def local_path_listener(self,msg):
        if len(self.local_path.localpath) > 0:
            return
        self.local_path.n_path_segs = msg.nseg
        self.local_path.localpath = np.dstack([msg.sampled_pt_x,msg.sampled_pt_y])[0]
        self.local_path.path_sampling_para = msg.resolution

    # Convert the Obstacle Trajectory into Ros msg
    def build_ros_msg(self, trajectory, type, obst):
        Pose_array = PoseArray()
        poses = []

        for i in range(len(trajectory)):
            pose = Pose()
            pose.position.x = float(trajectory[i,0])
            pose.position.y = float(trajectory[i,1])
            pose.position.z = float(0.0)
            poses.append(pose)

        Pose_array.poses = poses
        now = self.get_clock().now()
        Pose_array.header.stamp = now.to_msg()
        
        if type == "forward":
            Pose_array.header.frame_id = "f/" + str(obst.get_id())
        elif type == "backward":
            Pose_array.header.frame_id = "b/" + str(obst.get_id())
        else:
            Pose_array.header.frame_id = "c/" + str(obst.get_id())


        return Pose_array
    
    class Local_Path:
        n_path_segs = 0
        localpath = []
        path_sampling_para = 0