import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from mpc_dynamic_obstacles.dynamic_obstacle import DYNAMIC_OBSTACLE
from mpc_dynamic_obstacles.kalman_filter import KALMAN_FILTER
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import numpy as np


 
class DYNAMIC_OBSTACLES_NODE(Node):
    def __init__(self):
        super().__init__("dynamic_obstacles")

        self.predicted_traj_publisher_ =  self.create_publisher(PoseArray, "predicted_traj", 10)

        self.tf_buffer = Buffer()                                   # Buffer to hold the tf transformation information
        self.tf_listener = TransformListener(self.tf_buffer, self)  # listner to enable retrieving the tf info

        self.prediction_horizon = 20
        self.timer_period = 0.2  # seconds  
        self.timer = self.create_timer(self.timer_period/2, self.timer_callback)

        self.obst = DYNAMIC_OBSTACLE('base_link')
        self.kalman_filter = KALMAN_FILTER(self.timer_period, self.prediction_horizon)

        self.obstacles = []

    def get_poses(self, reference, obstacle):

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(reference,obstacle.get_target_prim(),now)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
            return -1

        pose = [0,0]
        pose[0] = trans.transform.translation.x
        pose[1] = trans.transform.translation.y

        return pose

    def build_ros_msg(self, trajectory):
        Pose_array = PoseArray()
        poses = []
        for i in range(len(trajectory)-1):
            point = Point()
            point.x = float(trajectory[i][0])
            point.y = float(trajectory[i][1])
            point.z = float(0.0)
            pose = Pose()
            pose.position = point
            poses.append(pose)
        Pose_array.poses = poses
        now = self.get_clock().now()
        Pose_array.header.stamp = now.to_msg()
        return Pose_array

    def timer_callback(self):
        obstacle_pose = self.get_poses('opx_l12', self.obst)
        if obstacle_pose == -1:
            return

        self.obst.update_pose(obstacle_pose)
        [state_estimate, cov_estimate] = self.kalman_filter.update_estimation(self.obst)
        self.obst.update_estimated_state(state_estimate)
        self.obst.update_estimated_cov(cov_estimate)
        predicted_trajectory = self.kalman_filter.predict_trajectory(self.obst.get_estimated_state())

        trajectory = self.build_ros_msg(predicted_trajectory)
        self.predicted_traj_publisher_.publish(trajectory)
        
        

def main(args=None):
    rclpy.init(args=args)
    dynamic_obstacles_publisher = DYNAMIC_OBSTACLES_NODE()

    rclpy.spin(dynamic_obstacles_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()