from typing import Counter
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import SingleThreadedExecutor

import numpy as np
class Simple_IgoNeo_Commands(Node):
    def __init__(self):

        super().__init__("simple_commander")
        
        self.Cmdpublisher_ =  self.create_publisher(JointState, "joint_command", 10)
        self.Jointsubscriper_ = self.create_subscription(JointState, "joint_states", self.joint_listener, 2)
        
        # Create a JointState message
        self.joint_state = JointState()
        self.joint_state_vel = JointState()
        self.joint_state_pos = JointState()
        self.joint_state.name = ["chassis_drehschemel","drehschemel_antriebsrad"]
        self.joint_state_vel.name = ["drehschemel_antriebsrad"]
        self.joint_state_pos.name = ["chassis_drehschemel"]
        self.counter = 0
        self.response = False
        self.load = True
        

        
    def joint_listener(self,msg):
        self.vel_cmd = [float(0.0),float(0.0)]
        self.pos_cmd = [float(0.0),float(0.0)]
        #self.joint_state.velocity = self.vel_cmd
        self.joint_state.position == self.pos_cmd
        if self.response:
            if self.counter < 1000 and self.counter > 100:
                self.pos_cmd = [float(np.pi/2),float(0.0)]
                self.joint_state.position = self.pos_cmd
            elif self.counter > 1000 and self.counter < 2000:
                self.pos_cmd = [float(0.0),float(0.0)]
                self.joint_state.position = self.vel_cmd
            elif self.counter >2000 and self.counter < 3000:
                self.pos_cmd = [float(-np.pi/2),float(0.0)]
                self.joint_state.position = self.pos_cmd
            elif self.counter > 3000:
                self.pos_cmd = [float(0.0),float(0.0)]
                self.joint_state.position = self.pos_cmd
                self.get_logger().info("Log Finished !")
                
        if self.load:
            if self.counter > 100 and self.counter < 500:
                self.joint_state.velocity = [float(0.0),float(-4/0.127)]
                now1 = self.get_clock().now()
                self.joint_state.header.stamp = now1.to_msg()
                self.Cmdpublisher_.publish(self.joint_state)
                #self.joint_state_pos.position = [float(0.0)]
            if self.counter > 1000:
                self.joint_state.position = [float(np.pi/5),float(0.0)]
                self.joint_state_pos.position = [float(np.pi/7)]
                now1 = self.get_clock().now()
                self.joint_state.header.stamp = now1.to_msg()
                self.Cmdpublisher_.publish(self.joint_state)
            
            
 
        self.counter = self.counter+1 
        now1 = self.get_clock().now()
        #self.joint_state.header.stamp = now1.to_msg()
        #self.Cmdpublisher_.publish(self.joint_state_vel)
        #self.Cmdpublisher_.publish(self.joint_state_pos)
    
def main(args=None):
    rclpy.init(args=args)

    try:
        simple_igoneo_command_node = Simple_IgoNeo_Commands()
        executor = SingleThreadedExecutor()
        executor.add_node(simple_igoneo_command_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            simple_igoneo_command_node.destroy_node()
    # Destroy the node explicitly  
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    
    
# Bag command
#ros2 bag record -o igoneo_resp2 /odom  /joint_command