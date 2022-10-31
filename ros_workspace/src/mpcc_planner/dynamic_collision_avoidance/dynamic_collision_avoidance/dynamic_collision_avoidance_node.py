import rclpy
from rclpy.executors import SingleThreadedExecutor
from dynamic_collision_avoidance.dynamic_collision_avoidance import DynamicCollisionAvoidance


def main(args=None):
    rclpy.init(args=args)
    try:
        dynamic_collision_avoidance_node = DynamicCollisionAvoidance()
        executor = SingleThreadedExecutor()
        executor.add_node(dynamic_collision_avoidance_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            dynamic_collision_avoidance_node.destroy_node()
    # Destroy the node explicitly  
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()