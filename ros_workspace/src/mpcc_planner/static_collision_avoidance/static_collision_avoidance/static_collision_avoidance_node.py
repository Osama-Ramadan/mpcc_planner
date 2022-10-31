import rclpy
from rclpy.executors import SingleThreadedExecutor
from static_collision_avoidance.static_collision_avoidance import StaticCollisionAvoidance


def main(args=None):
    rclpy.init(args=args)
    try:
        static_collision_avoidance_node = StaticCollisionAvoidance()
        executor = SingleThreadedExecutor()
        executor.add_node(static_collision_avoidance_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            static_collision_avoidance_node.destroy_node()
    # Destroy the node explicitly  
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()