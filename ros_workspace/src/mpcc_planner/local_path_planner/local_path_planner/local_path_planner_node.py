import rclpy
from rclpy.executors import SingleThreadedExecutor
from local_path_planner.local_path_planner import LOCAL_PATH_PLANNER
def main(args=None):
    rclpy.init(args=args)

    try:
        path_planner_node = LOCAL_PATH_PLANNER()
        executor = SingleThreadedExecutor()
        executor.add_node(path_planner_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            path_planner_node.destroy_node()
    # Destroy the node explicitly  
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()