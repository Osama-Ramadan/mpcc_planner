import rclpy
from rclpy.executors import MultiThreadedExecutor
from igoneo_dynamic_window.igoneo_dynamic_window import DYNAMIC_WINDOW
def main(args=None):
    rclpy.init(args=args)

    try:
        dw_node = DYNAMIC_WINDOW()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(dw_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            dw_node.destroy_node()
    # Destroy the node explicitly  
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()