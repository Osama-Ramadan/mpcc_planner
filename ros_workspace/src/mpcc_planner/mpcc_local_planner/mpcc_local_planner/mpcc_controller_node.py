import rclpy
from rclpy.executors import MultiThreadedExecutor
from mpcc_local_planner.mpcc_controller import MPCC_CONTROLLER
def main(args=None):
    rclpy.init(args=args)

    try:
        mpc_node = MPCC_CONTROLLER()
        executor = MultiThreadedExecutor(num_threads=3)
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