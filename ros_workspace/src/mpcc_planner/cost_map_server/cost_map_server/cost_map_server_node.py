import rclpy
from rclpy.executors import SingleThreadedExecutor
from cost_map_server.cost_map_server import CostMapServer


def main(args=None):
    rclpy.init(args=args)
    try:
        costmap_server_node = CostMapServer()
        executor = SingleThreadedExecutor()
        executor.add_node(costmap_server_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            costmap_server_node.destroy_node()
    # Destroy the node explicitly  
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()