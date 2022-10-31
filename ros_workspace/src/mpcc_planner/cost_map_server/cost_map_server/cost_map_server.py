import rclpy
from rclpy.node import Node

from mpcc_interfaces.srv import MapRequest
from nav2_msgs.srv import LoadMap
from nav2_msgs.msg import Costmap
from include_cost.MPCC_CostMap import MPCC_CostMap

class CostMapServer(Node):
    def __init__(self):
        super().__init__("cost_map_server")

        ## Create Service Definitions and Subscripers ##
        self.map_request_srv_ = self.create_service(MapRequest, 'global_map_request', self.global_map_request_callback)


        ## Create Clinet Definitions ##
        self.cost_map_cli = self.create_client(LoadMap, 'map_server/load_map')
        while not self.cost_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')

        ## Create Publisher Definition ## 
        self.map_publisher_ =  self.create_publisher(Costmap, "cost_map", 10)

        ## Create Cost Map Object ## 
        self.cost_map = None

        ## Request Map from the map_server node ## 
        self.map_url = '/home/ros2_workspace/src/mpcc_planner/mpcc_local_planner/maps/ware_house.yaml'
        req = LoadMap.Request()
        req.map_url = self.map_url
        future = self.cost_map_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result.result == 0:
            self.get_logger().info('Map is Loaded Successfully')
        else:
            self.get_logger().error('Map was not Loaded !! ')
        self.ros_map = result.map                   # Store the map
        

        # Create the Cost Map Object
        self.cost_map = MPCC_CostMap(self.ros_map)

        self.map_publisher_.publish(self.cost_map.get_ros_cost_map())   # Publish map Once

    def global_map_request_callback(self,request,response):
        
        while self.cost_map == None:
            self.get_logger().info("Map is not loaded yet!!")
            response.costmap = Costmap()
            return response

        response.costmap = self.cost_map.get_ros_cost_map()
        return response