from distutils.log import error
import rclpy
from rclpy.node import Node

from mpcc_interfaces.srv import MapRequest
from mpcc_interfaces.msg import StaticConstraints
from static_cost_map.MPCC_StaticCostMap import MPCC_StaticCostMap
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PoseArray
import numpy as np

class StaticCollisionAvoidance(Node):
    def __init__(self):
        super().__init__("static_collision_avoidance")

        # Create CostMap Object #
        self.costmap = None

        ## Define Client Definition ##
        self.req_map_cli = self.create_client(MapRequest, 'global_map_request')
        while not self.req_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...') 

        ## Define Subscribers Definition ## 
        self.veh_traj_sub_ = self.create_subscription(PoseArray,"mpcc_predicted_traj",self.pred_traj_listener,5)
        self.static_constraints_pub_ = self.create_publisher(StaticConstraints,"static_constraints",5)

        ## Get the Global Map from the service ## 
        costmap_serv = Costmap()
        while len(costmap_serv.data) < 1:
            req = MapRequest.Request()
            future = self.req_map_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            costmap_serv = result.costmap

        self.costmap = MPCC_StaticCostMap(costmap_serv) 

    def pred_traj_listener(self,msg):
        point_type = msg.header.frame_id
        pred_poses = msg.poses
        pred_poses = self.extract_pred_traj_poses(pred_poses)
        n_poses = len(pred_poses)
        ul_corner_x = []
        ul_corner_y = []
        lr_corner_x = []
        lr_corner_y = []
        ul = 0
        lr = 0

        for i in range(n_poses):
            
            pose = pred_poses[i]
            
            max_search_x = 1.5
            max_search_y = 1.5
            x_search_ext = np.abs(np.cos(pose[2])*2)
            y_search_ext = np.abs(np.sin(pose[2])*2)
            max_search_x = max_search_x + x_search_ext
            max_search_y = max_search_y + y_search_ext
            safe_corners = self.get_safe_area(pred_poses[i,0:2],max_search_x,max_search_x,max_search_y,max_search_y)
            if (len(safe_corners) < 1):
                return
            ul = safe_corners[0]
            ul = ul[0]
            lr = safe_corners[1]
            lr = lr[0]
            ul_corner_x.append(float(ul[0]))
            ul_corner_y.append(float(ul[1]))
            lr_corner_x.append(float(lr[0]))
            lr_corner_y.append(float(lr[1]))

        # Create and Publish the Constraints msg
        constraints_msg = StaticConstraints()
        constraints_msg.point_type = point_type
        constraints_msg.ul_x = list(ul_corner_x)
        constraints_msg.ul_y = list(ul_corner_y)
        constraints_msg.lr_x = list(lr_corner_x)
        constraints_msg.lr_y = list(lr_corner_y)
        self.static_constraints_pub_.publish(constraints_msg)

    def extract_pred_traj_poses(self,traj):
        poses_list = np.zeros([len(traj),3])
        for i in range(len(traj)):
            current_pose = traj[i]
            p = current_pose.position
            orient = current_pose.orientation
            angle = self.quaternion_to_euler(orient.x,orient.y,orient.z,orient.w)
            pose = [p.x,p.y,angle]
            poses_list[i,:] = pose
        return poses_list

    def get_safe_area(self,point, search_max_x,search_min_x,search_max_y,search_min_y):
        
        if (self.costmap == None) or (len(self.costmap.cost_data) < 1):
            self.get_logger().error("CostMap is not Avaialable")
            return[]
        [local_search_map,map_ul_corner] = self.costmap.get_local_cost_map(point,search_max_x,search_min_x,search_max_y,search_min_y)
        
        max_area = 0
        max_row = -1
        rows = len(local_search_map)

        if rows < 1 or np.size(local_search_map) < 1:
            raise error("LocalMap is Empty")

        for r in range(rows):
            result = self.get_histogramArea(local_search_map[:r+1])
            if result[0]>max_area:
                max_area = result[0]
                max_height = result[2]
                max_right = result[3]
                max_left = result[4]
                max_row = r

        ul_corner_local = [max_row-max_height+1,max_left]
        lr_corner_local = [max_row,max_right]


        ul_corner_global = [map_ul_corner[0] + ul_corner_local[0] ,map_ul_corner[1] + ul_corner_local[1] ]
        lr_corner_global = [map_ul_corner[0] + lr_corner_local[0] ,map_ul_corner[1] + lr_corner_local[1] ]


        ul_corner_global = self.costmap.get_world_x_y(ul_corner_global[0],ul_corner_global[1])  
        lr_corner_global = self.costmap.get_world_x_y(lr_corner_global[0],lr_corner_global[1]) 


        return [ul_corner_global,lr_corner_global]

    def get_histogramArea(self,rows):
        
        cols = len(rows[0])
        histogram = [0]*cols

        for row in rows:
            for col in range(cols):
                histogram[col] = 0 if not(row[col]==0) else(1 + histogram[col])

        return self.largestRectangleAreaHistogram(histogram)

    def largestRectangleAreaHistogram(self, heights):
        n = len(heights)
        left = [0] * n
        right = [0] * n
        stack = []
        #getting the left elements
        for i in range(n):
            if not stack:
                left[i] = 0
            else:
                while stack and heights[stack[-1]] >= heights[i]:
                    stack.pop()
                left[i] = stack[-1]+1 if stack else 0
            stack.append(i)
        stack.clear()
        
        #getting the right elements
        for i in range(n-1,-1,-1):
            if not stack:
                right[i] = n-1
            else:
                while stack and heights[stack[-1]] >= heights[i]:
                    stack.pop()
                right[i] = stack[-1]-1 if stack else n-1
            stack.append(i)

        #calculating the area
        max_area = 0
        max_height = 0
        max_right = 0
        max_left = 0
        bar_index = 0
        for i in range(n):
            height = heights[i]
            width = right[i]-left[i]+1
            area = height * width
            if area > max_area:
                max_area = area
                max_height = height
                max_right = right[i]
                max_left = left[i]
                bar_index = i
        return [max_area,bar_index,max_height,max_right,max_left]

    def quaternion_to_euler(self,x, y, z, w):
        t3 = 2*(w*z + x*y)
        t4 = 1-2*(y*y + z*z)
        angle = np.arctan2(t3,t4)
        return angle