
from logging import error
from pickletools import uint8
from nav2_msgs.msg import Costmap
import numpy as np
import array as arr #importing array module

class MPCC_CostMap(object):
    def __init__(self,ros_map):
        
        # Initialize the map variables
        self._ros_map = ros_map
        self._grid_data = None
        self._cost_map_metadata = None
        self._reference_frame = None

        if ros_map != None:                                 # If map is not empty then create the map
            
            self._cost_map_metadata = ros_map.info

            self._reference_frame = ros_map.header.frame_id

            self._grid_data = self.create_cost_map(ros_map.data, self._cost_map_metadata)
        else:
            raise error("The Passed ros map is empty!!")


    @property
    def resolution(self):
        return self._cost_map_metadata.resolution

    @property
    def width(self):
        return  self._cost_map_metadata.width

    @property
    def height(self):
        return  self._cost_map_metadata.height

    @property
    def origin(self):
        return  self._cost_map_metadata.origin

    @property
    def reference_frame(self):
        return self._reference_frame
    
    @property
    def cost_data(self):
        return self._grid_data
    
   
    # Create 2D Cost Map from Ros Map data
    def create_cost_map(self,ros_map_data,ros_map_metadata):
            grid_data = np.array(ros_map_data,
                                   dtype=np.int8).reshape(ros_map_metadata.height, ros_map_metadata.width)
            
            grid_data = np.flipud(grid_data)
            return grid_data

    # Create a costmap msg from the occupancy grid map
    def get_ros_cost_map(self):

        self.ros_cost_map = Costmap()
        self.ros_cost_map.header = self._ros_map.header
        self.ros_cost_map.metadata.map_load_time = self._ros_map.info.map_load_time
        self.ros_cost_map.metadata.resolution = self._ros_map.info.resolution
        self.ros_cost_map.metadata.size_x = self._ros_map.info.width
        self.ros_cost_map.metadata.size_y = self._ros_map.info.height
        self.ros_cost_map.metadata.origin = self._ros_map.info.origin 
        data = arr.array('B',np.uint8(self._ros_map.data))
        self.ros_cost_map.data = data


        return self.ros_cost_map


    