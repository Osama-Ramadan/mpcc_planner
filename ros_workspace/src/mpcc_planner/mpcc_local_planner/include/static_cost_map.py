
from logging import error
import numpy as np

# This Class takes a costmap message as map input ----- 

class StaticCostMap(object):
    def __init__(self,ros_map):
        
        # Initialize the map variables
        self._ros_map = ros_map
        self._grid_data = None
        self._cost_map_metadata = None
        self._reference_frame = None
        self._cleared_grid = None
        if ros_map != None:                                 # If map is not empty then create the map
            
            self._cost_map_metadata = ros_map.metadata

            self._reference_frame = ros_map.header.frame_id

            self._grid_data = self.create_cost_map(ros_map.data, self._cost_map_metadata)
        else:
            raise error("The Passed ros map is empty!!")


    @property
    def resolution(self):
        return self._cost_map_metadata.resolution

    @property
    def width(self):
        return  self._cost_map_metadata.size_x

    @property
    def height(self):
        return  self._cost_map_metadata.size_y

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
                                   dtype=np.int8).reshape(ros_map_metadata.size_y, ros_map_metadata.size_x)

        grid_data = np.flipud(grid_data)
        self._cost_map_metadata.origin.position.x = float(0)
        self._cost_map_metadata.origin.position.y = float(ros_map_metadata.size_y*ros_map_metadata.resolution)
        return grid_data

    def clear_cost_map(self):
        self._grid_data = np.zeros([self._cost_map_metadata.size_y,self._cost_map_metadata.size_x])

    # Convert pose from grid indeces to world pose
    def get_world_x_y(self,costmap_y,costmap_x):                        
        world_x = costmap_x * self.resolution + self.origin.position.x
        world_y = self.origin.position.y - costmap_y * self.resolution
        return np.column_stack((world_x, world_y))

   
    # Convert pose from world pose to grid indeces
    def get_costmap_x_y(self, world_x, world_y):      
        # Check if the point outside of the map               
        costmap_x = int(round((world_x - self.origin.position.x) / self.resolution))
        costmap_y = int(round((abs(world_y - self.origin.position.y)) / self.resolution))
        if self.is_in_gridmap(costmap_x,costmap_y):
            return [costmap_y, costmap_x]
        else:
            raise error("One or Both of Given Points are Outside of the Map")

   
    # Check if a given pose is within the cost map
    def is_in_gridmap(self, x, y):                                      
        if -1 < x < self.width and -1 < y < self.height:
            return True
        else:
            return False

   
    # Get the cost value of a specific index in cost map
    def get_cost_from_costmap_x_y(self, x, y):                          
        if self.is_in_gridmap(x, y):
            # first index is the row, second index the column
            return self._grid_data[y][x]
        else:
            raise IndexError(
                "Coordinates out of gridmap, x: {}, y: {} must be in between: [0, {}], [0, {}]".format(
                    x, y, self.height, self.width))

   
    # Get the cost value of a specific pose in world
    def get_cost_from_world_x_y(self, x, y):                            
        cx, cy = self.get_costmap_x_y(x, y)
        try:
            return self.get_cost_from_costmap_x_y(cx, cy)
        except IndexError as e:
            raise IndexError("Coordinates out of grid (in frame: {}) x: {}, y: {} must be in between: [{}, {}], [{}, {}]. Internal error: {}".format(
                self.reference_frame, x, y,
                self.origin.position.x,
                self.origin.position.x + self.height * self.resolution,
                self.origin.position.y,
                self.origin.position.y + self.width * self.resolution,
                e))
    
    # Get a local cost map around a point in the global cost map
    def get_local_cost_map(self,origin,x_max,x_min,y_max,y_min):
        
        # Get Index of the Origin Point in the Global Cost Map
        origin_in_costmap = self.get_costmap_x_y(origin[0],origin[1])

        # Calculate the local map boundries cosidering the bounderies of the global map
        min_x_in_costmap = np.round(max((origin_in_costmap[1]-x_min/self.resolution),0))
        max_x_in_costmap = np.round(min((origin_in_costmap[1]+x_max/self.resolution),self.width))
        min_y_in_costmap = np.round(max((origin_in_costmap[0]-y_max/self.resolution),0))
        max_y_in_costmap = np.round(min((origin_in_costmap[0]+y_min/self.resolution),self.height))
        
        # Extract local Map from the global Map
        local_cost_map = self._grid_data[int(min_y_in_costmap):int(max_y_in_costmap),
                                         int(min_x_in_costmap):int(max_x_in_costmap)]

        ul_corner_global = [min_y_in_costmap,min_x_in_costmap]

        return [local_cost_map.copy(),ul_corner_global]


    # Get a list of obstalce locations around a point in world
    def get_obst_around_pose(self,point,x_search_min,x_search_max, y_search_min,y_search_max):
        
        # Create a local map around the point
        [local_cost_map,ul_corner] = self.get_local_cost_map(point,x_search_max,x_search_min,y_search_max,y_search_min) 
        
        # Get a list of all obstalces in the local cost map 
        obst_locations = np.argwhere(local_cost_map > 0)

        # Calculate the Position of the Obstacles in the Global CostMap
        obst_locations = obst_locations + ul_corner
        obst_locations = self.get_world_x_y(obst_locations[:,0],obst_locations[:,1])
        return obst_locations
    