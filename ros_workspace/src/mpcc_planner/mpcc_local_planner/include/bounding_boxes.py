
import numpy as np

class BOUNDING_BOXES():
    def __init__(self,costmap):
        self.costmap = costmap
        self.static_constraints_f = []
        self.static_constraints_b = []
        self.path_boundary_table = {}
    
    def get_path_boundary(self):
        return self.path_boundary_table
    
    def set_costmap(self,map):
        self.costmap = map
        
    def get_bounding_boxes(self,type):
        if type == "f":
            return self.static_constraints_f.copy()
        elif type == "b":
            return self.static_constraints_b.copy()
        
    def compute_bounding_boxes(self,traj,type):
        
        if type == "p":
            traj = np.array(traj).reshape(1,3)
        n_poses = np.shape(traj)[0]

        if n_poses < 1:
            return []
        ul_corner_x = []
        ul_corner_y = []
        lr_corner_x = []
        lr_corner_y = []
        ul = 0
        lr = 0

        for i in range(n_poses):     
            pose = traj[i,:]
            init_search_x = 2.2
            init_search_y = 2.2
            x_search_ext = np.cos(pose[2])*1
            y_search_ext = np.sin(pose[2])*1
            max_search_x = init_search_x + x_search_ext
            min_search_x = init_search_x - x_search_ext
            max_search_y = init_search_y + y_search_ext
            min_search_y = init_search_y - y_search_ext
            if type == "f" or type == "p":
                safe_corners = self.get_safe_area(pose[0:2],max_search_x,min_search_x,max_search_y,min_search_y)
            elif type == "b":
                safe_corners = self.get_safe_area(pose[9:11],max_search_x,min_search_x,max_search_y,min_search_y)
            if (len(safe_corners) < 1):
                return []
            ul = safe_corners[0]
            lr = safe_corners[1]
            
            ul_corner_x.append(float(ul[0]))
            ul_corner_y.append(float(ul[1]))
            lr_corner_x.append(float(lr[0]))
            lr_corner_y.append(float(lr[1]))
        
        if type == "f": 
            self.static_constraints_f = np.dstack([ul_corner_x,ul_corner_y,lr_corner_x,lr_corner_y])
            return np.array(self.static_constraints_f[0]).reshape(n_poses*2,4)
        elif type == "b":
            self.static_constraints_b = np.dstack([ul_corner_x,ul_corner_y,lr_corner_x,lr_corner_y])
            return np.array(self.static_constraints_b[0]).reshape(n_poses*2,4)
        else:
            return np.array([ul_corner_x,ul_corner_y,lr_corner_x,lr_corner_y])
        
    def get_safe_area(self,point, search_max_x,search_min_x,search_max_y,search_min_y):
        
        if (self.costmap == None) or (len(self.costmap.cost_data) < 1):
            self.get_logger().error("CostMap is not Avaialable")
            return[]
        [local_search_map,map_ul_corner] = self.costmap.get_local_cost_map(point,search_max_x,search_min_x,search_max_y,search_min_y)
        max_area = -1
        max_row = -1
        rows = len(local_search_map)
        cols = np.shape(local_search_map)
        cols = cols[1]

        if rows < 1 or np.size(local_search_map) < 1:
            raise error("LocalMap is Empty")


        if int(rows/2) == 0:
            return [[],[]]
        for r in range(int(rows/2),rows):
            result = self.get_histogramArea(local_search_map[:r+1],rows)
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

        ul_corner_global = ul_corner_global[0]
        lr_corner_global = lr_corner_global[0]

        #if ul_corner_local[0]<=3:
            #ul_corner_global[1] = 1000
        #if ul_corner_local[1]<=3:
            #ul_corner_global[0] = -1000
        #if lr_corner_local[0] >= rows-3:
            #lr_corner_global[1] = -1000
        #if lr_corner_local[1] >=cols-3:
            #lr_corner_global = 1000

        return [ul_corner_global,lr_corner_global]

    def get_histogramArea(self,rows,total_n_rows):
        
        cols = len(rows[0])
        n_rows = len(rows)
        histogram = [0]*cols

        for row in rows:
            for col in range(cols):
                histogram[col] = 0 if not(row[col]==0) else(1 + histogram[col])

        return self.largestRectangleAreaHistogram(histogram,n_rows,total_n_rows)

    def largestRectangleAreaHistogram(self, heights,n_rows,total_n_rows):
        n = len(heights)
        p_horiz = int(n/2)
        p_vert = int(total_n_rows/2)
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
            if (area > max_area) and (left[i]<p_horiz and right[i]>p_horiz) and (n_rows>=p_vert and (n_rows-height)<(p_vert-2)):
                max_area = area
                max_height = height
                max_right = right[i]
                max_left = left[i]
                bar_index = i
        return [max_area,bar_index,max_height,max_right,max_left]

    def create_path_bounding_area(self,path):
        for i in range(len(path.localpath)):
            path_pt = path.localpath[i]
            safe_bounds = self.compute_bounding_boxes(path_pt,"p")
            self.path_boundary_table[i]=safe_bounds
            
        
    def quaternion_to_euler(self,x, y, z, w):
        t3 = 2*(w*z + x*y)
        t4 = 1-2*(y*y + z*z)
        angle = np.arctan2(t3,t4)
        return angle