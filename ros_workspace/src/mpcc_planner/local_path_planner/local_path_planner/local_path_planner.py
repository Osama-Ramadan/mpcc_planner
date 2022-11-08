from threading import local
import time
import rclpy
from rclpy.node import Node
from mpcc_interfaces.msg import LocalPath
from mpcc_interfaces.srv import MapRequest
from mpcc_planner_costmap.mpcc_planner_costmap import MPCC_PlannerCostMap
from geometry_msgs.msg import PoseArray
import numpy as np
from sys import path
import pandas as pd 
import random
 #Import CasADi 
 
path.append(r"/home/ros_workspace/casadi-linux-py36-v3.5.5-64bit")
from casadi import *


class LOCAL_PATH_PLANNER(Node):
    
    def __init__(self):
        super().__init__("local_path_planner")

        #self.get_logger().info('Iam the path planner node and I am working')

        ## Create Publishers and Subscribers ##
        self.local_path_publisher_ =  self.create_publisher(LocalPath, "local_path", 1)                                  # Publish The LocalPath to the mpcc_controller 

        ## Create Client to get the Map ##
        self.req_map_cli = self.create_client(MapRequest, 'global_map_request')
        while not self.req_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...') 


        ## Get the Map and Create the Map Object ##
        result = None
        while result == None:
            req = MapRequest.Request()
            future = self.req_map_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()

        self.costmap = MPCC_PlannerCostMap(result.costmap)
        ## Create Local Path Object
        self.path_sampling_para = 100
        #self.originalwaypoints = [[3,18],[7.5,11.8]]
        #self.originalwaypoints = [[3,18],[10,10],[18,15]]
        self.originalwaypoints = [[17,10.5],[9,9.5],[17,15],[3,18]]

        self.lookahead = 3

        
        # Local Path Message
        self.local_path= self.generate_path(self.path_sampling_para,self.originalwaypoints,self.lookahead)
        self.local_path_msg = self.update_local_path_msg(self.local_path)
        
        print("\n")
        print("Original Path Length")
        self.calc_path_length(self.local_path_msg)
        
        typef = "Orig"
        self.log_local_path(self.local_path_msg,typef,0)
        
        st = time.time()
        self.local_path = self.check_path(self.local_path, self.costmap)   
        et = time.time()
        elapsed_time = et - st   
        print("\n")
        print("Excution Time: " + str(elapsed_time)) 
        
        self.local_path_msg = self.update_local_path_msg(self.local_path)
        
        print("\n")
        print("New Path Length")
        self.calc_path_length(self.local_path_msg)
        
        typef = "Final"
        self.log_local_path(self.local_path_msg,typef,0)
        print("-----------------------------")
        print("I am finished") 
        
        #self.path_planner_timer = self.create_timer(1, self.local_path_callback)
        
        ## Log Path ##
    
    def calc_path_length(self,local_path):
        x_pts = local_path.sampled_pt_x
        y_pts = local_path.sampled_pt_y
        points = np.dstack([x_pts,y_pts])[0]
        lengths = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
        print("Length " + str(np.sum(lengths)))
    
    
      
    def generate_random_point(self,Areas):
        points = []
        for area in Areas:
            xmin = area[0]
            xmax = area[1]
            ymin = area[2]
            ymax = area[3]
            x_p = round(random.uniform(xmin,xmax),2)
            y_p = round(random.uniform(ymin,ymax),2)
            obst_list = self.costmap.get_obst_around_pose([x_p,y_p],0.8,0.8,0.8,0.8)
            if len(obst_list)==0:
                points.append([x_p,y_p])
        return points
    def log_local_path(self,local_path_msg,type,num):
        path_pts = np.dstack([local_path_msg.sampled_pt_x,local_path_msg.sampled_pt_y,local_path_msg.sampled_pt_th])
        control_pts = np.dstack([local_path_msg.qx, local_path_msg.qy])
        waypoints = np.dstack([local_path_msg.waypoints_x,local_path_msg.waypoints_y])
        orig_waypts = np.array(self.originalwaypoints)
        if type == "Orig":
            path = "/home/ros_workspace/local_path_log/path_pts_orig"+str(num)+".csv"
            pd.DataFrame(path_pts[0]).to_csv(path)
            path = "/home/ros_workspace/local_path_log/control_pts_orig"+str(num)+".csv"
            pd.DataFrame(control_pts[0]).to_csv(path)    
            path = "/home/ros_workspace/local_path_log/waypoints_orig"+str(num)+".csv"
            pd.DataFrame(waypoints[0]).to_csv(path) 
        else:
            path = "/home/ros_workspace/local_path_log/path_pts"+str(num)+".csv"
            pd.DataFrame(path_pts[0]).to_csv(path)
            path = "/home/ros_workspace/local_path_log/control_pts"+str(num)+".csv"
            pd.DataFrame(control_pts[0]).to_csv(path)    
            path = "/home/ros_workspace/local_path_log/waypoints"+str(num)+".csv"
            pd.DataFrame(waypoints[0]).to_csv(path) 

    def local_path_callback(self):
        self.local_path_publisher_.publish(self.local_path_msg)
    
    def check_path(self, local_path, cost_map):
        start_seg = 1
        while(start_seg <= len(local_path.segments)):
            seg = local_path.segments[start_seg-1]
            seg_check_result = self.check_seg(seg,cost_map)
            if seg_check_result[0] == False:                        # Segment is not collision Free
                collision_point = seg_check_result[1]               # The point that collide on the segment
                collision_path_th = seg_check_result[2]
                
                [updated_seg,result,Found] = self.update_seg(seg,cost_map,collision_point,collision_path_th)
                if Found == False:
                    self.get_logger().error("Could not Updated the Path, The current Path Either Collid or Very Close to Obstacles")
                    if result == 1:
                        new_wps = []
                        for j in range(len(local_path.segments)):
                            if j!=start_seg-1:
                                new_wps.append((local_path.segments[j]).waypoints[0])
                            else:
                                new_wps.append(updated_seg.waypoints[0])
                                new_wps.append(updated_seg.waypoints[1])
                        new_wps.append(local_path.segments[-1].waypoints[1])
                        local_path = self.generate_path(self.path_sampling_para,new_wps,self.lookahead)
                    else:
                        new_wps = []
                        for j in range(len(local_path.segments)):
                            if j!=start_seg-1:
                                new_wps.append(local_path.segments[j].waypoints[0])
                            else:
                                new_wps.append(updated_seg.waypoints[0])
                        new_wps.append(local_path.segments[-1].waypoints[1])
                        local_path = self.generate_path(self.path_sampling_para,new_wps,self.lookahead)
                    return local_path
                
                elif Found == True and result == 1:
                    new_wps = []
                    for j in range(len(local_path.segments)):
                        if j!=start_seg-1:
                            new_wps.append((local_path.segments[j]).waypoints[0])
                        else:
                            new_wps.append(updated_seg.waypoints[0])
                            new_wps.append(updated_seg.waypoints[1])
                    new_wps.append(local_path.segments[-1].waypoints[1])
                    print(new_wps)
                    local_path = self.generate_path(self.path_sampling_para,new_wps,self.lookahead)
                elif Found == True and result == 2:
                    new_wps = []
                    for j in range(len(local_path.segments)):
                        if j!=start_seg-1:
                            new_wps.append(local_path.segments[j].waypoints[0])
                        else:
                            new_wps.append(updated_seg.waypoints[0])
                    new_wps.append(local_path.segments[-1].waypoints[1])
                    print(new_wps)
                    local_path = self.generate_path(self.path_sampling_para,new_wps,self.lookahead)
            else:
                start_seg = start_seg+1
            
                    
        return local_path
                           
    def check_seg(self,seg,map):
        # Obtain corresponding Control points
        sampled_x = seg.seg_x
        sampled_y = seg.seg_y

        for t in range(len(sampled_x)):
            path_point = [sampled_x[t],sampled_y[t]]
            obst_around_point = map.get_obst_around_pose(path_point,0.5,0.5,0.5,0.5)    # Check if a square of 1m search area around the path point has obstacles
            if len(obst_around_point)!=0:
                return [False,path_point,t]                                             # if yes return the path parameter value and the point
        return [True,0,t]                                                               # if free return true

    def generate_path(self, path_sample_para, orig_waypoints, lookahead):
        
        path = self.Path()
        Local_Path = self.Path()
        
        path_finished = False
        
        increment = max(lookahead-1,1)
        waypoints = orig_waypoints
        
        for i in range(0,len(orig_waypoints),increment):
            if path_finished:
                break
            
            if i+lookahead+1 >= len(orig_waypoints):      # The waypoints are covered with the given lookahead
                path_finished = True
               
            waypoints = orig_waypoints[i:min(i+lookahead+1,len(orig_waypoints))]
        
            x = np.zeros(len(waypoints))
            y = np.zeros(len(waypoints))
            for i in range(len(waypoints)):
                x[i] = waypoints[i][0]          # extract x values from  waypoints
                y[i] = waypoints[i][1]          # extract y values from  waypoints

            Px=np.concatenate(([0],x,[0]))
            Py=np.concatenate(([0],y,[0]))

            # Interpolation equations
            n=len(x)
            phi=np.zeros((n+2,n+2))
            for i in range(n):
                phi[i+1,i]=1
                phi[i+1,i+1]=4
                phi[i+1,i+2]=1

            # end condition constraints (end to end tangent)
            phi[0,0]=1
            phi[0,1]=-2
            phi[0,2]=1
            phi[n+1,n-1]=1
            phi[n+1,n]=-2
            phi[n+1,n+1]=1

            # Control points
            Qx=6*np.linalg.inv(phi).dot(Px)
            Qy=6*np.linalg.inv(phi).dot(Py)

            # Generate sample points
            t = np.linspace(0,1,num=path_sample_para)

            local_path = np.zeros([(n-1)*len(t),3])

            for k in range(0,n-1):
                x_t=1.0/6.0*(((1-t)**3)*Qx[k]+(3*t**3-6*t**2+4)*Qx[k+1]+(-3*t**3+3*t**2+3*t+1)*Qx[k+2]+(t**3)*Qx[k+3])
                y_t=1.0/6.0*(((1-t)**3)*Qy[k]+(3*t**3-6*t**2+4)*Qy[k+1]+(-3*t**3+3*t**2+3*t+1)*Qy[k+2]+(t**3)*Qy[k+3]) 
                local_path[k*len(t):(k+1)*len(t),0] = x_t.astype(np.float)
                local_path[k*len(t):(k+1)*len(t),1] = y_t.astype(np.float)

            for k in range(np.size(local_path,0)-1):
                delta_y = local_path[k+1,1]- local_path[k,1]
                delta_x = local_path[k+1,0]- local_path[k,0]
                local_path[k,2] = np.arctan2(delta_y,delta_x)
                
            for k in range(len(waypoints)-1):
                seg = self.segment()
                seg.Qx = Qx[k:k+4]
                seg.Qy = Qy[k:k+4]
                seg.waypoints = [waypoints[k],waypoints[k+1]]
                seg.seg_x = local_path[k*path_sample_para:(k+1)*path_sample_para,0]
                seg.seg_y = local_path[k*path_sample_para:(k+1)*path_sample_para,1]
                seg.seg_th = local_path[k*path_sample_para:(k+1)*path_sample_para,2]
                path.segments.append(seg)
        
        if len(path.segments)==0:
            return Local_Path
        
        for i in range(len(path.segments)-1):
            seg1 = path.segments[i]
            if seg1.waypoints != -1:
                seg2 = path.segments[i+1]
                if seg1.waypoints[0]==seg2.waypoints[0] and seg1.waypoints[1]==seg2.waypoints[1]:
                    Local_Path.segments.append(seg1)
                    path.segments[i+1].waypoints = -1
                else:
                    Local_Path.segments.append(seg1)
        Local_Path.segments.append(path.segments[-1])
        return Local_Path

    def get_updated_seg(self,p,seg):
        updated_seg = self.segment()
        first_waypoint = list(seg.waypoints[0])
        dist_to_fp = np.sqrt((p[0]-first_waypoint[0])**2+(p[1]-first_waypoint[1])**2)
        waypoints = list(seg.waypoints)
        if dist_to_fp > 3:
            waypoints.insert(1,p)
            result = 1                                  # Add the point to the list of waypoints
        else:
            if seg.waypoints[0] in self.originalwaypoints:
                waypoints.insert(1,p)
                result = 1                              # Add the point to the list of waypoints
            else:
                waypoints[0] = list(p)                        # Replace first waypoint with new point
                result = 2
        updated_seg.waypoints = waypoints
        return [updated_seg,result]
    
    def update_seg(self,seg, map, collision_point, t):
        
        # Update Algorithm Parameters
        step_size = 0.0005
        obst_force_gain = 0.5
        goal_force_gain = 0.0

        Qx = seg.Qx
        Qy = seg.Qy
        
        t = t*0.01
        p = collision_point
        best_p = p
              
        search_finished = False
        counter = 0
        MAX_COUNT = 100
        force_mag_buffer = 1000000
        
        while(not search_finished and counter < MAX_COUNT):

            # Get the obstacles around the collision point
            obst_list = map.get_obst_around_pose(p,0.9,0.9,0.9,0.9)
            if(len(obst_list)==0):
                search_finished = True 
                best_p = list(p) 
                break
            
            [force_val,d_force_val,force_mag] = self.get_obstacle_forces(obst_list,p,obst_force_gain)
            if force_mag > force_mag_buffer:
                counter = counter+1
                #force_mag_buffer = force_mag
            else:
                best_p = list(p)
                force_mag_buffer = force_mag

            # Get the tangent direction of the path at the collision point
            d_s = self.get_tang(Qx,Qy,t)
            d_s = d_s/(np.sqrt(np.sum((np.array(d_s)) ** 2, axis=0)))

            # Formulate the constraint equation
            M = [[d_force_val[0],       0,       -d_s[0]],
                 [0,       d_force_val[3],       -d_s[1]],
                 [d_s[0]   ,                d_s[1] ,          0]]
            
            F = [[-force_val[0]], [-force_val[0]], [0]]

            delta = np.linalg.inv(M)@F
            
            delta = np.array(delta[0:2],dtype=float)
            #d_p = (delta)/(np.sqrt(delta[0]**2+delta[1]**2))

            delta_x = delta[0]*step_size
            delta_y = delta[1]*step_size

            if(np.abs(delta_x) <= 0.0001 and np.abs(delta_y)<=0.0001):
                search_finished = False
                best_p = list(p) 
                break
            p[0] = round(float(p[0] + delta_x),4)
            p[1] = round(float(p[1] + delta_y),4)
            
        
        if search_finished:
            [seg,result] = self.get_updated_seg(best_p,seg)
            return [seg,result,True]
        else:                                               # Search Failed (Manully test the search directions)
            #self.get_logger().info("First Search Attempt Failed, Trying Other Search Directions")
            d_s = self.get_tang(Qx,Qy,t)
            d_s = d_s/(np.sqrt(np.sum((np.array(d_s)) ** 2, axis=0)))   
            
            search_directions = [[-d_s[1],d_s[0]],[d_s[1],-d_s[0]]]
            #best_p = collision_point
            min_f_p = list(best_p)
            min_force_mag = force_mag_buffer
            step_size = 0.0002
            
            for direction in search_directions:
                counter = 0
                MAX_COUNT = 100
                p = list(collision_point)
                force_mag_buffer = 10000
                search_finished = False
                
                while(not search_finished and counter < MAX_COUNT):
                    
                    obst_list = map.get_obst_around_pose(p,0.9,0.9,0.9,0.9)
                    if(len(obst_list)==0):
                        search_finished = True
                        best_p = list(p) 
                        break
                    
                    [force_val,d_force_val,force_mag] = self.get_obstacle_forces(obst_list,p,obst_force_gain)
                    
                    delta_x = force_mag*direction[0]*step_size
                    delta_y = force_mag*direction[1]*step_size
                    
                    if(np.abs(delta_x) <= 0.0001 and np.abs(delta_y)<=0.0001):
                        search_finished = True
                        best_p = list(p)
                        break
                    
                    p[0] = round(float(p[0] + delta_x),4)
                    p[1] = round(float(p[1] + delta_y),4)
                    if force_mag > force_mag_buffer:
                        counter = counter+1
                        #force_mag_buffer = force_mag
                    else:
                        best_p = list(p)
                        force_mag_buffer = force_mag
                        if force_mag < min_force_mag:
                            min_force_mag = force_mag
                            min_f_p = list(p)
                
                if search_finished:
                    [seg,result] = self.get_updated_seg(best_p,seg)
                    print("-------------------------------")   
                    return [seg,result,True]
                
        [seg,result] = self.get_updated_seg(min_f_p,seg)  
        print("-------------------------------")      
        return [seg,result,False]                   # All Trials Failed
                    
    def get_obstacle_forces(self,obst_list,point,obst_force_gain):
        
        p_xy = SX.sym('p_xy',2,1)
        p_x = p_xy[0]
        p_y = p_xy[1]
        
        potential = 0
        
        # Calculate Obstacles Potential
        for i in range(len(obst_list)):
            obst_pose = obst_list[i,:]
            dist_obst = np.sqrt(np.sum(([p_x,p_y] - obst_pose) ** 2, axis=0))
            potential = potential + (obst_force_gain/(dist_obst+0.000001))
        
        # Get Forces from Potential
        force = -1*gradient(potential,p_xy)
        d_force = jacobian(force,p_xy)
        force_f = Function('force', [p_xy],[force],['p_xy'],['force'])
        d_force_f = Function('d_force', [p_xy],[d_force],['p_xy'],['d_force'])

        # Get Forces Values
        force_val = np.round((force_f(point)).full(),3)
        force_mag = np.round(np.sqrt(np.sum((force_val) ** 2, axis=0)),3)
        d_force_val = np.round((d_force_f(point)).full().flatten(),3)
        
        return [force_val,d_force_val,force_mag]
    # Get the tangent of the path at a given point
    def get_tang(self,Qx,Qy,t):
        dx_t = (1/6)*((-3*(1-t)**2*Qx[0])+((9*t**2-12*t)*Qx[1])+((-9*t**2+6*t+3)*Qx[2])+(3*t**2*Qx[3]))
        dy_t = (1/6)*((-3*(1-t)**2*Qy[0])+((9*t**2-12*t)*Qy[1])+((-9*t**2+6*t+3)*Qy[2])+(3*t**2*Qy[3]))
        ds_t = [dx_t,dy_t]
        return ds_t

    def update_local_path_msg(self,local_path):
        
        local_path_msg = LocalPath()
        sampled_pt_x  = []
        sampled_pt_y  = []
        sampled_pt_th = []
        qx = []
        qy = []
        local_path_msg.resolution = self.path_sampling_para
        local_path_msg.nseg = len(local_path.segments)
        
        for seg in local_path.segments:
            local_path_msg.waypoints_x.append(float(seg.waypoints[0][0]))
            local_path_msg.waypoints_y.append(float(seg.waypoints[0][1]))
            sampled_pt_x.append(list(seg.seg_x))
            sampled_pt_y.append(list(seg.seg_y))
            sampled_pt_th.append(list(seg.seg_th))
            qx.append(seg.Qx)
            qy.append(seg.Qy)
        
        local_path_msg.sampled_pt_x = list(np.array(sampled_pt_x).flatten())
        local_path_msg.sampled_pt_y = list(np.array(sampled_pt_y).flatten())
        local_path_msg.sampled_pt_th = list(np.array(sampled_pt_th).flatten())
        local_path_msg.qx = list(np.array(qx).flatten())
        local_path_msg.qy = list(np.array(qy).flatten())
        
        local_path_msg.waypoints_x.append(float(seg.waypoints[-1][0]))
        local_path_msg.waypoints_y.append(float(seg.waypoints[-1][1]))
        
        return local_path_msg

    class Path:
        def __init__(self):
            self.segments = []

    class Local_Path_msg:
        waypoints_x = []
        waypoints_y = []
        Qx = []
        Qy = []
        sampled_x_pt  = []
        sampled_y_pt  = []
        sampled_th_pt = []
        nseg = 0
        resolution = 0
        
    class segment:
        def __init__(self) -> None:
            self.seg_x = []
            self.seg_y = []
            self.seg_th = []
            self.waypoints = []
            self.Qx = []
            self.Qy = []
        def updated_seg(self,seg_x,seg_y,seg_th,waypoints,Qx,Qy):
            self.seg_x  = list(seg_x)
            self.seg_y  = list(seg_y)
            self.seg_th = list(seg_th)
            self.waypoints = list(waypoints)
            self.Qx = list(Qx)
            self.Qy = list(Qy)
