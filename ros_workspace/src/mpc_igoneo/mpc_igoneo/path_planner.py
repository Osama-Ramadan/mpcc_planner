import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from interfaces.srv import Interpolation 

class PATH_PLANNER(Node):
    def __init__(self):
        super().__init__("path_planner")

        group3 = MutuallyExclusiveCallbackGroup()

        self.srv = self.create_service(Interpolation, 'interpolation_service', self.interpolate)
        self.current_path = []
        self.current_waypoints = []
        self.Qx = []
        self.Qy = []

    def interpolate(self, request, response):     

        accuracy_parameter = request.accuracyparameter     # update interpolation accuracy paramter
        global_waypoints_lists = request.globalwaypoints.lists
        size = request.globalwaypoints.size
        global_waypoints = []
        for i in range(size):
            global_waypoints.append(global_waypoints_lists[i].elements)

        if self.current_waypoints == global_waypoints:              # if no change happened to the waypoint just return the current path
            response.localpath_x = list(self.current_path[:,0])
            response.localpath_y = list(self.current_path[:,1])
            response.localpath_theta = list(self.current_path[:,2])
            response.qx = self.Qx 
            response.qy = self.Qy
            return response

        self.current_waypoints = global_waypoints           # store the current set of waypoints

        x = np.zeros(len(global_waypoints))
        y = np.zeros(len(global_waypoints))
        for i in range(len(global_waypoints)):
            x[i] = global_waypoints[i][0]          # extract x values from global waypoints
            y[i] = global_waypoints[i][1]          # extract y values from global waypoints

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
        self.Qx = [float(i) for i in Qx]
        self.Qy = [float(i) for i in Qy]

        t = np.linspace(0,1,num=accuracy_parameter)
        local_path = np.zeros([(n-1)*len(t),3])
        for k in range(0,n-1):
            x_t=1.0/6.0*(((1-t)**3)*Qx[k]+(3*t**3-6*t**2+4)*Qx[k+1]+(-3*t**3+3*t**2+3*t+1)*Qx[k+2]+(t**3)*Qx[k+3])
            y_t=1.0/6.0*(((1-t)**3)*Qy[k]+(3*t**3-6*t**2+4)*Qy[k+1]+(-3*t**3+3*t**2+3*t+1)*Qy[k+2]+(t**3)*Qy[k+3]) 
            local_path[k*len(t):(k+1)*len(t),0] = x_t.astype(np.float)
            local_path[k*len(t):(k+1)*len(t),1] = y_t.astype(np.float)
        
        for k in range(np.size(local_path,0)-1):
            delta_y = local_path[k+1][1]- local_path[k][1]
            delta_x = local_path[k+1][0]- local_path[k][0]
            local_path[k][2] = np.arctan2(delta_y,delta_x)
        

            self.current_path = local_path                  # store the current calculated path

            response.localpath_x = list(local_path[:,0])
            response.localpath_y = list(local_path[:,1])
            response.localpath_theta = list(local_path[:,2])

        response.qx = self.Qx
        print(response.qx)
        response.qy = self.Qy
        
        return response

def main(args=None):
    rclpy.init(args=args)

    
    try:
        path_planning_service = PATH_PLANNER()
        executor1 = MultiThreadedExecutor(num_threads=2)
        executor1.add_node(path_planning_service)


        try:
            executor1.spin()
        finally:
            executor1.shutdown()
            path_planning_service.destroy_node()
    # Destroy the node explicitly  
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

