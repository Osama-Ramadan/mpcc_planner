import numpy as np

class KALMAN_FILTER():
    def __init__(self, dt, N):
        
        self.dt = dt
        self.N = N

        self.A = np.array([[1, 0, self.dt, 0],
                  [0, 1, 0, self.dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

        
        self.B = np.array([[0.5*dt**2],[ 0.5*dt**2], [dt], [dt]])
        self.C = np.array([[1, 0, 0, 0],[0, 1, 0, 0]])

        self.R_noise = 0.0001

        Q_pose = 10*0.5*dt**2
        Q_vel = 10*dt
        self.Q_noise = np.array([[Q_pose, 0, 0, 0],
                                 [0 , Q_pose, 0, 0],
                                 [0, 0, Q_vel, 0],
                                 [0, 0, 0, Q_vel]])

        self.u = 0

        
    def update_prediction_model(self, A, B, C):
        self.A_f = np.array(A)
        self.B_f = np.array(B)
        self.C_f = np.array(C)

    def update_noise_model(self, Q, R):
        self.R_noise = np.array(R)
        self.Q_noise = np.array(Q)

    def update_estimation(self, obstacle):
        obst_pose = obstacle.get_pose()
        [obstacle_old_estimated_state_f,obstacle_old_estimated_state_b] = obstacle.get_estimated_state()
        [obstacle_old_estimated_cov_f,obstacle_old_estimated_cov_b] = obstacle.get_estimated_cov()
        
        #dist_f = np.sqrt(np.sum((obst_pose[0] - obstacle_old_estimated_state_f[0:2]) ** 2, axis=0))
        #if dist_f > 5:
            #obstacle.clear_estimations()
        
        obst_pose = obstacle.get_pose()
        [obstacle_old_estimated_state_f,obstacle_old_estimated_state_b] = obstacle.get_estimated_state()
        [obstacle_old_estimated_cov_f,obstacle_old_estimated_cov_b] = obstacle.get_estimated_cov()
        
        if len(obstacle_old_estimated_state_f)==0 and len(obstacle_old_estimated_cov_f)==0:
            obstacle_old_estimated_state_f = np.zeros([4,1])
            obstacle_old_estimated_cov_f = np.zeros([4,4])
        else:
            obstacle_old_estimated_state_f = obstacle_old_estimated_state_f
            obstacle_old_estimated_cov_f = obstacle_old_estimated_cov_f

        if len(obstacle_old_estimated_state_b)==0 and len(obstacle_old_estimated_cov_b)==0:
            obstacle_old_estimated_state_b = np.zeros([4,1])
            obstacle_old_estimated_cov_b = np.zeros([4,4])
        else:
            obstacle_old_estimated_state_b = obstacle_old_estimated_state_b
            obstacle_old_estimated_cov_b = obstacle_old_estimated_cov_b

        # Predictions for forward Point
        old_estimated_state_f = self.A@obstacle_old_estimated_state_f + self.B*self.u
        old_estimated_cov_f = (self.A@obstacle_old_estimated_cov_f)@(self.A.T)+self.Q_noise

        # Predictions for backward Point
        old_estimated_state_b = self.A@obstacle_old_estimated_state_b + self.B*self.u
        old_estimated_cov_b = (self.A@obstacle_old_estimated_cov_b)@(self.A.T)+self.Q_noise

        # Corrections for forward Point
        [forward_pose,backward_pose] = obstacle.get_pose()
        pose_residual_f = forward_pose - self.C@old_estimated_state_f
        S = (self.C@old_estimated_cov_f)@self.C.T + self.R_noise
        K = (old_estimated_cov_f@self.C.T)@np.linalg.inv(S)

        # New estimations for forward point
        new_estimated_state_f = old_estimated_state_f + K@pose_residual_f
        new_estimated_cov_f = (np.identity(4)-K@self.C)@old_estimated_cov_f

        # Corrections for backward Point
        pose_residual_b = backward_pose - self.C@old_estimated_state_b
        S = (self.C@old_estimated_cov_b)@self.C.T + self.R_noise
        K = (old_estimated_cov_b@self.C.T)@np.linalg.inv(S)

        # New estimations for backward point
        new_estimated_state_b = old_estimated_state_b + K@pose_residual_b
        new_estimated_cov_b = (np.identity(4)-K@self.C)@old_estimated_cov_b

        return [new_estimated_state_f,new_estimated_state_b, new_estimated_cov_f,new_estimated_cov_b]

    def predict_trajectory(self,state):

        F = np.array([[1, 0, self.dt*10, 0],
                      [0, 1, 0, self.dt*10],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        
        current_state = state
        predicted_trajectory = []
        
        for i in range(self.N):
            next_state = F@current_state
            predicted_trajectory.append(next_state)
            current_state = next_state
        return np.array(predicted_trajectory).reshape(self.N,4)
        


    