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
                  [0, 0, Q_vel, Q_vel],
                  [0, 0, 0, Q_vel]])

        self.u = 0

    def update_prediction_model(self, A, B, C):
        self.A = np.array(A)
        self.B = np.array(B)
        self.C = np.array(C)

    def update_noise_model(self, Q, R):
        self.R_noise = np.array(R)
        self.Q_noise = np.array(Q)

    def update_estimation(self, obstacle):
        obstacle_old_estimated_state = obstacle.get_estimated_state()
        obstacle_old_estimated_cov = obstacle.get_estimated_cov()

        if len(obstacle_old_estimated_state)==0 and len(obstacle_old_estimated_cov)==0:
            obstacle_old_estimated_state = np.zeros([4,1])
            obstacle_old_estimated_cov = np.zeros([4,4])
        else:
            obstacle_old_estimated_state = obstacle_old_estimated_state
            obstacle_old_estimated_cov = obstacle_old_estimated_cov

        # Predictions
        old_estimated_state = self.A@obstacle_old_estimated_state + self.B*self.u
        old_estimated_cov = (self.A@obstacle_old_estimated_cov)@(self.A.T)+self.Q_noise

        # Corrections

        pose_residual = obstacle.get_pose() - self.C@old_estimated_state
        S = (self.C@old_estimated_cov)@self.C.T + self.R_noise
        K = (old_estimated_cov@self.C.T)@np.linalg.inv(S)

        # New estimations
        new_estimated_state = old_estimated_state + K@pose_residual
        new_estimated_cov = (np.identity(4)-K@self.C)@old_estimated_cov

        return new_estimated_state, new_estimated_cov

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
        return predicted_trajectory
        


    