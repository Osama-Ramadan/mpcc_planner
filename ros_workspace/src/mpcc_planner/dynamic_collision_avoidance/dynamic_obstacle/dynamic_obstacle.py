import numpy as np

class DYNAMIC_OBSTACLE():
    def __init__(self, target1,target2,id):

        self.r = 0.5

        self.id = id
        
        self.target_f = target1
        self.target_b = target2

        self.current_pose_f = np.zeros([2,1])
        self.current_estimated_state_f = np.zeros([4,1])
        self.current_estimated_cov_f = np.zeros([4,4])

        self.current_pose_b = np.zeros([2,1])
        self.current_estimated_state_b = np.zeros([4,1])
        self.current_estimated_cov_b = np.zeros([4,4])
        
    def clear_estimations(self):
        self.current_estimated_state_f = np.zeros([4,1])
        self.current_estimated_cov_f = np.zeros([4,4])
        
        self.current_estimated_state_b = np.zeros([4,1])
        self.current_estimated_cov_b = np.zeros([4,4])

    def update_target(self, target1, target2):
        self.target_f = target1
        self.target_b = target2

    def update_estimated_state(self, est_pose1, est_pose2):
        self.current_estimated_state_f = est_pose1
        self.current_estimated_state_b = est_pose2

    def update_estimated_cov(self, est_cov1, est_cov2):
        self.current_estimated_cov_f = est_cov1
        self.current_estimated_cov_b = est_cov2

    
    def update_pose(self, pose1, pose2):
        self.current_pose_f = np.reshape(pose1,[2,1])
        self.current_pose_b = np.reshape(pose2,[2,1])

    def get_pose(self):
        return [np.reshape(self.current_pose_f,[2,1]),np.reshape(self.current_pose_b,[2,1])]

    def get_estimated_state(self):
        return [np.reshape(self.current_estimated_state_f,[4,1]),np.reshape(self.current_estimated_state_b,[4,1])]

    def get_estimated_cov(self):
        return [np.reshape(self.current_estimated_cov_f,[4,4]),np.reshape(self.current_estimated_cov_b,[4,4])]

    def get_target_prim(self):
        return [self.target_f,self.target_b]
    
    def get_id(self):
        return self.id
