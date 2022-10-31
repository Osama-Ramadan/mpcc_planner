import numpy as np
class DYNAMIC_OBSTACLE():
    def __init__(self, target):

        self.target = target
        self.current_pose = np.zeros([2,1])
        self.current_estimated_state = np.zeros([4,1])
        self.current_estimated_cov = np.zeros([4,4])

    def update_target(self, target):
        self.target = target

    def update_estimated_state(self, est_pose):
        self.current_estimated_state = est_pose
    
    def update_estimated_cov(self, est_cov):
        self.current_estimated_cov = est_cov
    
    def update_pose(self, pose):
        self.current_pose = np.reshape(pose,[2,1])

    def get_pose(self):
        return np.reshape(self.current_pose,[2,1])

    def get_estimated_state(self):
        return np.reshape(self.current_estimated_state,[4,1])

    def get_estimated_cov(self):
        return np.reshape(self.current_estimated_cov,[4,4])

    def get_target_prim(self):
        return self.target
