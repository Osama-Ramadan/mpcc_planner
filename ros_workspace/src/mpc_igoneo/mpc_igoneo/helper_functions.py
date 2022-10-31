from scipy import spatial

def find_closest_point(list, point):
        mytree = spatial.cKDTree(list)
        dist, indexes = mytree.query(point)
        return indexes

def get_next_waypoints(lookahead, goals, current_path, current_waypoints, current_pose):
        if len(current_path)==0 and len(current_waypoints)==0:      # first time to call no path or waypoint calculated before
            return goals[0:lookahead+1]
        waypoint_index = find_closest_point(current_path,[current_waypoints[1],current_pose])
        if (waypoint_index[0]-10) > waypoint_index[1]:         # vehciel did not complete the first segmetn of the path yet
            return current_waypoints
        else:
            current_waypoint_index_in_goal = goals.index(current_waypoints[0])  # vehicle already finished the firts segment
            if current_waypoint_index_in_goal == (len(goals)-lookahead-1):        # if the current path reaches the final goal point
                return current_waypoints
            else:
                current_waypoints = goals[current_waypoint_index_in_goal+1:current_waypoint_index_in_goal+lookahead+2]
                return current_waypoints