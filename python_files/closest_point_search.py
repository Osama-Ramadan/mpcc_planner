
import numpy as np
import time
from scipy import spatial

y_array = np.random.random(100).reshape(10,10)
x_array = np.random.random(100).reshape(10,10)


points = np.array([5,5])

combined_x_y_arrays = np.dstack([y_array.ravel(),x_array.ravel()])[0]

print(combined_x_y_arrays)

def find_nearist_point(combined_x_y_arrays,points):
    mytree = spatial.cKDTree(combined_x_y_arrays)
    dist, indexes = mytree.query(points)
    return indexes

start = time.time()
results2 = do_kdtree(combined_x_y_arrays,points)
end = time.time()
print(results2)
print ('Completed in: ',end-start)