import matplotlib.pyplot as plt
import numpy as np
 
global_waypoints = [[0,0], [1,2], [2,4]]

look_ahead = 3                 # update the look ahead paramter
accuracy_paramter = 10     # update interpolation accuracy paramter

x = np.zeros(look_ahead)
y = np.zeros(look_ahead)
for i in range(look_ahead):
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
phi[0,0]=-3
phi[0,2]=3
phi[0,n-1]=3
phi[0,n+1]=-3
phi[n+1,0]=6
phi[n+1,1]=-12
phi[n+1,2]=6
phi[n+1,n-1]=-6
phi[n+1,n]=12
phi[n+1,n+1]=-6

# Control points
Qx=6*np.linalg.inv(phi).dot(Px)
Qy=6*np.linalg.inv(phi).dot(Py)
plt.figure(figsize=(12, 5))

t = np.linspace(0,1,num=accuracy_paramter)
local_path = np.zeros([(n-1)*len(t),2])
for k in range(0,n-1):
    x_t=1.0/6.0*(((1-t)**3)*Qx[k]+(3*t**3-6*t**2+4)*Qx[k+1]+(-3*t**3+3*t**2+3*t+1)*Qx[k+2]+(t**3)*Qx[k+3])
    y_t=1.0/6.0*(((1-t)**3)*Qy[k]+(3*t**3-6*t**2+4)*Qy[k+1]+(-3*t**3+3*t**2+3*t+1)*Qy[k+2]+(t**3)*Qy[k+3])  
    plt.plot(x_t,y_t,'k',linewidth=2.0,color='orange')
    local_path[k*len(t):(k+1)*len(t),0] = x_t
    local_path[k*len(t):(k+1)*len(t),1] = y_t
print(local_path)
plt.plot(x, y, 'ko', label='fit knots',markersize=15.0)

plt.plot(Qx, Qy, 'o--', label='control points',markersize=15.0)
plt.xlabel('x')
plt.ylabel('y')
plt.legend(loc='upper left', ncol=2)
plt.savefig('cubic_spline_end-to-end_tangent_continuity.png')
plt.show()