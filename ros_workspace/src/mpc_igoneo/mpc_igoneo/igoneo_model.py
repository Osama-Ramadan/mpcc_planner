
from sys import path
import numpy as np

# Import CasADi 
path.append(r"/home/casadi-linux-py36-v3.5.5-64bit")
from casadi import *

class IGONEO():
    def __init__(self):
        self.f = 0
        self.define_model()

   
    def get_forward_function(self):
        return self.f
       
    def define_model(self):
        mB= 10                     # mass of the chassis
        self.d= 0.0                    # distance from the center of the wheels to the center of mass of chassis
        D= 1.827             
        IB= mB*D**2            # moment of inertia of the chassis
        IT= IB + mB*(self.d)**2 + 100
            
        x = SX.sym('x')
        y = SX.sym('y')
        theta = SX.sym('theta')
        gamma = SX.sym('gamma')
        x_dot = SX.sym('x_dot')
        y_dot = SX.sym('y_dot')
        theta_dot = SX.sym('theta_dot')
        gamma_dot = SX.sym('gamma_dot')
        w = SX.sym('w')

        
        tau = SX.sym('tau') 
        omega = SX.sym('omega')
        wd = SX.sym('wd')
            
        states = vertcat(x,y,theta,gamma,x_dot,y_dot,theta_dot,gamma_dot,w)
        q_dot =vertcat(x_dot, y_dot, theta_dot, gamma_dot)       # system states
        T = vertcat(tau*cos(gamma+theta), tau*sin(gamma+theta), -tau*sin(gamma)*(D-self.d), omega)                                               # system control inputs
        Input = vertcat(tau,omega,wd)



        M = SX.zeros(4,4)
        M[0,0] = mB
        M[0,2] = mB*self.d*sin(theta)
        M[1,1] = mB
        M[1,2] = -mB*self.d*cos(theta)
        M[2,0] = mB*self.d*sin(theta)
        M[2,1] = -mB*self.d*cos(theta)
        M[2,2] = IT
        M[3,3] = 2
        M_inv = inv(M)

        B = SX.zeros(4,1)
        B[0,0] = -cos(theta)*mB*self.d*(theta_dot*theta_dot)
        B[1,0] = -sin(theta)*mB*self.d*(theta_dot*theta_dot)

        C = SX.zeros(2,4)
        C[0,0] = sin(theta+gamma)
        C[0,1] = -cos(theta+gamma)
        C[0,2] = (D)*cos(gamma)
        C[0,3] = 0
        C[1,0] = sin(theta)
        C[1,1] = -cos(theta)
        C[1,2] = 0
        C[1,3] = 0


        Cdot = SX.zeros(2,4)
        Cdot[0,0] = cos(theta+gamma)*(theta_dot+gamma_dot)
        Cdot[0,1] = sin(theta+gamma)*(theta_dot+gamma_dot)
        Cdot[0,2] = -(D)*sin(gamma)*gamma_dot
        Cdot[0,3] = 0
        Cdot[1,0] = cos(theta)*theta_dot
        Cdot[1,1] = sin(theta)*theta_dot


        lambdas=-inv(C@M_inv@C.T)@(C@M_inv@(T-B)+Cdot@q_dot)
        q_dd= M_inv@(T-B+C.T@lambdas)
        q_dd = vertcat(q_dd,0,0,0,0,0)
        self.f = Function('f', [states,Input], [q_dd])
