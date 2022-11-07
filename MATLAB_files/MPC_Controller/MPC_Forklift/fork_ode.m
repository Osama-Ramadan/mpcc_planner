function dxdt = adip_ode(x, u)
% Fork_ODE Implementation of the ODE for the Forklift
%   This function implements the ODE for the Forklift. This function can also be used to define the

L = 1;

x_dot = cos(x(4))*cos(x(3))*u(1);
y_dot = cos(x(4))*sin(x(3))*u(1);
theta_dot = sin(x(4))/L*u(1);
phi_dot = u(2);

dxdt = [x_dot;y_dot;theta_dot;phi_dot];
end