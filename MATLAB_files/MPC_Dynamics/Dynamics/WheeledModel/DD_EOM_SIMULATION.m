clc;close;clear;clear global

%q_dd = [0;0;0;0;0];
% initial conditions
x0=0;
xdot0=0;
y0=0;
ydot0=0;
theta0=0;
thetadot0=0;
gamma0=90*(pi/180);
gammadot0=0;
init_conds=[x0;y0;theta0;gamma0;xdot0;ydot0;thetadot0;gammadot0];

% time span
tspan=[0 50];

% solution
opts = odeset('MaxStep',1e-2);
[T,Z]=ode45(@ODE_fork_red,tspan,init_conds,opts);

xs = [0;0;0];
x = Z(:,8);
t = 1:1:length(Z);
plot(Z(:,1),Z(:,2))
%Draw_MPC_PS_Obstacles(x,xs,1);