clc;close;clear;clear global

%q_dd = [0;0;0;0;0];
% initial conditions
x0=0;
xdot0=0;
y0=0;
ydot0=0;
theta0=0;
thetadot0=0;
phi10=0;
phi1dot0=0;
phi20=0;
phi2dot0=0;
init_conds=[x0;xdot0;y0;ydot0;theta0;thetadot0;phi10;phi1dot0;phi20;phi2dot0];

% time span
tspan=[0 100];

% solution
opts = odeset('MaxStep',1e-2);
[T,Z]=ode45(@DD_EOM,tspan,init_conds,opts);

xs = [0;0;0];
x = Z(:,8);
t = 1:1:length(Z);
phi_diff = Z(:,8)- Z(:,10);
plot(T,Z(:,2))
%Draw_MPC_PS_Obstacles(x,xs,1);