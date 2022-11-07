clc;close;clear; clear global;

% parameters
global Lambdaa;


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
tspan=[0 10];

opts = odeset('MaxStep',1e-2);
% solution
[T,Z]=ode45(@DD_EOM_D,tspan,init_conds, opts);

plot(Z(:,1:2))