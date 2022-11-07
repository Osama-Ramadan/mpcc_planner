clc;close;clear;

% parameters
global mT mx d IT Iyy ro T1 D;
mw=0.2; % mass of wheels
mB=100; % mass of the chassis
d=1; % distance from the center of the wheels to the center of mass of chassis
D = 1.8;
IB=0.5*mB*(D/2)^2; % moment of inertia of the chassis
ro=0.127; % radius of the wheels
t=0.01; % thickness of the wheels
mT=mB+2*mw; % total mass
mx=mB*d+mw*D;
Iyy=mw*(ro^2)/2;
IT=IB+mB*d^2+2*Iyy;


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

% input torques
T1=100;

% solution
opts = odeset('MaxStep',1e-3);
[T,Z]=ode45(@DD_EOM,tspan,init_conds,opts);
xs = [0;0;0];
x = Z(:,2);
t = 1:1:length(Z);
plot(x)
%Draw_MPC_PS_Obstacles(x,xs,1);