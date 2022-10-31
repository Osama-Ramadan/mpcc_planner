clc; clear all; close all;
%% passenger car parameters
m = 1573; % [kg]
Iz = 2873; %[kg*m^2]
l = 2.68; %[m]
WF = 0.6; % front weight ratio
lR = WF*l; % distance from CG to rear axle
lF = l-lR; % distance from CG to front axle
Cf = 0.5*80000; %[N/rad] combinded conrnering stifness of front tires
Cr = Cf; %[N/rad] combinded conrnering stifness of rear tires

V = 20; % constant velocity [m/s]

Ybeta = -(Cr + Cf);
Yr = (Cr*lR-Cf*lF)/V;
Ydelta = Cf;
Nbeta = Cr*lR-Cf*lF;
Nr = -(Cf*lF^2+Cr*lR^2)/V;
Ndelta = Cf*lR;

A = [Ybeta/(m*V), Yr/(m*V)-1; ...
    Nbeta/Iz, Nr/Iz];

B = [Ydelta/(m*V); ...
    Ndelta/Iz];

beta0 = 0;  %initial sideslip angle
r0 = 0;     %initial yaw rate

states(:,1) = [beta0 ; r0];

dt = 0.01;
t = (0:dt:20)';
N = length(t);
T = 10;
omega = 2*(2*pi/T);
delta = 1*pi/180;


for k = 1:N-1
    dstates(:,k) = A*states(:,k)+B*delta; 
    states(:,k+1) = states(:,k) + dstates(:,k)*dt;

end

beta = states(1,:);
r = states(2,:);

figure
plot(t,beta, 'linewi',2), grid on
xlabel('t [s]'), ylabel('\beta [rad]')
hold on

%% Predicting heading angle and x, y positions given beta and r
psi(1) = 0;
x(1) = 0;
y(1) = 0;

for k = 1:N-1
    psi(k+1) = psi(k) + r(k)*dt;

    Vx(k) = V*cos(psi(k)+beta(k));
    Vy(k) = V*sin(psi(k)+beta(k));

    x(k+1) = x(k) + Vx(k)*dt;
    y(k+1) = y(k) + Vy(k)*dt;
end

figure 
plot(y,x,'LineWidth',2), grid on
xlabel('y position [m]'), ylabel('x position [m]')
axis equal

%% animate the vehicle
xF = x + lF*cos(psi);
yF = y + lF*sin(psi);
xR = x - lR*cos(psi);
yR = y - lR*sin(psi);

Rt = 0.5;
xF_F = xF + Rt*cos(psi+delta'); % x position front of front tire
yF_F = yF + Rt*sin(psi+delta'); % y position rear of front tire
xF_R = xF - Rt*cos(psi+delta'); % x position front of front tire
yF_R = yF - Rt*sin(psi+delta'); % y position rear of front tire

xR_F = xR + Rt*cos(psi); % x position front of rear tire
yR_F = yR + Rt*sin(psi); % y position rear of rear tire
xR_R = xR - Rt*cos(psi); % x position front of rear tire
yR_R = yR - Rt*sin(psi); % y position rear of rear tire

skip = 5;
for k = 1:skip:N

    figure(101)
    
    plot(y,x,'c-','LineWidth',2), grid on ; hold on;

    plot([yF_R(k) yF_F(k)],[xF_R(k) xF_F(k)], 'r-', 'LineWidth',8); % front
    plot([yR_R(k) yR_F(k)],[xR_R(k) xR_F(k)], 'r-', 'LineWidth',8); % rear
    plot(y(k), x(k), 'ko','markersize', 6, 'LineWidth',4); 
    plot([yR(k) yF(k)],[xR(k) xF(k)], 'k-', 'LineWidth',4); hold off;
    axis equal
    axis([y(k)-lF*5 y(k)+lF*5 x(k)-lF*5 x(k)+lF*5])
    drawnow
end