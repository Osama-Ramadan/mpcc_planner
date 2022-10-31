clc; clear;close all;
%% kinematics bicycle Model
l = 2.68; %[m]
WF = 0.4; % front weight ratio
lR = WF*l; % distance from CG to rear axle
lF = l-lR; % distance from CG to front axle


V = 20; % constant velocity [m/s]
x0 = [0;0;0;V];
states(:,1) = x0;
beta(1) = 0;

a = 0.0;
gamma = 1*pi/180;
inputs = [a ; gamma];

dt = 0.01;
t = (0:dt:10)';
N = length(t);

for k = 1:N-1
    theta = states(3,k);
    A = [cos(theta+beta(k)) ; sin(theta+beta(k)); sin(beta(k))/lF ; 0];
    B = [0; 0; 0; 1];
    dstates(:,k) = A*states(4,k)+B*inputs(1); 
    states(:,k+1) = states(:,k) + dstates(:,k)*dt;
    beta(k+1)= atan((lF/lF+lR)*sin(gamma));

end

x = states(1,:);
y = states(2,:);

figure
plot(t,beta, 'linewi',2), grid on
xlabel('t [s]'), ylabel('\beta [rad]')
hold on

%% Predicting heading angle and x, y positions given beta and r
theta = states(3,:);
x = states(1,:);
y = states(2,:);



figure 
plot(y,x,'LineWidth',2), grid on
xlabel('y position [m]'), ylabel('x position [m]')
axis equal

%% animate the vehicle
xF = x + lF*cos(theta);
yF = y + lF*sin(theta);
xR = x - lR*cos(theta);
yR = y - lR*sin(theta);

Rt = 0.5;
xF_F = xF + Rt*cos(theta); % x position front of front tire
yF_F = yF + Rt*sin(theta); % y position rear of front tire
xF_R = xF - Rt*cos(theta); % x position front of front tire
yF_R = yF - Rt*sin(theta); % y position rear of front tire

xR_F = xR + Rt*cos(theta-gamma); % x position front of rear tire
yR_F = yR + Rt*sin(theta-gamma); % y position rear of rear tire
xR_R = xR - Rt*cos(theta-gamma); % x position front of rear tire
yR_R = yR - Rt*sin(theta-gamma); % y position rear of rear tire

skip = 5;
for k = 1:skip:N

    figure(102)
    
    plot(y,x,'c-','LineWidth',2), grid on ; hold on;
    plot([yF_R(k) yF_F(k)],[xF_R(k) xF_F(k)], 'r-', 'LineWidth',8); % front
    plot([yR_R(k) yR_F(k)],[xR_R(k) xR_F(k)], 'r-', 'LineWidth',8); % rear
    plot(y(k), x(k), 'ko','markersize', 6, 'LineWidth',4); 
    plot([yR(k) yF(k)],[xR(k) xF(k)], 'k-', 'LineWidth',4); hold off;

    axis([y(k)-lF*3 y(k)+lF*3 x(k)-lF*3 x(k)+lF*3]);
    axis equal
    drawnow
end

%% Dynmaics Bicycle Model