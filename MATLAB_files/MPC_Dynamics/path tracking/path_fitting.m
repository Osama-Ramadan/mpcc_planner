clear; clc;
%% new way
waypoints = [0 0 0; 10 -5 0; 15 5 0; 10 15 pi/3; 0 15 pi/2];
waypoints=unique(waypoints,'rows','stable'); % <- this is required because your example as duplicated points
% Spline interpolation
t=cumsum([0;sqrt(sum(diff(waypoints,1,1).^2,2))]);
ti=linspace(t(1),t(end),128);
xyzi=interp1(t,waypoints,ti,'spline');
%plot
%figure
%plot(waypoints(:,1),waypoints(:,2),'or',xyzi(:,1),xyzi(:,2),'b');
%axis equal

%% Spline Fitting
goals = [-5 0; 5 5];
x = goals(:,1);
y = goals(:,2);

Px = [0; x; 0];
Py = [0; y; 0];

n = length(x);
phi(1:n+2,1:n+2) = 0;
for i = 1:n
    phi(i+1,i)= 1;
    phi(i+1,i+1) = 4;
    phi(i+1, i+2) = 1;
end

phi(1,1) = 1;
phi(1,2) = -2;
phi(1,3) = 1;
phi(n+2,n)=1;
phi(n+2,n+1) = -2;
phi(n+2,n+2) = 1;

Qx = 6*inv(phi)*Px;
Qy = 6*inv(phi)*Py;

t = 0:0.01:1;
x_ts = [];
y_ts = [];
for k = 1:n-1
     x_t= (1/6)*(((1-t).^3*Qx(k)) + (3*t.^3-6*t.^2+4)*Qx(k+1) + (-3*t.^3+3*t.^2+3*t+1)*Qx(k+2) + t.^3*Qx(k+3));
     y_t= (1/6)*(((1-t).^3*Qy(k)) + (3*t.^3-6*t.^2+4)*Qy(k+1) + (-3*t.^3+3*t.^2+3*t+1)*Qy(k+2) + t.^3*Qy(k+3));
     dx_t = (1/6)*((-3*(1-t).^2*Qx(k))+((9*t.^2-12*t)*Qx(k+1))+((-9*t.^2+6*t+3)*Qx(k+2))+(3*t.^2*Qx(k+3)));
     dy_t = (1/6)*((-3*(1-t).^2*Qy(k))+((9*t.^2-12*t)*Qy(k+1))+((-9*t.^2+6*t+3)*Qy(k+2))+(3*t.^2*Qy(k+3)));
     x_ts = [x_ts ; x_t'];
     y_ts = [y_ts ; y_t'];
end

slop = atan2(dy_t,dx_t);

theta = [];
for k = 1:length(x_ts)-1
    theta_t = atan2((y_ts(k+1)-y_ts(k)),(x_ts(k+1)-x_ts(k)));
    theta = [theta; theta_t];
end

