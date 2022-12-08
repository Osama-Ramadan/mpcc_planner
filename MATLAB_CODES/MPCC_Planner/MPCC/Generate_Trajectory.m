function [Xout, waypoints, Q] = Generate_Trajectory(goalsp, currentInt, count)

waypoints = goalsp(currentInt:currentInt+count,:);
%% Spline Fitting
x = waypoints(:,1);
y = waypoints(:,2);

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

Q = [Qx , Qy];

t = 0:0.01:1;
x_ts = [];
y_ts = [];
for k = 1:n-1
     x_t= (1/6)*(((1-t).^3*Qx(k)) + (3*t.^3-6*t.^2+4)*Qx(k+1) + (-3*t.^3+3*t.^2+3*t+1)*Qx(k+2) + t.^3*Qx(k+3));
     y_t= (1/6)*(((1-t).^3*Qy(k)) + (3*t.^3-6*t.^2+4)*Qy(k+1) + (-3*t.^3+3*t.^2+3*t+1)*Qy(k+2) + t.^3*Qy(k+3));
     x_ts = [x_ts ; x_t'];
     y_ts = [y_ts ; y_t'];
end

Xout = [x_ts,y_ts];


