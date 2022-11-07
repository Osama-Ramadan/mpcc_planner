function [diff, diff1] =constraints_test(Z,q_dd)

D = 0.5;
ro=0.127;
alpha = Z(:,6);
alphadot = Z(:,12);
theta = Z(:,3);
thetadot = Z(:,9);

x_dot = [Z(:,7);Z(:,8);Z(:,9);Z(:,10);Z(:,11)];


C=[1 0 D*sin(theta) -cos(theta+alpha)*ro 0;...
   0 1 -D*cos(theta) -sin(theta+alpha)*ro 0;...
   1 0 0 0 -cos(theta)*ro;...
   0 1 0 0 -sin(theta)*ro];

Cdot=[0 0 D*thetadot*cos(theta) sin(theta+alpha)*ro*(thetadot+alphadot) 0;...
      0 0 D*thetadot*sin(theta) -cos(theta+alpha)*ro*(thetadot+alphadot) 0;...
      0 0 0 0 sin(theta)*thetadot*ro;...
      0 0 0 0 -cos(theta)*thetadot*ro];

diff = C*q_dd' + Cdot*x_dot;
diff1 = C*x_dot;
end
