function [x0] = Test_Constraints(state)
theta = state(5);
%gamma = state(6);
gamma = 0;
ro = 0.127;
%ro = 0.5;
D = 0.5;
W = 0.15;
%x_dot = [state(7); state(8); state(9); state(10); state(11)];
%x_dot = [state(2); state(4); state(6); state(8); state(10)];
x_dot = state;
C=[1 0 D*sin(theta) -cos(theta+gamma)*ro 0;...
   0 1 -D*cos(theta) -sin(theta+gamma)*ro 0;...
   1 0 0 0 -cos(theta)*ro;...
   0 1 0 0 -sin(theta)*ro];

%C=[cos(theta) sin(theta) 0 ro/2 -ro/2;...
   %-sin(theta) cos(theta) 0 0 0;...
  % 0 0 1 0.5*ro/W 0.5*ro/W];

x0 = C*x_dot;
end
