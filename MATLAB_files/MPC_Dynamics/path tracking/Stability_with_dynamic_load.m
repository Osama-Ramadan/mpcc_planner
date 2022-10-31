%% Vehicle and Load Parameters

m_l = 1000;             % mass of the load
m_b = 2000;             % mass of the forklift

d_l = 1.5;              % distance from load to support triangle
d_b = 1;                % distance from forklift CG to support triangle
g = 9.81;               % gravitatinal acceleration

h_b = 0.5;              % hight of forklift CG
h_l = 1;                % hight of load

l_b = 0.5;              % distance from forklift CG to support wheels
l_l = 0.5;              % distance from load to support wheels

a_bn = 1;               % normal acceleration of forklift CG
v_r = 1;               % forward velocity of forklift CG
v_rdot = 1;             % forward acceleration of forklift CG

k_r = 1;

%% calculate max curvature

a_bn = 1:0.1:100;

k = abs(((d_l*m_l+d_b*m_b)*g-sign(k_r)*(h_b*m_b*a_bn))/(h_l*m_l*(v_r^2+l_l*v_rdot)));

plot(k,'LineWidth',2)
grid on
xlabel('Normal Acceleration (N)')
ylabel('Maximum raduis of Curvature (rad/m)')