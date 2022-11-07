%% Dynamic Model Parameters 

b = 0.5;
d = 0.5;
D = b+d;
m = 100;
J = 0.1;

T = 0.1;
K = 1;

A = (J/D) - (m*b*d/D);
B = b*m/D;
F_d = 5;
phi = 0;