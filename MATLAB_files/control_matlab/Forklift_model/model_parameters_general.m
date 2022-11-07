%% Dynamic Model Parameters

%% Robot Parameters
m = 20; % mass of the vehicle
g = 9.8;
Iz = 0.1; % Ineratia of the vehicle

xbc = 0.4 ; % coordinates of the center of mass
ybc = 0 ; 

%% Initial Conditions
eta0 = [0;0;0]; % Initial pose and orientation
zeta0 = [0;0;0]; % Initial vector of inputs 

eta(:,1) = eta0;
zeta(:,1)= zeta0;

u = zeta(1,1);
v = zeta(2,1);
r = zeta(3,1);

%% State Equation
D = [m , 0 , -m*ybc ; 
     0 , m ,  m*xbc ;
    -m*ybc ,  m*xbc , Iz+m*(xbc^2+ybc^2) ];


D_inv = inv(D);

n_v = [-m*r*(v+xbc*r);
        m*r*(u-ybc*r);
        m*r*(xbc*u-ybc*v)];

tau = [1;0;0];
xs = [0;0;0];
x = out.state.Data;
Draw_MPC_PS_Obstacles(x,xs,1);