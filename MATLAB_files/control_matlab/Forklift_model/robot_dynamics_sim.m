%% Dynamic model for wheeled robot 

clear all ; clc ; close all;

%% Simulation parameters
dt = 0.1 ; %step size
ts = 10; % simulation time 
t = 0:dt:ts; % time span

%% Initial Conditions
eta0 = [0;0;0]; % Initial pose and orientation
zeta0 = [0;0;0] % Initial vector of inputs 

eta(:,1) = eta0;
zeta(:,1)= zeta0;

%% Robot Parameters
m = 10; % mass of the vehicle
Iz = 0.1; % Ineratia of the vehicle

xbc = 0 ; % coordinates of the center of mass
ybc = 0 ; 

%% State Propagation

for i = 1:length(t)
    u = zeta(1,i);
    v = zeta(2,i);
    r = zeta(3,i);
    
    D = [m , 0 , -m*ybc ; 
         0 , m ,  m*xbc ;
        -m*ybc ,  m*xbc , Iz+m*(xbc^2+ybc^2) ];

    n_v = [-m*r*(v+xbc*r);
            m*r*(u-ybc*r);
            m*r*(xbc*u-ybx*v)];

    tau(:,i) = [1;0;0];
    
    psi = eta(3,i);
    J_eta = [cos(psi), -sin(psi), 0 ; 
             sin(psi), cos(psi),  0 ; 
             0 , 0 , 1];  % Jacobian

    zeta_dot(:,i) = inv(D)*(tau(:,i)-n_v);
    zeta(:,i+1) = zeta(:,i)+dt*zeta_dot(:,i);  % velocity update

    eta(:,i+1) = eta(:,i) + dt * (J_eta*(zeta(:,i)+zeta_dot(:,i))); %state update

    
end