import casadi.*
clear;
clc;

% parameters
dt = 0.2;                    %[s]  sampling time 
sim_time = 25;
N = 50;                     % prediction horizon

mw= 1;                      % mass of wheels
mB= 50;                     % mass of the chassis
d= -0.0;                      % distance from the center of the wheels to the center of mass of chassis
D=0.5;                     % half of wheel-to-wheel distance 
IB=0.5*mB*D^2;              % moment of inertia of the chassis
ro=0.127;                   % radius of the wheels
mT=mB+2*mw;                      % total mass
mx = mB*d+mw*D;
IT=IB+mB*d^2;
Iyy=mw*(ro^2)/2;
obs_diam = 0.5;
rob_diam = 0.5;

%% Dynamic Model ------------------------------------------------
x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta'); gamma= SX.sym('gamma'); phi1= SX.sym('phi1'); phi2= SX.sym('phi2');  % define the symbols for CasADi
x_dot = SX.sym('x_dot'); y_dot = SX.sym('y_dot'); theta_dot = SX.sym('theta_dot'); gamma_dot= SX.sym('gamma_dot'); phi1_dot= SX.sym('phi1_dot'); phi2_dot= SX.sym('phi2_dot');  % define the symbols for CasADi
tau = SX.sym('tau'); omega = SX.sym('omega');

q = [x; y; theta; phi1; phi2; gamma];                        % system states
q_dot = [x_dot; y_dot; theta_dot; phi1_dot; phi2_dot];       % system states

states = [x;y;theta;phi1;phi2;gamma;x_dot;y_dot;theta_dot; phi1_dot; phi2_dot; gamma_dot];
n_states = length(states);
opt_states = [x,y,theta];
n_opt_states = length(opt_states);

T = [0;0;0;tau;0];           % system control inputs
Input = [tau;omega];
n_controls = 2;


M=[mT                   0            mx*sin(theta)      0   0;...
   0                    mT          -mx*cos(theta)      0   0;...
   mx*sin(theta) -mx*cos(theta)         IT              0   0;...
   0                    0               0               Iyy 0;...
   0                    0               0               0  Iyy];

B= mx*theta_dot^2*[cos(theta);sin(theta);0;0;0];

C=[1 0 D*sin(theta) -cos(theta+gamma)*ro 0;...
   0 1 -D*cos(theta) -sin(theta+gamma)*ro 0;...
   1 0 0 0 -cos(theta)*ro;...
   0 1 0 0 -sin(theta)*ro];

Cdot=[0 0 D*theta_dot*cos(theta) sin(theta+gamma)*ro*(theta_dot+gamma_dot) 0;...
      0 0 D*theta_dot*sin(theta) -cos(theta+gamma)*ro*(theta_dot+gamma_dot) 0;...
      0 0 0 0 sin(theta)*theta_dot*ro;...
      0 0 0 0 -cos(theta)*theta_dot*ro];

lambdas=-inv(C*inv(M)*C')*(C*inv(M)*(T-B)+Cdot*q_dot);
q_dd= inv(M)*(T-B+C'*lambdas);
gamma_dot = omega;
q_dd = [q_dd;gamma_dot];
f = Function('f',{states,Input},...
    {q_dd},...
    {'states','input'},{'q_dd'});            

%% ----------------------------------------------------------------------------------------------------------------

U = SX.sym('U',n_controls,N);                                                   % Decision variables (controls)
P = SX.sym('P',n_states + n_states);                                            % parameters (which include at the initial state of the robot and the reference state)
X = SX.sym('X',n_states,(N+1));                                                 % A vector that represents the states over the optimization problem.

obj = 0;                                                                        % Objective function
g = [];                                                                         % constraints vector

obs_x =    [3; 3.5];         
obs_y =    [3; 3];  
num_obs = size(obs_x);
obst_pose = [];

Q = diag([2, 2, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0]);                    % weighing matrices (states)
Q_Terminal = diag([20,20, 2, 0, 0.0, 0.0, 0, 0, 0, 0, 0, 0]);                    % weighing matrices (states)
R = diag([0.01,0.01]);                                                             % weighing matrices (controls)
S = 10;

st  = X(1:n_states,1);                                                                   % initial state
g = [g;st-P(1:n_states)];                                                       % initial condition constraints

for k = 1:N
     st = X(1:n_states,k);  con = U(:,k);
     if k < N
        obj = obj+(st-P(n_states+1:2*n_states))'*Q*(st-P(n_states+1:2*n_states)) + con'*R*con; % calculate obj
     else
        obj = obj+(st-P(n_states+1:2*n_states))'*Q_Terminal*(st-P(n_states+1:2*n_states)) + con'*R*con; % calculate obj
     end
end

for k = 1:N
    st = X(:,k);  con = U(:,k);
    st_next_euler = st;
    st_next = X(:,k+1);                 % get the next state symbols from the Big vector X
    x_dd = f(st,con);                   % get the dynamics expressed in the states st
    x_d = st(7:length(st)-1) + (dt*x_dd(1:5));          % get the next state by  euler method
    st_next_euler(7:length(st_next_euler)-1) = x_d(1:5);
    st_next_euler(length(st_next_euler)) = x_dd(6);
    x = st(1:5) + (dt*x_d);
    st_next_euler(1:5)= x;
    st_next_euler(6) = st(6) + (dt*x_dd(6));
    g = [g;st_next-st_next_euler];      % add the difference between the state obtained from the dynamics and the decision variable to the constraints
end

for k = 1:N+1                       % In this loop we add the condition to ensure collision free path to the constraints
    for j = 1:num_obs(1)
        g = [g ; -sqrt((X(1,k)-obs_x(j))^2+(X(2,k)-obs_y(j))^2) + (rob_diam/2 + obs_diam/2)];
    end
end

OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_controls*N,1)];        % make the decision variable one column  vector

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);                % struct to define the optimization problem

opts = struct;                                                                  % struct to stablish the optimization options 
opts.ipopt.max_iter = 200;
opts.ipopt.print_level =0;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);     % The actual solver

args = struct;
                % These constraints make sure that the states will follow the dynamics
args.lbg(1:n_states*(N+1)) = 0;     % equality constraints 
args.ubg(1:n_states*(N+1)) = 0;     % equality constraints

args.lbg(n_states*(N+1)+1 : n_states*(N+1)+ num_obs(1)*(N+1)) = -100;   % inequality constraints
args.ubg(n_states*(N+1)+1 : n_states*(N+1)+ num_obs(1)*(N+1)) = -0.1;   % inequality constraints

   % Constraints over the states 
args.lbx(1:n_states:n_states*(N+1),1) = -30; %state x lower bound
args.ubx(1:n_states:n_states*(N+1),1) = 30;  %state x upper bound

args.lbx(2:n_states:n_states*(N+1),1) = -30; %state y lower bound
args.ubx(2:n_states:n_states*(N+1),1) = 30;  %state y upper bound

args.lbx(3:n_states:n_states*(N+1),1) = -pi; %state theta lower bound
args.ubx(3:n_states:n_states*(N+1),1) = pi;  %state theta upper bound

args.lbx(4:n_states:n_states*(N+1),1) = -inf; %state phi1 lower bound
args.ubx(4:n_states:n_states*(N+1),1) = inf;  %state phi1 upper bound

args.lbx(5:n_states:n_states*(N+1),1) = -inf; %state phi2 lower bound
args.ubx(5:n_states:n_states*(N+1),1) = inf;  %state phi2 upper bound

args.lbx(6:n_states:n_states*(N+1),1) = -pi; %state gamma lower bound
args.ubx(6:n_states:n_states*(N+1),1) = pi;  %state gamma upper bound

args.lbx(7:n_states:n_states*(N+1),1) = -2; %state xdot lower bound
args.ubx(7:n_states:n_states*(N+1),1) = 0.3;  %state xdot upper bound

args.lbx(8:n_states:n_states*(N+1),1) = -2; %state ydot lower bound
args.ubx(8:n_states:n_states*(N+1),1) = 0.3;  %state ydot upper bound

args.lbx(9:n_states:n_states*(N+1),1) = -30; %state thetad lower bound
args.ubx(9:n_states:n_states*(N+1),1) = 30;  %state thetad upper bound

args.lbx(10:n_states:n_states*(N+1),1) = -inf; %state phi1d lower bound
args.ubx(10:n_states:n_states*(N+1),1) = inf;  %state phi1d upper bound

args.lbx(11:n_states:n_states*(N+1),1) = -inf; %state phi2d lower bound
args.ubx(11:n_states:n_states*(N+1),1) = inf;  %state phi2d upper bound

args.lbx(12:n_states:n_states*(N+1),1) = -0.5; %state gamma_dot lower bound
args.ubx(12:n_states:n_states*(N+1),1) = 0.5;  %state gamma_dot upper bound

args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = -10; %v lower bound
args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = 3; %v upper bound

args.lbx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = -0.5; %omega lower bound
args.ubx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = 0.5; %omega upper bound
%----------------------------------------------
t0 = 0;
x0 = [0 ; 0 ; 0 ; 0.0; 0 ; 0 ; 0.0 ; 0.0; 0; 0; 0; 0];                              % initial state.
xs = [5 ; 5 ; pi/2 ; 0.0; 0 ; 0 ; 0.0 ; 0.0; 0; 0; 0; 0];                                                         % Reference posture.
mpciter = 0 ;
t(1) = t0;

u0 = zeros(N,2);                                       % control inputs for  robot
X0 = repmat(x0,1,N+1)';                                % initialization of the states decision variables

u_cl=[];                                               % Input control calculated from controller
predicted_x = [];                                      % The predicted solution from the controller

while(mpciter < sim_time/dt)
 
    args.p   = [x0;xs];                             % set the values of initial state and the reference state
                                                    % initial value of the optimization variables
    args.x0  = [reshape(X0',n_states*(N+1),1);reshape(u0',n_controls*N,1)];
    
    tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...  % the optimization problem is solved here .. 
       'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    t_mpc(mpciter+1) = toc;                                           % get time needed to solve the problem

                                                    % extract controls only from the solution
    u = reshape(full(sol.x(n_states*(N+1)+1:end))',n_controls,N)';

    x_x(:,1:n_states) = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)';
                                                    % extract the predicted TRAJECTORY
    predicted_x(:,1:n_states,mpciter+1)= reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; 
    u_cl= [u_cl ; u(1,:)];                          % get first control input
    t(mpciter+1) = t0;
    [x0] = Dynamic_shift(dt,x0,u,f);           % Apply control input and get the next initial state from the dynamics
    t0 = t0+dt;
    xx_hist(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; % extract the predicted TRAJECTORY
    X0 = [X0(2:end,:);X0(end,:)];                               % Shift trajectory to initialize the next step
    
    mpciter = mpciter + 1;
end

Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose)