import casadi.*
clear; clc;
%% Defining the CasADi variables and robot model
                                % -------------------------------
dt = 0.2;                    %[s]  sampling time 
N = 30;                     % prediction horizon

v_max = 0.5;                % Input constraints on both linear speed and steering speed
v_min = -v_max;
omega_max = pi/4; 
omega_min = -omega_max;


mB= 10;                     % mass of the chassis
d= 0;                      % distance from the center of the wheels to the center of mass of chassis
D=0.5;                     % half of wheel-to-wheel distance 
IB=0.5*mB*D^2;              % moment of inertia of the chassis
IT=IB+mB*d^2;
obs_diam = 1;
rob_diam = 1;

%% Define Symbols and Functions
x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta'); gamma= SX.sym('gamma');
x_dot = SX.sym('x_dot'); y_dot = SX.sym('y_dot'); theta_dot = SX.sym('theta_dot'); gamma_dot= SX.sym('gamma_dot');
tau = SX.sym('tau'); omega = SX.sym('omega');

q = [x; y; theta;gamma];                        % system states
q_dot = [x_dot; y_dot; theta_dot; gamma_dot];       % system states


states = [x;y;theta;gamma;x_dot;y_dot;theta_dot; gamma_dot];
n_states = length(states);


T = [tau*cos(theta+gamma);tau*sin(theta+gamma);-tau*sin(gamma)*(D-d);omega];           % system control inputs
Input = [tau;omega];
n_controls = 2;

M=[mB                   0            mB*d*sin(theta)      0   ;...
   0                    mB          -mB*d*cos(theta)      0   ;...
   mB*d*sin(theta) -mB*d*cos(theta)         IT            0   ;...
   0                    0               0                 1  ];

B= mB*d*theta_dot^2*[cos(theta);sin(theta);0;0];

C=[sin(theta+gamma)   sin(theta) ; ...
  -cos(theta+gamma)  -cos(theta) ; ...
   (D)*cos(gamma)          0   ; ...
        0                  0     ];

C = C';

Cdot=[cos(theta+gamma)*(theta_dot+gamma_dot)       cos(theta)*theta_dot ; ...
      sin(theta+gamma)*(theta_dot+gamma_dot)       sin(theta)*theta_dot ; ...
      -(D)*sin(gamma)*gamma_dot                             0         ; ...
                 0                                            0         ];
Cdot  = Cdot';

lambdas=-inv(C*inv(M)*C')*(C*inv(M)*(T-B)+Cdot*q_dot);
q_dd= inv(M)*(T-B+C'*lambdas);

f = Function('f',{states,Input},...
    {q_dd},...
    {'states','input'},{'q_dd'});

%% Optimization Variables

U = SX.sym('U',n_controls,N);                                                   % Decision variables (controls)
P = SX.sym('P',n_states + N*(n_states));                             % parameters (which include at the initial state of the robot and the reference state)
X = SX.sym('X',n_states,(N+1));                                                 % A vector that represents the states over the optimization problem.

%% Setting the MPC Problem 
                % --------------------------------------------------------

obj = 0;                                                                        % Objective function
g = [];                                                                         % constraints vector


%Q = diag([2, 2, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0]);                                % weighing matrices (states)
Q = diag([50, 50, 5]);
Q_terminal = diag([50, 50, 5]);
R = diag([0.01,0.01]);                                                             % weighing matrices (controls)

st  = X(:,1);                                                                   % initial state
g = [g;st-P(1:n_states)];                                                       % initial condition constraints

obs_x =    [];         
obs_y =    [];  
num_obs = size(obs_x);
obst_pose = [];

for k = 1:N
    st = X(:,k);  con = U(:,k);
    st_obj = X(1:3,k);
    if k < N
    obj = obj+(st_obj-P(k*(n_states)+1:k*(n_states)+3))'*Q*(st_obj-P(k*(n_states)+1:k*(n_states)+3)) + (con)'*R*(con) ; % calculate obj
    else
    obj = obj+(st_obj-P(k*(n_states)+1:k*(n_states)+3))'*Q_terminal*(st_obj-P(k*(n_states)+1:k*(n_states)+3)) + (con)'*R*(con) ; % calculate obj
    end

    st_next_euler = st;
    st_next = X(:,k+1);                 % get the next state symbols from the Big vector X
    x_dd = f(st,con);                   % get the dynamics expressed in the states st
    x_d = st(5:length(st)-1) + (dt*x_dd(1:3));          % get the next state by  euler method
    st_next_euler(5:length(st_next_euler)-1) = x_d(1:3);
    st_next_euler(length(st_next_euler)) = x_dd(4);
    x = st(1:3) + (dt*x_d);
    st_next_euler(1:3)= x;
    st_next_euler(4) = st(4) + (dt*x_dd(4));
    g = [g;st_next-st_next_euler];      % add the difference between the state obtained from the dynamics and the decision variable to the constraints
end

for k = 1:N+1                       % In this loop we add the condition to ensure collision free path to the constraints
    for j = 1:num_obs(1)
        g = [g ; -sqrt((X(1,k)-obs_x(j))^2+(X(2,k)-obs_y(j))^2) + (rob_diam/2 + obs_diam/2)];
    end
end

OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_controls*N,1)];        % make the decision variable one column  vector

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);                % struct to define the optimization problem

opts = struct;
%opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
%opts.ipopt.acceptable_tol =1e-8;
%opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);     % The actual solver

args = struct;
                % These constraints make sure that the states will follow the dynamics
args.lbg(1:n_states*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:n_states*(N+1)) = 0;  % 1e-20   % Equality constraints

args.lbg(n_states*(N+1)+1 : n_states*(N+1)+ num_obs(1)*(N+1)) = -100;   % inequality constraints
args.ubg(n_states*(N+1)+1 : n_states*(N+1)+ num_obs(1)*(N+1)) = -0.1;   % inequality constraints
               
             % Constraints over the states 
args.lbx(1:n_states:n_states*(N+1),1) = -30; %state x lower bound
args.ubx(1:n_states:n_states*(N+1),1) = 30;  %state x upper bound

args.lbx(2:n_states:n_states*(N+1),1) = -30; %state y lower bound
args.ubx(2:n_states:n_states*(N+1),1) = 30;  %state y upper bound

args.lbx(3:n_states:n_states*(N+1),1) = -2*pi; %state theta lower bound
args.ubx(3:n_states:n_states*(N+1),1) = 2*pi;  %state theta upper bound

args.lbx(4:n_states:n_states*(N+1),1) = -2*pi; %state gamma lower bound
args.ubx(4:n_states:n_states*(N+1),1) = 2*pi;  %state gamma upper bound

args.lbx(5:n_states:n_states*(N+1),1) = -2; %state xdot lower bound
args.ubx(5:n_states:n_states*(N+1),1) = v_max;  %state xdot upper bound

args.lbx(6:n_states:n_states*(N+1),1) = -2; %state ydot lower bound
args.ubx(6:n_states:n_states*(N+1),1) = v_max;  %state ydot upper bound

args.lbx(7:n_states:n_states*(N+1),1) = -30; %state thetad lower bound
args.ubx(7:n_states:n_states*(N+1),1) = 30;  %state thetad upper bound

args.lbx(8:n_states:n_states*(N+1),1) = -1; %state gamma_dot lower bound
args.ubx(8:n_states:n_states*(N+1),1) = 1;  %state gamma_dot upper bound


args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = -10; %v lower bound
args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = v_max; %v upper bound

args.lbx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = -1; %omega lower bound
args.ubx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = 1; %omega upper bound

%% THE SIMULATION LOOP SHOULD START FROM HERE
                %-------------------------------------------



% MPC Controller 

t0 = 0;
x0 = [0 ; 0 ; 0 ; 0; 0 ; 0 ; 0 ; 0];                              % initial state.
%xs = [1.5 ; 1.5 ; pi ; 0.0];                           % Reference posture.
v_ref = v_max; 
omega_ref = 0;

x_hist(:,1) = x0;                                      % contains the history of states
t(1) = t0;

u0 = zeros(N,2);                                       % control inputs for  robot
X0 = repmat(x0,1,N+1)';                                % initialization of the states decision variables

sim_tim = 20;                                          % Maximum simulation time in (s)

mpciter = 0;                                           % Number of iterations

predicted_x = [];                                      % The predicted solution from the controller
u_cl=[];                                               % Input control calculated from controller

t_mpc(1) = 0;                          % Store times needed to solve the problem
mpc_cost = 0;                                          % Store total cost
mpc_costs = [];                                        % Store costs at each time step

% Genetrate the Circular Trajectory
Tinit = 0;
Tfinal = sim_tim;
Tvec=(Tinit : dt : Tfinal+N);
%circleode = @(~,x)Forkleft_ODE(t,D,v_ref,omega_ref,x);
%[Tout, Xout] = ode45(circleode, Tvec, x0); % Solve the ODE to get the trajectory

[Xout, ref, wayps] = Generate_Trajectory(D,Tvec);
i=0;
while(t0<50) 

    current_time = mpciter*dt;                           % get the current time
    args.p(1:n_states) = x0;                            % initial condition of the robot 

    for k = 1:N                                         % Update the values of the reference trajecotry
        i = dsearchn(Xout(:,(1:2)),x0(1:2,:)');
        args.p(k*(n_states)+1:k*(n_states)+8) = [Xout(k+i,1), Xout(k+i,2), 0, 0, 0, 0, 0, 0];
    end
 
                                                         % initial value of the optimization variables
    args.x0  = [reshape(X0',n_states*(N+1),1);reshape(u0',n_controls*N,1)];
    
    tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...    % The problem is solver here ..                                 
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    t_mpc(mpciter+1) = toc; 

                                                    % extract controls only from the solution
    u = reshape(full(sol.x(n_states*(N+1)+1:end))',n_controls,N)'; 
                                                    % extract the predicted TRAJECTORY
    predicted_x(:,1:n_states,mpciter+1)= reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; 

    u_cl= [u_cl ; u(1,:)];                                   % get first control input
    t(mpciter+1) = t0;
    [x0, u0] = dynamic_red_shift(dt, x0, u,f);           % Apply control input and get the next initial state from the dynamics
    t0 = t0+dt;

    x_hist(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)';             % extract the predicted TRAJECTORY
    X0 = [X0(2:end,:);X0(end,:)];                                           % Shift trajectory to initialize the next step

    mpciter = mpciter + 1;

end
%% Plotting and Data logging 

fprintf('Mean solver time :    %g [ms]\n', mean(t_mpc)*1e3);
fprintf('Median solver time :  %g [ms]\n', median(t_mpc)*1e3);
fprintf('Max solver time :     %g [ms]\n', max(t_mpc)*1e3);

Draw_MPC_tracking (t,x_hist,predicted_x,u_cl,Xout,N,rob_diam, obs_diam,obs_x, obs_y, t_mpc, wayps)