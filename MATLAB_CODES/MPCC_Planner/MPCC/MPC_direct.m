import casadi.*
clear; clc;
%% Define The Global Map

mapmat = load('Maps/warehouse_map.mat');
mapmat = mapmat.imageOccupancy;
map = occupancyMap(mapmat,5);

%mapmat = load('Maps/map3.mat');
%mapmat = mapmat.mapmat;
%map = binaryOccupancyMap(mapmat,10);

%% Defining the CasADi variables and robot model
                                % -------------------------------
dt = 0.2;                    %[s]  sampling time 
N = 30;                     % prediction horizon
N_obs = 30;


v_max = 0.5;                % Input constraints on both linear speed and steering speed
v_min = -v_max;
omega_max = pi/4; 
omega_min = -omega_max;


mB= 10;                     % mass of the chassis
d= 0;                      % distance from the center of the wheels to the center of mass of chassis
D=1.8;                     % half of wheel-to-wheel distance 
IB=0.5*mB*D^2;              % moment of inertia of the chassis
IT=IB+mB*d^2;
obs_diam = 1;
rob_diam = 1;

%% Define Symbols and Functions
x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta'); gamma= SX.sym('gamma'); w= SX.sym('w');
x_dot = SX.sym('x_dot'); y_dot = SX.sym('y_dot'); theta_dot = SX.sym('theta_dot'); gamma_dot= SX.sym('gamma_dot');
tau = SX.sym('tau'); omega = SX.sym('omega'); wd = SX.sym('wd');

q = [x; y; theta;gamma];                        % system states
q_dot = [x_dot; y_dot; theta_dot; gamma_dot];       % system states


states = [x;y;theta;gamma;x_dot;y_dot;theta_dot; gamma_dot];
n_states = length(states);


Torque = [tau*cos(theta+gamma);tau*sin(theta+gamma);-tau*sin(gamma)*(D-d);omega];           % system control inputs
Input = [tau;omega];
n_controls = length(Input);

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

lambdas=-inv(C*inv(M)*C')*(C*inv(M)*(Torque-B)+Cdot*q_dot);
q_dd= inv(M)*(Torque-B+C'*lambdas);

f = Function('f',{states,Input},...
    {q_dd},...
    {'states','input'},{'q_dd'});

%% Optimization Variables

U = SX.sym('U',n_controls,N);                                                   % Decision variables (controls)
P = SX.sym('P',n_states + 2);                             % parameters (which include at the initial state of the robot and the reference state)
X = SX.sym('X',n_states,(N+1));                                                 % A vector that represents the states over the optimization problem.

%% Setting the MPC Problem 
                % --------------------------------------------------------

obj = 0;                                                                        % Objective function
obj_far = 0;
obj_close = 0;
g = [];                                                                         % constraints vector

obs_x_static =    [];         
obs_y_static =    [];  
num_obs = size(obs_x_static);

st  = X(:,1);                                                                   % initial state
g = [g;st-P(1:n_states)];                                                       % initial condition constraints



% cost function ------------------------------------
Q = diag([100, 100]);
Q_Terminal = diag([500, 500]);
R = diag([2,2]);  
for k = 1:N
    st = X(1:2,k);  con = U(1:2,k);
    if k < N
        obj = obj+(st-P(n_states+1:end))'*Q*(st-P(n_states+1:end)) + con'*R*con; % calculate obj
    else
        obj = obj+(st-P(n_states+1:end))'*Q_Terminal*(st-P(n_states+1:end)) + con'*R*con; % calculate obj
    end
end

% Dynamic equation constraints ------------------------------------------
for k = 1:N
    st = X(:,k);  con = U(:,k);
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

% Constraints to avoid static obstacles ---------------------------------
for k = 1:N+1                       
    for j = 1:num_obs(1)
        g = [g ; -sqrt((X(1,k)-obs_x_static(j))^2+(X(2,k)-obs_y_static(j))^2) + (rob_diam/2 + obs_diam/2)];
    end
end

OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_controls*N,1)];        % make the decision variable one column  vector

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);                % struct to define the optimization problem

opts = struct;
opts.ipopt.max_iter = 20;
opts.ipopt.print_level =0;
opts.print_time = 0;
%opts.ipopt.acceptable_tol =1e-8;
%opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);     % The actual solver

%% MPC Constraints
args = struct;
                % These constraints make sure that the states will follow the dynamics
args.lbg(1:n_states*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:n_states*(N+1)) = 0;  % 1e-20   % Equality constraints

args.lbg(n_states*(N+1)+1 : n_states*(N+1)+num_obs*(N+1)) = -100;   % inequality constraints
args.ubg(n_states*(N+1)+1 : n_states*(N+1)+num_obs*(N+1)) = -0.1;   % inequality constraints
         
% Constraints over the states 
args.lbx(1:n_states:n_states*(N+1),1) = -30; %state x lower bound
args.ubx(1:n_states:n_states*(N+1),1) = 30;  %state x upper bound

args.lbx(2:n_states:n_states*(N+1),1) = -30; %state y lower bound
args.ubx(2:n_states:n_states*(N+1),1) = 30;  %state y upper bound

args.lbx(3:n_states:n_states*(N+1),1) = -30; %state theta lower bound
args.ubx(3:n_states:n_states*(N+1),1) =  30;  %state theta upper bound

args.lbx(4:n_states:n_states*(N+1),1) = -80*(pi/180); %state gamma lower bound
args.ubx(4:n_states:n_states*(N+1),1) =  80*(pi/180);  %state gamma upper bound

args.lbx(5:n_states:n_states*(N+1),1) = v_min; %state xdot lower bound
args.ubx(5:n_states:n_states*(N+1),1) = v_max;  %state xdot upper bound

args.lbx(6:n_states:n_states*(N+1),1) = v_min; %state ydot lower bound
args.ubx(6:n_states:n_states*(N+1),1) = v_max;  %state ydot upper bound

args.lbx(7:n_states:n_states*(N+1),1) = -30; %state thetad lower bound
args.ubx(7:n_states:n_states*(N+1),1) = 30;  %state thetad upper bound

args.lbx(8:n_states:n_states*(N+1),1) = -1.5; %state gamma_dot lower bound
args.ubx(8:n_states:n_states*(N+1),1) = 1.5;  %state gamma_dot upper bound

args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = -10; %v lower bound
args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = 10; %v upper bound

args.lbx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = -1.5; %omega lower bound
args.ubx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = 1.5; %omega upper bound


%% Initial Values
t0 = 0;
x0 = [37 ; 3 ; pi/4 ; 0; 0 ; 0 ; 0; 0];                              % initial state.

x_hist(:,1) = x0;                                      % contains the history of states
time(1) = t0;

u0 = zeros(N,n_controls);                                       % control inputs for  robot
X0 = repmat(x0,1,N+1)';                                % initialization of the states decision variables

mpciter = 0;                                           % Number of iterations

predicted_x = [];                                      % The predicted solution from the controller
u_cl=[];                                               % Input control calculated from controller

t_mpc(1) = 0;                                          % Store times needed to solve the problem
mpc_cost = 0;                                          % Store total cost
mpc_costs = [];                                        % Store costs at each time step
safe_zone = [];

%% Genetrate the Circular Trajectory
%goalsp = [4 23 ; 18 20; 16 10; 25 7 ];
goalsp = [37 3 ; 37 15; 20 20; 9 33 ]; % (warehouse)
lookahead = 3;
currentGoalIndex = 1;

objec = [];
pred_x = x0;

%% Dynamic Obstacles
obs_poses = [];
predicted_obs_poses = [];
x_pred = [];
collision_points = [];
collision_time = [];
dist_to_coll = 100;
dist_t_c = -1;
x0_obs = [8;6;0;0];
obs_v = [0.0;0.0];
x_dot_obs =[[1;0;0;0],...
            [0;1;0;0]]*obs_v;
valid_seg = true;

current_obs_pose = x0_obs;
obj_far_st = obj_far;
obj_close_st = obj_close;
g_static = g;
lbg_st = args.lbg;
ubg_st = args.ubg;

%% Kalman Filter Initialization
    Pred_cov0 = [1,0,0,0;...
      0,1,0,0;...
      0,0,1,0;...
      0,0,0,1];
    [pred_x_N, estimated_x, estimated_p] = kalman_Filter(x0_obs,N+1,dt,x0_obs,Pred_cov0);

%% LOOP STARTING

while(mpciter < 250) 
    current_time = mpciter*dt;                              % get the current time

    obs_position = current_obs_pose + dt*x_dot_obs;     % one forward step for the dynamic obstacle
    obs_poses(:,mpciter+1)= obs_position;
    [pred_x_N, estimated_x, estimated_p] = kalman_Filter(obs_position,N_obs+1,dt,estimated_x,estimated_p);
    predicted_obs_poses(:,:,mpciter+1)= pred_x_N;
    current_obs_pose = obs_position;


    if size(predicted_x,1) > 10                         % make sure one run has been done and we have some predicted states
        pred_x = predicted_x(10,1:2,mpciter)';         % take the last predicted state
    end

    args.p(1:n_states) = x0;                                                % initial condition of the robot 
    

    currentgoal = goalsp(currentGoalIndex,:);
    if sqrt((x0(1)-currentgoal(1))^2+(x0(2)-currentgoal(2))^2) <= 0.5                           % distance between current goal and robot less than 1m
        currentGoalIndex = min(currentGoalIndex+1,size(goalsp,1));
        currentgoal = goalsp(currentGoalIndex,:);
    end
    args.p(n_states+1) = currentgoal(1);
    args.p(n_states+2) = currentgoal(2);

    % Update the local safe Area
    max_search_x = 2;
    max_search_y = 2;
    safe_zone_boarders = [];
    safe_zone_min_x = [];
    safe_zone_min_y = [];
    safe_zone_max_x = [];
    safe_zone_max_y = [];

    for k = 1:N+1
    if ~isempty(predicted_x)
    predicted_x_s = predicted_x(k,:,mpciter);
    robot_curr_x = predicted_x_s(1);
    robot_curr_y = predicted_x_s(2);
    robot_curr_th = predicted_x_s(3);
    index_in_map = local2grid(map,[robot_curr_x, robot_curr_y]);
    else
        robot_curr_x = x0(1);
        robot_curr_y = x0(2);
        robot_curr_th = x0(3);
        index_in_map = local2grid(map,[robot_curr_x, robot_curr_y]);
    end
    
    min_y_search_index = max(index_in_map(1)-max_search_y*map.Resolution,1);
    max_y_search_index = min(index_in_map(1)+max_search_y*map.Resolution,size(mapmat,1));
    min_x_search_index = max(index_in_map(2)-max_search_x*map.Resolution,1);
    max_x_search_index = min(index_in_map(2)+max_search_x*map.Resolution,size(mapmat,2));

    searchmatrix = mapmat(min_y_search_index:max_y_search_index,...
                          min_x_search_index:max_x_search_index);
    
    search_area_min_x = index_in_map(2)-max_search_x*map.Resolution;
    search_area_max_y = index_in_map(1)-max_search_y*map.Resolution;

    [leftcorner, rightcorner] = getMaxSearchArea(searchmatrix, max_search_x*10,max_search_y*10);
    Upper_left_corner_local = grid2local(map,[search_area_max_y+leftcorner(2),search_area_min_x+leftcorner(1)]);
    Lower_right_corner_local = grid2local(map,[search_area_max_y+rightcorner(2),search_area_min_x+rightcorner(1)]);
    


    safe_zone_min_x1 = Upper_left_corner_local(1)+0.2;
    safe_zone_max_x1 = Lower_right_corner_local(1)-0.2;
    safe_zone_min_y1 = Lower_right_corner_local(2)+0.2;
    safe_zone_max_y1 = Upper_left_corner_local(2)-0.2; 

    safe_zone_min_x = [safe_zone_min_x ; safe_zone_min_x1];
    safe_zone_min_y = [safe_zone_min_y ; safe_zone_min_y1];
    safe_zone_max_x = [safe_zone_max_x ; safe_zone_max_x1];
    safe_zone_max_y = [safe_zone_max_y ; safe_zone_max_y1];
    
    safe_zone = [safe_zone; [safe_zone_min_x1 safe_zone_max_x1 safe_zone_min_y1 safe_zone_max_y1]];
    end

    args.lbx(1:n_states:n_states*(N+1),1) = [safe_zone_min_x]; %state x lower bound
    args.ubx(1:n_states:n_states*(N+1),1) = [safe_zone_max_x];  %state x upper bound


    args.lbx(2:n_states:n_states*(N+1),1) = [safe_zone_min_y]; %state y lower bound
    args.ubx(2:n_states:n_states*(N+1),1) = [safe_zone_max_y];  %state y upper bound
    
    nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);                % struct to define the optimization problem
    solver = nlpsol('solver', 'ipopt', nlp_prob,opts);     % The actual solver

    % initial value of the optimization variables
    args.x0  = [reshape(X0',n_states*(N+1),1);reshape(u0',n_controls*N,1)];
    
    tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...    % The problem is solver here ..                                 
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    t_mpc(mpciter+1) = toc; 
    
    objec = [objec ; sol.f];

                                                    % extract controls only from the solution
    u = reshape(full(sol.x(n_states*(N+1)+1:n_states*(N+1)+n_controls*N))',n_controls,N)'; 
                                                    % extract the predicted TRAJECTORY
    x_pred = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)';
    predicted_x(:,1:n_states,mpciter+1)= reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; 

    u_cl= [u_cl ; u(1,:)];                                   % get first control input
    time(mpciter+1) = t0;
    [x0, u0] = dynamic_shift_Nopath(dt, x0, u,f);           % Apply control input and get the next initial state from the dynamics
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
currXout = [];
Q_p = [];
Draw_MPC_tracking (time,x_hist,predicted_x,u_cl,currXout,N,rob_diam, obs_diam,predicted_obs_poses ,t_mpc, goalsp, collision_points,map, safe_zone, Q_p)