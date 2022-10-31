import casadi.*
clear; clc;
%% Define The Global Map
mapmat = load('Maps/map1.mat');
mapmat = mapmat.mapmat;
map = binaryOccupancyMap(mapmat,10);

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


states = [x;y;theta;gamma;x_dot;y_dot;theta_dot; gamma_dot; w];
n_states = length(states);


Torque = [tau*cos(theta+gamma);tau*sin(theta+gamma);-tau*sin(gamma)*(D-d);omega];           % system control inputs
Input = [tau;omega;wd];
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

%% Spline Functions
Qx = SX.sym('Qx',4,1);
Qy = SX.sym('Qy',4,1);
t = SX.sym('t');
x_t= (1/6)*(((1-t).^3*Qx(1)) + (3*t.^3-6*t.^2+4)*Qx(2) + (-3*t.^3+3*t.^2+3*t+1)*Qx(3) + t.^3*Qx(4));
y_t= (1/6)*(((1-t).^3*Qy(1)) + (3*t.^3-6*t.^2+4)*Qy(2) + (-3*t.^3+3*t.^2+3*t+1)*Qy(3) + t.^3*Qy(4));
xd = [x_t ; y_t];
spline = Function('s',{Qx,Qy,t},{xd}, {'Qx','Qy', 't'},{'xd'});
     
dx_t = (1/6)*((-3*(1-t).^2*Qx(1))+((9*t.^2-12*t)*Qx(2))+((-9*t.^2+6*t+3)*Qx(3))+(3*t.^2*Qx(4)));
dy_t = (1/6)*((-3*(1-t).^2*Qy(1))+((9*t.^2-12*t)*Qy(2))+((-9*t.^2+6*t+3)*Qy(3))+(3*t.^2*Qy(4)));
theta_d = atan2(dy_t,dx_t);
drivative = Function('derv', {Qx,Qy,t},{theta_d},{'Qx','Qy', 't'},{'theta_d'});



%% Optimization Variables

U = SX.sym('U',n_controls,N);                                                   % Decision variables (controls)
P = SX.sym('P',n_states + 8 + 2);                             % parameters (which include at the initial state of the robot and the reference state)
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

%Q = diag([2, 2, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0]);                                % weighing matrices (states)
Q = diag([200, 1000]);
R = diag([10,100,1]);                                                             % weighing matrices (controls)
R_con = diag([1,10,1]); 
q_t = 400;
q_t_terminal = 10000;
st  = X(:,1);                                                                   % initial state

g = [g;st-P(1:n_states)];                                                       % initial condition constraints

% Close to the path cost function -----------------------------------
for k = 1:N
    st = X(:,k);  con = U(:,k);
    delta_x_obs = st(1) - obs_x_static;
    delta_y_obs = st(2) - obs_y_static;
   
    if k == N
        delta_u = con;
    else
        con_next = U(:,k+1);
        delta_u = con_next - con;
    end

    t_th = X(9,k);
    s = spline(P(n_states+1:n_states+4),P(n_states+5:n_states+8),t_th);
    head = drivative(P(n_states+1:n_states+4),P(n_states+5:n_states+8),t_th);

    e_c = sin(head)*(st(1)-s(1,1)) - cos(head)*(st(2)-s(2,1));
    e_l = -cos(head)*(st(1)-s(1,1)) - sin(head)*(st(2)-s(2,1));

    e = [e_c ; e_l];
    obj_close = obj_close+ e'*Q*e + (delta_u)'*R*(delta_u) - q_t*t_th; % calculate obj
    if k==N
        dist_to_way = sqrt((st(1)-P(end-1))^2+(st(2)-P(end))^2);
        obj_close = obj_close + dist_to_way*q_t_terminal;
    end
end

% Far from the path cost function ------------------------------------
Q = diag([1000, 2000]);
R = diag([20,20,10]);  
q_t = 15000;
for k = 1:N
    st = X(:,k);  con = U(:,k);
    delta_x_obs = st(1) - obs_x_static;
    delta_y_obs = st(2) - obs_y_static;
   
    if k == N
        delta_u = con;
    else
        con_next = U(:,k+1);
        delta_u = con_next - con;
    end

    t_th = X(9,k);
    s = spline(P(n_states+1:n_states+4),P(n_states+5:n_states+8),t_th);
    head = drivative(P(n_states+1:n_states+4),P(n_states+5:n_states+8),t_th);

    e_c = sin(head)*(st(1)-s(1,1)) - cos(head)*(st(2)-s(2,1));
    e_l = -cos(head)*(st(1)-s(1,1)) - sin(head)*(st(2)-s(2,1));

    e = [e_c ; e_l];
    obj_far = obj_far+ e'*Q*e + (delta_u)'*R*(delta_u) + q_t*t_th; % calculate obj
end

% Dynamic equation constraints ------------------------------------------
for k = 1:N
    st = X(:,k);  con = U(:,k);
    st_next_euler = st;
    st_next = X(:,k+1);                 % get the next state symbols from the Big vector X
    x_dd = f(st,con);                   % get the dynamics expressed in the states st
    x_d = st(5:length(st)-2) + (dt*x_dd(1:3));          % get the next state by  euler method
    st_next_euler(5:length(st_next_euler)-2) = x_d(1:3);
    st_next_euler(length(st_next_euler)-1) = x_dd(4);
    x = st(1:3) + (dt*x_d);
    st_next_euler(1:3)= x;
    st_next_euler(4) = st(4) + (dt*x_dd(4));
    st_next_euler(9) = st(9) + con(3);
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

args.lbx(9:n_states:n_states*(N+1),1) = 0; %state gamma_dot lower bound
args.ubx(9:n_states:n_states*(N+1),1) = 1;  %state gamma_dot upper bound

args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = -10; %v lower bound
args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = 10; %v upper bound

args.lbx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = -1.5; %omega lower bound
args.ubx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = 1.5; %omega upper bound

args.lbx(n_states*(N+1)+3:n_controls:n_states*(N+1)+n_controls*N,1) = 0.0005; %omega lower bound
args.ubx(n_states*(N+1)+3:n_controls:n_states*(N+1)+n_controls*N,1) = 0.01; %omega upper bound


%% Initial Values
t0 = 0;
x0 = [2 ; 12 ; 0 ; 0; 0 ; 0 ; 0 ; 0 ; 0.01];                              % initial state.
t_0 = 0.01;
v_0 = 0;
%xs = [1.5 ; 1.5 ; pi ; 0.0];                           % Reference posture.
v_ref = v_max; 
omega_ref = 0;

x_hist(:,1) = x0;                                      % contains the history of states
time(1) = t0;

u0 = zeros(N,n_controls);                                       % control inputs for  robot
X0 = repmat(x0,1,N+1)';                                % initialization of the states decision variables

sim_tim = 20;                                          % Maximum simulation time in (s)

mpciter = 0;                                           % Number of iterations

predicted_x = [];                                      % The predicted solution from the controller
u_cl=[];                                               % Input control calculated from controller

t_mpc(1) = 0;                                          % Store times needed to solve the problem
mpc_cost = 0;                                          % Store total cost
mpc_costs = [];                                        % Store costs at each time step
safe_zone = [];
%% Genetrate the Circular Trajectory
goalsp = [1 10 ;12.5 2];
lookahead = 1;
currentInt = 1;
[Xout, wayps, Q_p] = Generate_Trajectory(goalsp,currentInt,lookahead);
currXout = Xout;
indeces = [];
for ss = 1:length(wayps(:,1))
    indeces = [indeces ; dsearchn(Xout(:,(1:2)),wayps(ss,1:2))];
end
i=0;
seg = 1;
p_seg =1;
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
x0_obs = [0;0;0;0];
obs_v = [0;0];
x_dot_obs =[[1;0;0;0],...
            [0;1;0;0]]*obs_v;

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

while(mpciter < 280) 
    current_time = mpciter*dt;                              % get the current time
    Xc = 0;                                                 % reset collision point
    Tc = -1;                                                % reset collision time
    g = g_static;                                           % reset constraints
    obj_far = obj_far_st;                                   % reset objectives
    obj_close = obj_close_st;

    obs_position = current_obs_pose + dt*x_dot_obs;     % one forward step for the dynamic obstacle
    obs_poses(:,mpciter+1)= obs_position;
    [pred_x_N, estimated_x, estimated_p] = kalman_Filter(obs_position,N_obs+1,dt,estimated_x,estimated_p);
    predicted_obs_poses(:,:,mpciter+1)= pred_x_N;
    current_obs_pose = obs_position;
    
    if size(predicted_x,1) > 10                         % make sure one run has been done and we have some predicted states
        pred_x = predicted_x(N-1,1:2,mpciter)';         % take the last predicted state
    end
    
    i_pred = dsearchn(Xout(:,(1:2)),pred_x(1:2,:)');    % closest point index to the predicted state
    i = dsearchn(Xout(:,(1:2)),x0(1:2,:)');             % closest point index to the actual state
    i_org = i_pred;

    if sqrt((x0(1)-Xout(i,1))^2+(x0(2)-Xout(i,2)^2)) > 5    % check if you are 3 meter away from the path
        obj = obj_far;
    else
        obj = obj_close;
    end
    

    for k = seg+1:length(indeces)                           % have a counter with the length of all waypoints
        if i_pred > indeces(k)-2                            % if the predicted state reaches close to the next waypoint 
            seg = seg+1;                                    % go to the next segment
            p_seg = seg;
            if seg > length(indeces)-1                      % if you are in the last segment
                seg = length(indeces)-1;                    % keep being there
                p_seg = seg;
            end
        end
    end

    i = i - (seg-1)*100;                                    % make sure i always between 0 and 1 for the MPC

    if i <= 0                                               % if we still on the prevoius segment keep tracking the first point on the new one 
        i = 1;
    end
    
    if sqrt((x0(1)-pred_x_N(1,1))^2+(x0(2)-pred_x_N(2,1))^2) < 6            % if the distance between the dynamic obst and the vehicle less than 6m
            [Xc,Tc] = get_collision_point(x0,pred_x_N,1,Xout,seg,dt);       % calculate the collision point (with the path) and time
            if Xc~=1000                                                     % check if a collision point is found 
                collision_points = [collision_points , Xc];
                dist_to_coll = sqrt((x0(1)-Xc(1))^2+(x0(2)-Xc(2))^2);       % calculate distance to collision 
            end
            if Tc~=-1
                collision_time = [collision_time , Tc(1)-Tc(2)];            % calclate if a valid time to collision is found
            end

            if (Tc(1)~=-1 && abs(Tc(1)-Tc(2))<3)||(Tc(1)~=-1 && dist_to_coll<=dist_t_c)     % if the time to collision is less than 3s or dis to coll decreases
                x_c = Xc(1);
                y_c = Xc(2);
                dist_t_c = sqrt((x0(1)-x_c)^2+(x0(2)-y_c)^2);                               % update distance to collision
                for k = 1:N
                    st = X(:,k);
                    o_st = pred_x_N(:,k);
                    dist_v = sqrt((st(1)-x_c)^2+(st(2)-y_c)^2);             % distance between vehicle and collision point
                    dist_o = sqrt((o_st(1)-x_c)^2+(o_st(2)-y_c)^2);         % distance between obstacle and collison point
                    v_v = sqrt(st(5)^2+st(6)^2);                            % speed of vehicle
                    v_o = sqrt(o_st(3)^2+o_st(4)^2);                        % speed of obstacle
                    delta_t = abs((dist_v*v_o-dist_o*v_v)/(v_o+0.1));       % difference of collision time between vehicle and obstacle
                    obj = obj + 100/(delta_t+0.01);                         % update cost function
                end
            else
                g = g_static;                                               % if collision is far away continue normally
                args.lbg = lbg_st;
                args.ubg = ubg_st;
            end

    else                                                                    % if obstacle is not in the 6m range continue normally 
        g = g_static;
        args.lbg = lbg_st;
        args.ubg = ubg_st;

    end

    t_0 = i*0.01;                                                           % get path parameter from the index
    x0(9)= t_0;                                                             % update x0
    args.p(1:n_states) = x0;                                                % initial condition of the robot 
    

    Q_x_trans = Q_p(p_seg:p_seg+3,1);                                       % get the Qs for the current segment
    Q_y_trans = Q_p(p_seg:p_seg+3,2);
    args.p(n_states+1:n_states+4) = Q_x_trans;                              % update the paramters for MPC
    args.p(n_states+5:n_states+8) = Q_y_trans;
    
    % Next waypoint 
    next_way = goalsp(seg+1,:);
    args.p(n_states+9) = next_way(1);
    args.p(n_states+10) = next_way(2);

    if i_org >= indeces(min(3,length(indeces)))                             % update the trajectory if lookahead dist is completed
        seg = 1;
        currentInt = currentInt+1;
        if currentInt <= (length(goalsp(:,1))-lookahead)
            [Xout, wayps, Q_p] = Generate_Trajectory(goalsp,currentInt,lookahead);
            currXout = [currXout; Xout];
            indeces = [];
            for s = 1:length(wayps(:,1))
                indeces = [indeces ; dsearchn(Xout(:,(1:2)),wayps(s,1:2))];
            end
        end
    end
    
    % Update the local safe Area
    max_search_x = round(abs(x0(5)*4) + 1.5,1);
    max_search_y = round(abs(x0(6)*4) + 1.5,1);
    safe_zone_min_x = x0(1)-max_search_x;
    safe_zone_max_x = x0(1)+max_search_x;
    safe_zone_min_y = x0(2)-max_search_y;
    safe_zone_max_y = x0(2)+max_search_y;
    safe_zone_boarders = [];
    robot_curr_x = x0(1);
    robot_curr_y = x0(2);
    index_in_map = local2grid(map,[robot_curr_x, robot_curr_y]);

    for i = index_in_map(2):-1:index_in_map(2)-max_search_x*map.Resolution
        if(mapmat(index_in_map(1),max(i,1))==1)                             % make sure that i in 1 or more
            safe_zone_min_x = grid2local(map,[index_in_map(1) i+2]);
            safe_zone_min_x = safe_zone_min_x(1);
            break;
        end
    end

    for i = index_in_map(2):index_in_map(2)+max_search_x*map.Resolution
        if(mapmat(index_in_map(1),i)==1)
            safe_zone_max_x = grid2local(map,[index_in_map(1) i-2]);
            safe_zone_max_x = safe_zone_max_x(1);
            break;
        end
    end

    for i = index_in_map(1):-1:index_in_map(1)-max_search_y*map.Resolution
        if(mapmat(i,index_in_map(2))==1)
            safe_zone_max_y = grid2local(map,[i+2 index_in_map(2)]);
            safe_zone_max_y = safe_zone_max_y(2);
            break;
        end
    end

    for i = index_in_map(1):index_in_map(1)+max_search_y*map.Resolution
        if(mapmat(min(i,size(mapmat,1)),index_in_map(2))==1)
            safe_zone_min_y = grid2local(map,[i-2 index_in_map(2)]);
            safe_zone_min_y = safe_zone_min_y(2);
            break;
        end
    end

    if(x0(5)>0.1)
        safe_zone_min_x = robot_curr_x - 0.2;
    elseif(x0(5)<-0.1)
        safe_zone_max_x = robot_curr_x + 0.2;
    end

    if(x0(6)>0.1)
        safe_zone_min_y = robot_curr_y - 0.2;
    elseif(x0(6)<-0.1)
        safe_zone_max_y = robot_curr_y + 0.2;
    end
    

    args.lbx(1:n_states:n_states*(N+1),1) = safe_zone_min_x; %state x lower bound
    args.ubx(1:n_states:n_states*(N+1),1) = safe_zone_max_x;  %state x upper bound


    args.lbx(2:n_states:n_states*(N+1),1) = safe_zone_min_y; %state y lower bound
    args.ubx(2:n_states:n_states*(N+1),1) = safe_zone_max_y;  %state y upper bound
    
    safe_zone = [safe_zone; [safe_zone_min_x safe_zone_max_x safe_zone_min_y safe_zone_max_y]];


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

Draw_MPC_tracking (time,x_hist,predicted_x,u_cl,currXout,N,rob_diam, obs_diam,obs_x_static, obs_y_static,predicted_obs_poses ,t_mpc, goalsp, collision_points,map, safe_zone)