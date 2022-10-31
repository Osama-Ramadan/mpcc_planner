import casadi.*
clear; clc;
%% Define The Global Map
mapmat = load('Maps/omni_map1.mat');
mapmat = mapmat.imageOccupancy;
map = occupancyMap(mapmat,5);

%mapmat = load('Maps/map1.mat');
%mapmat = mapmat.mapmat;
%map = occupancyMap(mapmat,10);

%% Defining Configuration Parameters
config = struct;
config.dt = 0.2;                    %[s]  sampling time 
config.N = 20;                     % prediction horizon
config.N_obs = 30;
config.obs_diam = 1;
config.rob_diam = 1;
config.v_max = 0.5;                % Input constraints on both linear speed and steering speed
config.v_min = -config.v_max;
config.omega_max = pi/4; 
config.omega_min = -config.omega_max;
config.gamma_max = 30*(pi/180);
config.gamma_min = -30*(pi/180);
config.gamma_max_load = config.gamma_max;
%--- Load Paramters --- %
config.Ll = 0.5;
config.ml = 10;
config.hl = 1;

%% Define the Model
[f,n_states,n_controls,config] = define_model(config);

%% Spline Functions
[spline, heading, derivative] = define_spline();

%% Optimization Variables
opt = struct;
opt.U = SX.sym('U',n_controls,config.N);                                                   % Decision variables (controls)
opt.P = SX.sym('P',n_states + 8 + 2);                             % parameters (which include at the initial state of the robot and the reference state)
opt.X = SX.sym('X',n_states,(config.N+1));                                                 % A vector that represents the states over the optimization problem.
opt.f = f;
%% Setting the MPC Problem 
obj = 0;                                                                        % Objective function
opt.obj_close = 0;
g = [];   

st  = opt.X(:,1);                                                                   % initial state
g = [g;st-opt.P(1:n_states)];   


Q = diag([50, 1000]);
R = diag([1,100,1]);                                                             % weighing matrices (controls)
q_t = 300;
q_t_terminal = 10;    
q_th = 0.2;
% initial condition constraints
% Close to the path cost function -----------------------------------
for k = 1:config.N
    st = opt.X(:,k);  con = opt.U(:,k);
   
    if k == config.N
        delta_u = con;
    else
        con_next = opt.U(:,k+1);
        delta_u = con_next - con;
    end

    t_th = opt.X(9,k);
    s = spline(opt.P(n_states+1:n_states+4),opt.P(n_states+5:n_states+8),t_th);
    head = heading(opt.P(n_states+1:n_states+4),opt.P(n_states+5:n_states+8),t_th);

    e_c = sin(head)*(st(1)-s(1,1)) - cos(head)*(st(2)-s(2,1));
    e_l = -cos(head)*(st(1)-s(1,1)) - sin(head)*(st(2)-s(2,1));
    e_h =  sqrt((head - st(3))^2);

    e = [e_c ; e_l];
    opt.obj_close = opt.obj_close+ e'*Q*e + (delta_u)'*R*(delta_u) - q_t*t_th + q_th*e_h; % calculate obj
    if k==config.N
        dist_to_way = sqrt((st(1)-opt.P(end-1))^2+(st(2)-opt.P(end))^2);
        opt.obj_close = opt.obj_close + dist_to_way*q_t_terminal;
    end
end

g = define_dynamics(opt,config,g);
OPT_variables = [reshape(opt.X,n_states*(config.N+1),1);reshape(opt.U,n_controls*config.N,1)];        % make the decision variable one column  vector
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', opt.P);                % struct to define the optimization problem

opts = struct;
opts.ipopt.max_iter = 20;
opts.ipopt.print_level =0;
opts.print_time = 0;
%opts.ipopt.acceptable_tol =1e-8;
%opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);     % The actual solver

%% MPC Constraints
args = set_mpc_constraints(config,n_states,n_controls);

%% Initial Values
t0 = 0;
x0 = [10 ; 18 ; -pi/2 ; 0; 0 ; 0 ; 0 ; 0 ; 0.01; 10-config.D*cos(pi/2) ; 18-config.D*sin(pi/2)];            % initial state (maps).
t_0 = 0.01;

x_hist(:,1) = x0;                                      % contains the history of states
time(1) = t0;

u0 = zeros(config.N,n_controls);                       % control inputs for  robot
X0 = repmat(x0,1,config.N+1)';                         % initialization of the states decision variables

mpciter = 0;                                           % Number of iterations

predicted_x = [];                                      % The predicted solution from the controller
u_cl=[];                                               % Input control calculated from controller

t_mpc(1) = 0;                                          % Store times needed to solve the problem
safe_zone_f = [];
safe_zone_b = [];

%% Genetrate the Circular Trajectory
goalsp = [10 18 ; 10 10 ; 18,10]; % (warehouse)
%goalsp = [5 17 ; 25 20; 35 22; 35 32]; % (warehouse)
%goalsp = [3 10 ; 9 8 ; 18 1];
%goalsp = [3 10 ; 9 10 ; 18 13];
goalsp_orig = goalsp;
goalsp_log = goalsp;
%goalsp = [3 10 ; 12 3 ]; % (maps)
lookahead = 2;
temp_lookahead = lookahead;
currentInt = 1;
[Xout, wayps, Q_p] = Generate_Trajectory(goalsp,currentInt,lookahead);
currXout = [];
Orig_Xout = Xout;
%Xout_Adj =currXout;
n_trials = 5;
n_added_bubbles = 0;
indeces = [];
for ss = 1:length(wayps(:,1))
    indeces = [indeces ; dsearchn(Xout(:,(1:2)),wayps(ss,1:2))];
end
i=0;
seg = 1;
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
x0_obs = [16;26;0;0]; % map3
obs_v = [-0.12;-0.1];
x_dot_obs =[[0;0;0;0],...
            [0;0;0;0]]*obs_v;
valid_seg = false;

current_obs_pose = x0_obs;
opt.obj_close_st = opt.obj_close;
g_static = g;
lbg_st = args.lbg;
ubg_st = args.ubg;

%% Kalman Filter Initialization
    Pred_cov0 = [1,0,0,0;...
      0,1,0,0;...
      0,0,1,0;...
      0,0,0,1];
    [pred_x_N, estimated_x, estimated_p] = kalman_Filter(x0_obs,config.N+1,config.dt,x0_obs,Pred_cov0);

%% LOOP STARTING

trials = 0;
while(mpciter < 150) 
    current_time = mpciter*config.dt;                              % get the current time
    Xc = 0;                                                 % reset collision point
    Tc = -1;                                                % reset collision time
    g = g_static;                                           % reset constraints
    opt.obj_close = opt.obj_close_st;
    

    % Check for Load Stability
    theta_dd = 0.1;
    [gamma_max,acc_max] = load_stability(config,x0,theta_dd);
    config.gamma_max_load = abs(gamma_max);
    args = set_gamma_constraints(config,n_states,args);

    obs_position = current_obs_pose + config.dt*x_dot_obs;     % one forward step for the dynamic obstacle
    obs_poses(:,mpciter+1)= obs_position;
    [pred_x_N, estimated_x, estimated_p] = kalman_Filter(obs_position,config.N_obs+1,config.dt,estimated_x,estimated_p);
    predicted_obs_poses(:,:,mpciter+1)= pred_x_N;
    current_obs_pose = obs_position;


    
    if size(predicted_x,1) > 10                         % make sure one run has been done and we have some predicted states
        pred_x = predicted_x(10,1:2,mpciter)';         % take the last predicted state
    end

    i_pred = dsearchn(Xout(:,(1:2)),pred_x(1:2,:)');    % closest point index to the predicted state
    i = dsearchn(Xout(:,(1:2)),x0(1:2,:)');             % closest point index to the actual state
    i_org = i_pred;

% Update the Segment based on the distance to next waypoint -----
    for k = seg+1:length(indeces)                           % have a counter with the length of all waypoints
        waypoint_pose = Xout(indeces(k),:);
        dist_to_waypoint = sqrt((x0(1)-waypoint_pose(1))^2+((x0(2)-waypoint_pose(2))^2));
        if dist_to_waypoint < 1                           
            seg = seg+1; 
            if seg > length(indeces)-1                      % if you are in the last segment
                seg = length(indeces)-1;                    % keep being there
            end
        end
    end

% Check if the segment is valid ------
seg_start = seg;
%valid_seg = true;
while(valid_seg == false && trials < n_trials)
    Xout_len = size(Xout,1);
    for r = (seg_start-1)*100+1:min((seg_start+lookahead-1)*101,Xout_len)
        point_index = local2grid(map,[Xout(r,1), Xout(r,2)]);
        point = [Xout(r,1), Xout(r,2)];
        obs_l = search_near(point, 1, 1, map);
        if ~isempty(obs_l)
            trials = trials+1;
            valid_seg = false;
            %currXout = Xout_Adj;
            collision_t = mod(r,101)-1;
            [goalsp,path_changed] = adjust_path(collision_t,Q_p,map,goalsp,wayps,floor(r/101)+1,goalsp(end,:),derivative,spline,goalsp_orig);
            goalsp_log = [goalsp_log;goalsp];
            if path_changed == 1
                trials = 0;
                seg_start = seg_start+1;
                n_added_bubbles = n_added_bubbles+1;
                path_changed = 0;
                [Xout, wayps, Q_p] = Generate_Trajectory(goalsp,currentInt,lookahead+n_added_bubbles);
                %currXout = [currXout;Xout];
                indeces = [];
                for ss = 1:length(wayps(:,1))
                    indeces = [indeces ; dsearchn(Xout(:,(1:2)),wayps(ss,1:2))];
                end
            elseif path_changed == 2
                trials = 0;
                path_changed = 0;
                [Xout, wayps, Q_p] = Generate_Trajectory(goalsp,currentInt,lookahead+n_added_bubbles);
            end
            break;
        end
    end
    if r >= min(((seg_start+lookahead-1)*101)-2,Xout_len-2)
        valid_seg = true;
    end
end

%---------------------------------------------------------
    if valid_seg
        
    obj = opt.obj_close;
    i = i - (seg-1)*100;                                    % make sure i always between 0 and 1 for the MPC

    if i <= 0                                               % if we still on the prevoius segment keep tracking the first point on the new one 
        i = 1;
    end
    
    if sqrt((x0(1)-pred_x_N(1,1))^2+(x0(2)-pred_x_N(2,1))^2) < 6            % if the distance between the dynamic obst and the vehicle less than 6m
        
        [obj,collision_points,collision_time,dist_t_c] = handle_dyn_obstacle(x0,pred_x_N,Xout,seg,config,collision_points,collision_time,dist_t_c,opt,obj);
    else                                                                   
        g = g_static;
        args.lbg = lbg_st;
        args.ubg = ubg_st;

    end

    t_0 = i*0.01;                                                           % get path parameter from the index
    x0(9)= t_0;                                                             % update x0
    args.p(1:n_states) = x0;                                                % initial condition of the robot 
    

    Q_x_trans = Q_p(seg:seg+3,1);                                       % get the Qs for the current segment
    Q_y_trans = Q_p(seg:seg+3,2);
    args.p(n_states+1:n_states+4) = Q_x_trans;                              % update the paramters for MPC
    args.p(n_states+5:n_states+8) = Q_y_trans;
    
    else
                                                      % initial condition of the robot 
        obj = 0;
        Q = diag([20, 20]);                    % weighing matrices (states)
        Q_Terminal = diag([200,200]);                    % weighing matrices (states)
        R = diag([0.01,0.01]);                                                             % weighing matrices (controls)
        next_waypoint = Xout(seg*101,:)';

        for k = 1:config.N
            st = opt.X(1:2,k);  con = opt.U(1:2,k);
            if k < config.N
                obj = obj+(st-next_waypoint)'*Q*(st-next_waypoint) + con'*R*con; % calculate obj
            else
                obj = obj+(st-next_waypoint)'*Q_Terminal*(st-next_waypoint) + con'*R*con; % calculate obj
            end
        end

        x0(9)= 0.1;                                                             % update x0
        args.p(1:n_states) = x0;                                                % initial condition of the robot 
        args.p(n_states+1:n_states+4) = 0;                              % update the paramters for MPC
        args.p(n_states+5:n_states+8) = 0;

    end
    % Next waypoint 
    next_way = goalsp(seg+1,:);
    args.p(n_states+9) = next_way(1);
    args.p(n_states+10) = next_way(2);

    end_current_path_waypoint = currentInt+lookahead+n_added_bubbles;
    ecp_waypoint_pose = goalsp(end_current_path_waypoint,:);
    dist_to_ecp = sqrt((x0(1)-ecp_waypoint_pose(1))^2+(x0(2)-ecp_waypoint_pose(2))^2);
    if dist_to_ecp <= 1                           % update the trajectory if lookahead dist is completed
        currXout = [currXout; Xout];
        seg = 1;
        valid_seg = false;
        currentInt = currentInt+1+n_added_bubbles;
        n_added_bubbles = 0;
        if currentInt <= (length(goalsp(:,1))-lookahead)
            [Xout, wayps, Q_p] = Generate_Trajectory(goalsp,currentInt,lookahead);
            Orig_Xout = [Orig_Xout; Xout];
            goalsp = wayps;
            currentInt=1;
            indeces = [];
            for s = 1:length(wayps(:,1))
                indeces = [indeces ; dsearchn(Xout(:,(1:2)),wayps(s,1:2))];
            end
        else
            currentInt = length(goalsp(:,1))-lookahead;
        end
    end
    X_f = x0(1:3);
    X_b(1:2) = x0(10:11);
    X_b(3) = x0(3);
    boxes = 2;
    [safe_zone_min_x,safe_zone_max_x,safe_zone_min_y,safe_zone_max_y,safe_zone_f] = safe_area(config,X_f,map,predicted_x,safe_zone_f,mpciter,boxes);

    args.lbx(1:n_states:n_states*(2),1) = (safe_zone_min_x(1:2)); %state x lower bound
    args.ubx(1:n_states:n_states*(2),1) = (safe_zone_max_x(1:2));  %state x upper bound


    args.lbx(2:n_states:n_states*(2),1) = (safe_zone_min_y(1:2)); %state y lower bound
    args.ubx(2:n_states:n_states*(2),1) = (safe_zone_max_y(1:2));  %state y upper bound

    [safe_zone_min_x,safe_zone_max_x,safe_zone_min_y,safe_zone_max_y,safe_zone_b] = safe_area(config,X_b,map,predicted_x,safe_zone_b,mpciter, boxes);

    args.lbx(10:n_states:n_states*(2),1) = (safe_zone_min_x(1:2)); %state x lower bound
    args.ubx(10:n_states:n_states*(2),1) = (safe_zone_max_x(1:2));  %state x upper bound


    args.lbx(11:n_states:n_states*(2),1) = (safe_zone_min_y(1:2)); %state y lower bound
    args.ubx(11:n_states:n_states*(2),1) = (safe_zone_max_y(1:2));  %state y upper bound

    nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', opt.P);                % struct to define the optimization problem
    solver = nlpsol('solver', 'ipopt', nlp_prob,opts);     % The actual solver

    % initial value of the optimization variables
    args.x0  = [reshape(X0',n_states*(config.N+1),1);reshape(u0',n_controls*config.N,1)];
    
    tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...    % The problem is solver here ..                                 
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    t_mpc(mpciter+1) = toc; 
    
    objec = [objec ; sol.f];

                                                    % extract controls only from the solution
    u = reshape(full(sol.x(n_states*(config.N+1)+1:n_states*(config.N+1)+n_controls*config.N))',n_controls,config.N)'; 
                                                    % extract the predicted TRAJECTORY
    x_pred = reshape(full(sol.x(1:n_states*(config.N+1)))',n_states,config.N+1)';
    predicted_x(:,1:n_states,mpciter+1)= reshape(full(sol.x(1:n_states*(config.N+1)))',n_states,config.N+1)'; 

    u_cl= [u_cl ; u(1,:)];                                   % get first control input
    time(mpciter+1) = t0;
    [x0, u0] = dynamic_red_shift(config.dt, x0, u,f,config.D);           % Apply control input and get the next initial state from the dynamics
    t0 = t0+config.dt;

    x_hist(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:n_states*(config.N+1)))',n_states,config.N+1)';             % extract the predicted TRAJECTORY
    X0 = [X0(2:end,:);X0(end,:)];                                           % Shift trajectory to initialize the next step

    mpciter = mpciter + 1;

end
%% Plotting and Data logging 

fprintf('Mean solver time :    %g [ms]\n', mean(t_mpc)*1e3);
fprintf('Median solver time :  %g [ms]\n', median(t_mpc)*1e3);
fprintf('Max solver time :     %g [ms]\n', max(t_mpc)*1e3);

Draw_MPC_tracking (time,x_hist,predicted_x,u_cl,currXout,Orig_Xout,config.N,config.rob_diam, config.obs_diam,predicted_obs_poses ,t_mpc, goalsp_log, collision_points,map, safe_zone_f,safe_zone_b, Q_p)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define the Model 
function [f,n_states,n_controls,config] = define_model(config)
import casadi.*
config.mB= 10;                     % mass of the chassis
config.Lb= 0;                      % distance from the center of the wheels to the center of mass of chassis
config.hb = 0.5;
config.w = 1;
config.D= 2.5;                     % half of wheel-to-wheel distance 
config.IB=0.5*config.mB*config.D^2+20;              % moment of inertia of the chassis
config.IT=config.IB+config.mB*config.Lb^2;

x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta'); gamma= SX.sym('gamma'); w= SX.sym('w'); x_b = SX.sym('x_b'); y_b = SX.sym('y_b');
x_dot = SX.sym('x_dot'); y_dot = SX.sym('y_dot'); theta_dot = SX.sym('theta_dot'); gamma_dot= SX.sym('gamma_dot');
tau = SX.sym('tau'); omega = SX.sym('omega'); wd = SX.sym('wd');

q = [x; y; theta;gamma];                        % system states
q_dot = [x_dot; y_dot; theta_dot; gamma_dot];       % system states


states = [x;y;theta;gamma;x_dot;y_dot;theta_dot; gamma_dot; w; x_b ; y_b];
n_states = length(states);

%Torque = [tau*cos(theta+gamma);tau*sin(theta+gamma);-tau*sin(gamma)*(config.D);omega];           % system control inputs
Torque = [tau*cos(theta+gamma);tau*sin(theta+gamma);0;omega];           % system control inputs
Input = [tau;omega;wd];
n_controls = length(Input);

M=[config.mB                   0            config.mB*config.Lb*sin(theta)      0   ;...
   0                    config.mB          -config.mB*config.Lb*cos(theta)      0   ;...
   config.mB*config.Lb*sin(theta) -config.mB*config.Lb*cos(theta)         config.IT            0   ;...
   0                    0               0                 1  ];

B= config.mB*config.Lb*theta_dot^2*[cos(theta);sin(theta);0;0];

C=[sin(theta+gamma)   sin(theta) ; ...
  -cos(theta+gamma)  -cos(theta) ; ...
   (config.D)*cos(gamma)          0   ; ...
        0                  0     ];

C = C';

Cdot=[cos(theta+gamma)*(theta_dot+gamma_dot)       cos(theta)*theta_dot ; ...
      sin(theta+gamma)*(theta_dot+gamma_dot)       sin(theta)*theta_dot ; ...
      -(config.D)*sin(gamma)*gamma_dot                             0         ; ...
                 0                                            0         ];


Cdot  = Cdot';

lambdas=-inv(C*inv(M)*C')*(C*inv(M)*(Torque-B)+Cdot*q_dot);
q_dd= inv(M)*(Torque-B+C'*lambdas);

f = Function('f',{states,Input},...
    {q_dd},...
    {'states','input'},{'q_dd'});
end

%% Define Spline Functions
function [spline, heading, derivative] = define_spline()
import casadi.*
Qx = SX.sym('Qx',4,1);
Qy = SX.sym('Qy',4,1);
t = SX.sym('t');
x_t= (1/6)*(((1-t).^3*Qx(1)) + (3*t.^3-6*t.^2+4)*Qx(2) + (-3*t.^3+3*t.^2+3*t+1)*Qx(3) + t.^3*Qx(4));
y_t= (1/6)*(((1-t).^3*Qy(1)) + (3*t.^3-6*t.^2+4)*Qy(2) + (-3*t.^3+3*t.^2+3*t+1)*Qy(3) + t.^3*Qy(4));
xd = [x_t ; y_t];
spline = Function('s',{Qx,Qy,t},{xd}, {'Qx','Qy', 't'},{'xd'});
     
dx_t = (1/6)*((-3*(1-t).^2*Qx(1))+((9*t.^2-12*t)*Qx(2))+((-9*t.^2+6*t+3)*Qx(3))+(3*t.^2*Qx(4)));
dy_t = (1/6)*((-3*(1-t).^2*Qy(1))+((9*t.^2-12*t)*Qy(2))+((-9*t.^2+6*t+3)*Qy(3))+(3*t.^2*Qy(4)));
ds_t = [dx_t;dy_t];
theta_d = atan2(dy_t,dx_t);
heading = Function('head', {Qx,Qy,t},{theta_d},{'Qx','Qy', 't'},{'theta_d'});
derivative = Function('derv', {Qx,Qy,t},{ds_t});

end

%% Set MPC Constraints
function args = set_mpc_constraints(config,n_states,n_controls)
args = struct;
                % These constraints make sure that the states will follow the dynamics
args.lbg(1:n_states*(config.N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:n_states*(config.N+1)) = 0;  % 1e-20   % Equality constraints
         
% Constraints over the states 
args.lbx(1:n_states:n_states*(config.N+1),1) = -100; %state x lower bound
args.ubx(1:n_states:n_states*(config.N+1),1) = 100;  %state x upper bound

args.lbx(2:n_states:n_states*(config.N+1),1) = -100; %state y lower bound
args.ubx(2:n_states:n_states*(config.N+1),1) = 100;  %state y upper bound

args.lbx(3:n_states:n_states*(config.N+1),1) = -30; %state theta lower bound
args.ubx(3:n_states:n_states*(config.N+1),1) =  30;  %state theta upper bound

args.lbx(4:n_states:n_states*(config.N+1),1) = config.gamma_min; %state gamma lower bound
args.ubx(4:n_states:n_states*(config.N+1),1) = config.gamma_max;  %state gamma upper bound

args.lbx(5:n_states:n_states*(config.N+1),1) = config.v_min; %state xdot lower bound
args.ubx(5:n_states:n_states*(config.N+1),1) = config.v_max;  %state xdot upper bound

args.lbx(6:n_states:n_states*(config.N+1),1) = config.v_min; %state ydot lower bound
args.ubx(6:n_states:n_states*(config.N+1),1) = config.v_max;  %state ydot upper bound

args.lbx(7:n_states:n_states*(config.N+1),1) = -30; %state thetad lower bound
args.ubx(7:n_states:n_states*(config.N+1),1) = 30;  %state thetad upper bound

args.lbx(8:n_states:n_states*(config.N+1),1) = -1.5; %state gamma_dot lower bound
args.ubx(8:n_states:n_states*(config.N+1),1) = 1.5;  %state gamma_dot upper bound

args.lbx(9:n_states:n_states*(config.N+1),1) = 0; %state gamma_dot lower bound
args.ubx(9:n_states:n_states*(config.N+1),1) = 1;  %state gamma_dot upper bound

args.lbx(10:n_states:n_states*(config.N+1),1) = -100; %state x_b lower bound
args.ubx(10:n_states:n_states*(config.N+1),1) = 100;  %state x_b upper bound

args.lbx(11:n_states:n_states*(config.N+1),1) = -100; %state y_b lower bound
args.ubx(11:n_states:n_states*(config.N+1),1) = 100;  %state y_b upper bound

args.lbx(n_states*(config.N+1)+1:n_controls:n_states*(config.N+1)+n_controls*config.N,1) = -10; %v lower bound
args.ubx(n_states*(config.N+1)+1:n_controls:n_states*(config.N+1)+n_controls*config.N,1) = 10; %v upper bound

args.lbx(n_states*(config.N+1)+2:n_controls:n_states*(config.N+1)+n_controls*config.N,1) = -1.5; %omega lower bound
args.ubx(n_states*(config.N+1)+2:n_controls:n_states*(config.N+1)+n_controls*config.N,1) = 1.5; %omega upper bound

args.lbx(n_states*(config.N+1)+3:n_controls:n_states*(config.N+1)+n_controls*config.N,1) = -0.001; %omega lower bound
args.ubx(n_states*(config.N+1)+3:n_controls:n_states*(config.N+1)+n_controls*config.N,1) = 0.01; %omega upper bound
end

%% Set the Dynamic Equations
function g = define_dynamics(opt,config,g)
% Dynamic equation constraints ------------------------------------------
for k = 1:config.N
    st = opt.X(:,k);  con = opt.U(:,k);
    st_next_euler = st;
    st_next = opt.X(:,k+1);                 % get the next state symbols from the Big vector X
    x_dd = opt.f(st,con);                   % get the dynamics expressed in the states st
    x_d = st(5:length(st)-4) + (config.dt*x_dd(1:3));          % get the next state by  euler method
    st_next_euler(5:length(st_next_euler)-4) = x_d(1:3);
    st_next_euler(length(st_next_euler)-3) = x_dd(4);
    x = st(1:3) + (config.dt*x_d);
    st_next_euler(1:3)= x;
    st_next_euler(4) = st(4) + (config.dt*x_dd(4));
    st_next_euler(9) = st(9) + con(3);
    st_next_euler(10) = st_next_euler(1)-config.D*cos(st_next_euler(3));
    st_next_euler(11) = st_next_euler(2)-config.D*sin(st_next_euler(3));
    g = [g;st_next-st_next_euler];      % add the difference between the state obtained from the dynamics and the decision variable to the constraints
end
end

%% Construct Vehicle Safe Area
function [safe_zone_min_x,safe_zone_max_x,safe_zone_min_y,safe_zone_max_y,safe_zone] = safe_area(config,x0,map,predicted_x,safe_zone,mpciter,n_boxes)
    max_search_x = 2;
    max_search_y = 3;
    safe_zone_boarders = [];
    safe_zone_min_x = [];
    safe_zone_min_y = [];
    safe_zone_max_x = [];
    safe_zone_max_y = [];
    mapmat = round(occupancyMatrix(map));
    
    for k = 1:n_boxes
    if ~isempty(predicted_x)

    predicted_x_s = predicted_x(k,:,mpciter);

    robot_curr_x = predicted_x_s(1);
    robot_curr_y = predicted_x_s(2);
    robot_curr_th = predicted_x_s(3);
    
    direction = [cos(robot_curr_th),sin(robot_curr_th)];
    x_search_ext = direction(1)*2;
    y_search_ext = direction(2)*2;

    max_search_x = 1 + abs(x_search_ext);
    max_search_y = 1 + abs(y_search_ext);


    index_in_map = local2grid(map,[robot_curr_x, robot_curr_y]);
    else
        robot_curr_x = x0(1);
        robot_curr_y = x0(2);
        robot_curr_th = x0(3);
        index_in_map = local2grid(map,[robot_curr_x, robot_curr_y]);
    end

    min_y_search_index = max(index_in_map(1)-round(max_search_y*map.Resolution),1);
    max_y_search_index = min(index_in_map(1)+round(max_search_y*map.Resolution),size(mapmat,1));
    min_x_search_index = max(index_in_map(2)-round(max_search_x*map.Resolution),1);
    max_x_search_index = min(index_in_map(2)+round(max_search_x*map.Resolution),size(mapmat,2));

    searchmatrix = mapmat(min_y_search_index:max_y_search_index,...
                          min_x_search_index:max_x_search_index);
    
    search_area_min_x = index_in_map(2)-round(max_search_x*map.Resolution);
    search_area_max_y = index_in_map(1)-round(max_search_y*map.Resolution);

    %[leftcorner, rightcorner] = getMaxSearchArea1(searchmatrix,max_search_x*map.Resolution,max_search_y*map.Resolution);
    [leftcorner, rightcorner] = getMaxSearchArea(searchmatrix);
    Upper_left_corner_local = grid2local(map,[search_area_max_y+leftcorner(1)-1,search_area_min_x+leftcorner(2)-1]);
    Lower_right_corner_local = grid2local(map,[search_area_max_y+rightcorner(1)-1,search_area_min_x+rightcorner(2)-1]);
    
    %Upper_left_corner_local = grid2local(map,[search_area_max_y+leftcorner(2),search_area_min_x+leftcorner(1)]);
    %Lower_right_corner_local = grid2local(map,[search_area_max_y+rightcorner(2),search_area_min_x+rightcorner(1)]);
    


    safe_zone_min_x1 = Upper_left_corner_local(1)+0.2;
    safe_zone_max_x1 = Lower_right_corner_local(1)-0.2;
    safe_zone_min_y1 = Lower_right_corner_local(2)+0.2;
    safe_zone_max_y1 = Upper_left_corner_local(2)-0.2; 

    safe_zone_min_x = [safe_zone_min_x ;safe_zone_min_x1];
    safe_zone_min_y = [safe_zone_min_y ;safe_zone_min_y1];
    safe_zone_max_x = [safe_zone_max_x ;safe_zone_max_x1];
    safe_zone_max_y = [safe_zone_max_y ;safe_zone_max_y1];
    
    safe_zone = [safe_zone; [safe_zone_min_x1 safe_zone_max_x1 safe_zone_min_y1 safe_zone_max_y1]];
    
    end
end

%% Function to Handle the Dynamic Obstacle
function [obj,collision_points,collision_time,dist_t_c] = handle_dyn_obstacle(x0,pred_x_N,Xout,seg,config,collision_points,collision_time,dist_t_c,opt,obj)
    [Xc,Tc] = get_collision_point(x0,pred_x_N,1,Xout,seg,config.dt);       % calculate the collision point (with the path) and time
            if Xc~=1000                                                     % check if a collision point is found 
                collision_points = [collision_points , Xc];
                dist_to_coll = sqrt((x0(1)-Xc(1))^2+(x0(2)-Xc(2))^2);       % calculate distance to collision 
            end
            if Tc~=-1
                collision_time = [collision_time , Tc(1)-Tc(2)];            % calclate if a valid time to collision is found
            end

            if (Tc(1)~=-1 && abs(Tc(1)-Tc(2))<10)||(Tc(1)~=-1 && dist_to_coll<=dist_t_c)     % if the time to collision is less than 3s or dis to coll decreases
                x_c = Xc(1);
                y_c = Xc(2);
                dist_t_c = sqrt((x0(1)-x_c)^2+(x0(2)-y_c)^2);                               % update distance to collision
                for k = 1:10
                    st = opt.X(:,k);
                    for j = 1:10
                    o_st = pred_x_N(:,j);
                    dist_v = sqrt((st(1)-x_c)^2+(st(2)-y_c)^2);             % distance between vehicle and collision point
                    v_v = sqrt(st(5)^2+st(6)^2);                            % speed of vehicle
                    delta_t = abs((dist_v)/(v_v + 0.1));       % difference of collision time between vehicle and obstacle
                    obj = obj + 100/(delta_t+0.001) + (100/(dist_v+0.001));                     % update cost function
                    end
                end
            end
end

%% Adjusting the Path
function [goalsp, path_changed] = adjust_path(collision_t,Q_p,map,goalsp,wayps,seg,goal,derivative,spline,goals_orig)
import casadi.*
p_x = SX.sym('p_x');
p_y = SX.sym('p_y');

Q_p = Q_p(seg:seg+3,:);
p = full(spline(Q_p(:,1),Q_p(:,2),collision_t*0.01));
step_size = 0.001;
p_obst_poses = 1;
path_changed = 1;
waypoint_updated = [];
delta_p = 0;
counter = 0;
while(~isempty(p_obst_poses))
counter = counter+1;
% obstalces around points
p_obst_poses = search_near(p,1.5,1.5,map);
if(isempty(p_obst_poses))
break;
end
Potential=0;
% Obstalce forces
force_gain_obst = 0.5;
force_gain_goal = 0.0;


% calculate goal force
d_x_g = p_x-goal(1);
d_y_g = p_y-goal(2);
Potential = Potential + d_x_g*force_gain_goal ;
Potential = Potential + d_y_g*force_gain_goal ;

% calculate obstacles force
for i = 1:size(p_obst_poses,1)
    obst = p_obst_poses(i,:);
    dist_o = sqrt((p(1)-obst(1))^2+(p(2)-obst(2))^2);
    d_o = sqrt((p_x - obst(1))^2+(p_y - obst(2))^2);
    Potential = Potential + (force_gain_obst/(d_o+0.001));
end

% update bubble point position
force = -gradient(Potential,[p_x,p_y])';
d_force = jacobian(force,[p_x,p_y])';
force_f = Function('force', {p_x,p_y},{force},{'p_x','p_y'},{'force'});
d_force_f = Function('d_force', {p_x,p_y},{d_force},{'p_x','p_y'},{'d_force'});
force_val = full(force_f(p(1),p(2)));
force_val_mag = sqrt(force_val(1)^2+force_val(2)^2);
d_force_val = full(d_force_f(p(1),p(2)));

d_s = full(derivative(Q_p(:,1),Q_p(:,2),collision_t*0.01));
d_ss = d_s;
d_s(1) = d_s(1)/sqrt(d_ss(1)^2+d_ss(2)^2);
d_s(2) = d_s(2)/sqrt(d_ss(1)^2+d_ss(2)^2);

M = [d_force_val(1),         0,        -d_s(1) ;...
            0,        d_force_val(4),  -d_s(2); ...
          d_s(1)   ,       d_s(2) ,      0];

F = [-force_val(1); -force_val(2); 0];
delta = inv(M)*F;
dx = delta(1)/sqrt(delta(1)^2+delta(2)^2);
dy = delta(2)/sqrt(delta(1)^2+delta(2)^2);
delta_x = force_val_mag*dx*step_size;
delta_y = force_val_mag*dy*step_size;
if(abs(delta_x) <= 0.001 && abs(delta_y)<=0.001)
    break;
end
p(1) = p(1)+delta_x;
p(2) = p(2)+delta_y;
end

current_seg_p = wayps(seg,:);

dist_p = sqrt((p(1)-goal(1))^2+(p(2)-goal(2))^2);
dist_seg = sqrt((current_seg_p(1)-goal(1))^2+(current_seg_p(2)-goal(2))^2);
global_seg = seg;
waypoint_updated(1:global_seg,:) = goalsp(1:global_seg,:);
wpoint = goalsp(global_seg,:);
wp_dist = hypot(p(1)-wpoint(1),p(2)-wpoint(2));
ingoals = ismember(goals_orig,wpoint,"rows");
if any(ismember(goals_orig,wpoint,"rows"))
    wp_dist = 5;
end
if wp_dist < 3
    path_changed = 2;
    waypoint_updated(global_seg,:) = p';
else
    waypoint_updated(global_seg+1,:) = p';
end
waypoint_updated = [waypoint_updated ; goalsp(global_seg+1:end,:)];
goalsp = waypoint_updated;
end

%% Search Neighbour function
function obst_list = search_near(point, x_search, y_search, map)
obst_list = [];
mapmat = round(getOccupancy(map));
point = local2grid(map,[point(1), point(2)]);
for i = round(point(2)-x_search*map.Resolution):round(point(2)+x_search*map.Resolution)           % Search the whole Area
    for j = round(point(1)-y_search*map.Resolution):round(point(1)+y_search*map.Resolution)
        if mapmat(j,i) == 1                                                                     % if you found an obstacle
            obst_pose = grid2local(map,[j,i]);
            obst_list = [obst_list ; obst_pose];
        end
    end
end
end

%% Get Max steering for Load Stability
function [gamma_max,acc_max] = load_stability(config,x0,theta_dd)

% check for very small velocity
theta = x0(3);
x_d = [cos(theta), sin(theta) ; -sin(theta), cos(theta)]*[x0(5) ; x0(6)];
vR = x_d(1);
if abs(vR) < 0.05
    acc_max = 1000;
    gamma_max = config.gamma_max;
    return;
end

% Longitodinal Stability ----
g = 9.81;
Ml = (config.Lb*config.mB+config.Ll*config.ml)*g;

acc_max = abs((Ml-x0(7)^2*(config.hl*config.ml*config.Ll+config.hb*config.mB*config.Lb))/(config.hl*config.ml+config.hb*config.mB));

% Lateral Stability ----
Db = config.w/2*((config.D-config.Lb)/config.D);
Dl = config.w/2*((config.D-config.Ll)/config.D);
Mt = (Db*config.mB+Dl*config.ml)*g;

direction = sign(x0(7)/vR);
theta_dd = -direction*theta_dd;
k_max = (Mt + direction*(theta_dd*(config.hl*config.ml*config.Ll+config.hb*config.mB*config.Lb)))/(vR^2*(config.hl*config.ml+config.hb*config.mB));
gamma_max = atan2(k_max*config.D,1)*(180/pi);
end

%% Set Steering Constraints from Load
function args = set_gamma_constraints(config,n_states,args)
args.lbx(4:n_states:n_states*(config.N+1),1) = -config.gamma_max_load; %state gamma lower bound
args.ubx(4:n_states:n_states*(config.N+1),1) = config.gamma_max_load;  %state gamma upper bound
end

%% Set Steering For narrow corridors
function args = set_steering_limits(config,n_states,args,safe_area,states)
    x = states(1);
    y = states(2);
    theta = states(3);
end