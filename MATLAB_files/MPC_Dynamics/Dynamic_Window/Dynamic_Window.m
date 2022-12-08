%%%% Implementation of the dynamic window approach for the forklift %%%%%%
clc; clear all;
%% Map Loading
mapmat = load('Maps/warehouse_map.mat');
mapmat = mapmat.imageOccupancy;
map = occupancyMap(mapmat,5);

%% Configurations
config = struct;
config.max_speed = 1;                        % (m/s)
config.min_speed = -0.5;                       % (m/s)
config.max_omega = 80*(pi/180);                % (rad/s)
config.max_acceleration = 1;                   % (m/s^2)
config.max_alpha = 20*(pi/180);                % (rad/s^2)
config.v_resolution = 0.1;                    % (m/s)
config.omega_resolution = 1*(pi/180);        % (rad/s)
config.dt = 0.1;                               % (s)
config.prediction_horizon = 4;                 % (s)
config.N = config.prediction_horizon/config.dt;
config.N_obs = 30;
config.heading_cost_gain = 1;
config.to_goal_cost_gain = 3;
config.speed_cost_gain = 5;
config.obstacle_cost_gain = 5;
config.dynamic_obstacle_cost_gain = 0.001;
config.stuck_flag_constant = 0.001;
config.robot_radius = 0.5;                       % (m)
config.obstacle_raduis = 0.2;
config.D = 1.8;                                % (m)

%% Generate Trajectory
goalsp = [37 3 ; 37 18; 20 24; 7 14 ]; % (warehouse)
lookahead = 3;
currentInt = 1;
[Xout, wayps, Q_p] = Generate_Trajectory(goalsp,currentInt,lookahead);

%% Dynamic Obstacles
obs_poses = [];
predicted_obs_poses = [];
x_pred = [];
x0_obs = [40;25;0;0]; % map3
obs_v = [-0.08;-0.08];
x_dot_obs =[[1;0;0;0],...
            [0;1;0;0]]*obs_v;

current_obs_pose = x0_obs;


%% Kalman Filter Initialization
    Pred_cov0 = [1,0,0,0;...
      0,1,0,0;...
      0,0,1,0;...
      0,0,0,1];
    [pred_x_N, estimated_x, estimated_p] = kalman_Filter(x0_obs,config.N+1,config.dt,x0_obs,Pred_cov0);
%% Main Loop
x = [37;2;pi/2;0];
u = [0;0];
trajectory_f = [x];
obst = [];
pred_traj = [];
u_f = [];
sim_steps = 0;
current_waypoint = 1;


sim_total_steps = 600;
while sim_steps < sim_total_steps

    % Update Dynamic Obstacle Pose
    obs_position = current_obs_pose + config.dt*x_dot_obs;     % one forward step for the dynamic obstacle
    obs_poses(:,sim_steps+1)= obs_position;
    [pred_x_N, estimated_x, estimated_p] = kalman_Filter(obs_position,config.N_obs+1,config.dt,estimated_x,estimated_p);
    predicted_obs_poses(:,:,sim_steps+1)= pred_x_N;
    current_obs_pose = obs_position;

    i = dsearchn(Xout(:,(1:2)),x(1:2,:)');             % closest point index to the actual state
    goal = Xout(i+10,:);
    
    [u,trajectory] = dwa_control(x,u,config,goal,map,pred_x_N);
    x = forward_step(x,u,config);
    trajectory_f = [trajectory_f, x];
    pred_traj(:,:,sim_steps+1) = trajectory;
    u_f = [u_f , u];
    
    % check for reaching goal
    dist_to_goal = hypot(x(1)-goalsp(end,1), x(2)-goalsp(end,2));
    if dist_to_goal<=config.robot_radius*4
        if current_waypoint == size(goalsp,1)
        disp('Reached the goal !!!');
        break;
        else
            current_waypoint = current_waypoint+1;
        end
    end
    sim_steps = sim_steps+1;
    end
    
    Draw_dwa (trajectory_f,u_f,pred_traj,config.prediction_horizon/config.dt,goalsp,2*config.robot_radius,2*config.obstacle_raduis,map,Xout,predicted_obs_poses);

%% Equation of Motion

function x_next = forward_step(x,u,config)
x_next(1) = x(1) + cos(x(4))*cos(x(3))*u(1)*config.dt;
x_next(2) = x(2) + cos(x(4))*sin(x(3))*u(1)*config.dt;
x_next(3) = x(3) + (-sin(x(4))/config.D)*config.dt;
x_next(4) = x(4) + u(2)*config.dt;
x_next = [x_next(1);x_next(2);x_next(3);x_next(4)];
end

%% Calculate Dynamic Window
function Dw = calc_dynamic_window(u,config)
% set of possible speeds
Vs = [config.min_speed, config.max_speed, -config.max_omega, config.max_omega];

%speed reached in next step
Vd = [u(1)-config.max_acceleration*config.dt, u(1)+config.max_acceleration*config.dt, u(2)-config.max_alpha*config.dt, u(2)+config.max_alpha*config.dt];

%dynamic window 
Dw = [max(Vs(1), Vd(1)),min(Vs(2), Vd(2)),max(Vs(3), Vd(3)),min(Vs(4), Vd(4))];
end

%% Predicted Trajectory
function traj = predict_trajectory(x0,u,config)
traj = [x0];
t = 0;
x = x0;
while t <= config.prediction_horizon
    x = forward_step(x,u,config);
    traj = [traj,x];
    t = t+config.dt;
end
end

%% Calculate Obstacle Cost
function obst_cost_value = calc_obst_cost(trajectory,config,map)
    obst_cost_value = 0;

    for i = 1:size(trajectory,2)
        % Compute Static Obstacle Cost
        traj_pt = trajectory(:,i);
        obst_list = search_near(traj_pt,1.5,1.5,map);
        if isempty(obst_list)
            return
        end
        ox = obst_list(:,1);
        oy = obst_list(:,2);
        dx = traj_pt(1) - ox;
        dy = traj_pt(2) - oy;
        r = hypot(dx,dy);
        r_min = min(r);
        
        if r_min <=(config.robot_radius+config.obstacle_raduis)
            obst_cost_value = inf;
            return;
        end

        obst_cost_value = obst_cost_value + 1/r_min;

    end


end

function cost = calc_dynamic_obst_cost(trajectory,obst_traj)
    cost = 0;
    for i = 1:size(trajectory,2)
        traj_pt = trajectory(:,i);
        dx = traj_pt(1) - obst_traj(1,1);
        dy = traj_pt(2) - obst_traj(2,1);
        r = hypot(dx,dy);
        % if the trajectory point is m distance from the dynamic obstalce
        % trajectory igonre it
        if r > 5
            cost = 0;
            return
        end
        for j = 1:size(obst_traj,2)
            dx = traj_pt(1) - obst_traj(1,j);
            dy = traj_pt(2) - obst_traj(2,j);
            r = hypot(dx,dy);
            cost = cost + ((size(obst_traj,2)-j)/(r+0.001));
        end
        
    end
end
%% Calculate to Goal Cost
function to_goal_cost = calc_to_goal_cost(trajectory,goal)
dx = goal(1) - trajectory(1,end);
dy = goal(2) - trajectory(2,end);
to_goal_cost = hypot(dy,dx);
end
%% Calculate To heading Cost
function cost = calc_heading_cost(trajectory,goal)
dx = goal(1) - trajectory(1,end);
dy = goal(2) - trajectory(2,end);
error_angle = atan2(dy,dx);
cost_angle = error_angle - trajectory(3,end);
cost = abs(atan2(sin(cost_angle),cos(cost_angle)));
end

%% Caluclate Optimal Control and Trajectory
function [opt_u,opt_traj] = calc_control_traj(x,Dw,config,goal,map,pred_x_N)
x_init = x;
min_cost = inf;
opt_u = [0;0];
opt_traj = [x];

% start evaluating velocities from the window
for v = Dw(1):config.v_resolution:Dw(2)
    for omega = Dw(3):config.omega_resolution:Dw(4)
        trajectory = predict_trajectory(x_init,[v,omega],config);
        % calculate costs
        heading_cost = config.heading_cost_gain*calc_heading_cost(trajectory,goal);
        speed_cost = config.speed_cost_gain*(config.max_speed-v);
        obst_cost = config.obstacle_cost_gain*calc_obst_cost(trajectory,config,map);
        dynamic_obst_cost = config.dynamic_obstacle_cost_gain*calc_dynamic_obst_cost(trajectory,pred_x_N);
        to_goal_cost = config.to_goal_cost_gain*calc_to_goal_cost(trajectory,goal);
        final_cost = heading_cost + speed_cost + obst_cost + to_goal_cost + dynamic_obst_cost;

        %Look for the minimum trajectory
        if min_cost >= final_cost
            min_cost = final_cost;
            opt_u = [v;omega];
            opt_traj = trajectory;
            if abs(opt_u(1)) < config.stuck_flag_constant && abs(opt_u(2)) < config.stuck_flag_constant
                opt_u(2) = -config.max_alpha;
            end
        end
    end
end
    if min_cost == inf
        disp("Could not find a solution");
        opt_u = [0;0];
    end
end

%% Search Neighbour function
function obst_list = search_near(point, x_search, y_search, map)
obst_list = [];
mapmat = round(getOccupancy(map));
point = local2grid(map,[point(1), point(2)]);
for i = max(round(point(2)-x_search*map.Resolution),1):min(round(point(2)+x_search*map.Resolution),size(mapmat,2))           % Search the whole Area
    for j = max(round(point(1)-y_search*map.Resolution),1):min(round(point(1)+y_search*map.Resolution),size(mapmat,1))
        if mapmat(j,i) ~= 0                                                                     % if you found an obstacle
            obst_pose = grid2local(map,[j,i]);
            obst_list = [obst_list ; obst_pose];
        end
    end
end
end

%% General Control Function
function [u, trajectory] = dwa_control(x,u,config,goal,map,pred_x_N)
    Dw = calc_dynamic_window(u,config);
    [u,trajectory] = calc_control_traj(x,Dw,config,goal,map,pred_x_N);
end

%% Adjusting the Path
function [goalsp, path_changed] = adjust_path(collision_t,Q_p,map,goalsp,wayps,seg,goal,derivative,spline)
import casadi.*
p_x = SX.sym('p_x');
p_y = SX.sym('p_y');
Potential = SX.sym('pot');

Q_p = Q_p(seg:seg+3,:);
p = full(spline(Q_p(:,1),Q_p(:,2),collision_t*0.01));
step_size = 0.001;
p_obst_poses = 1;
path_changed = 1;
waypoint_updated = [];
while(~isempty(p_obst_poses))
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
d_s(1) = d_s(1)/sqrt(d_s(1)^2+d_s(2)^2);
d_s(2) = d_s(2)/sqrt(d_s(1)^2+d_s(2)^2);

M = [d_force_val(1),        d_force_val(2),      -d_s(1) ;...
     d_force_val(3),        d_force_val(4),      -d_s(2); ...
          d_s(1)   ,            d_s(2) ,          0];
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
if wp_dist < 3
    path_changed = 2;
    waypoint_updated(global_seg,:) = p';
else
    waypoint_updated(global_seg+1,:) = p';
end
waypoint_updated = [waypoint_updated ; goalsp(global_seg+1:end,:)];
goalsp = waypoint_updated;
end