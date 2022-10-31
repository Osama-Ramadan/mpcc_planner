%%%% Implementation of the dynamic window approach for the forklift %%%%%%
clc; clear all;
%% Map Loading
mapmat = load('Maps/map3.mat');
mapmat = mapmat.mapmat;
map = binaryOccupancyMap(mapmat,10);

%% Configurations
config = struct;
config.max_speed = 1.0;                        % (m/s)
config.min_speed = -0.5;                       % (m/s)
config.max_omega = 40*(pi/180);                % (rad/s)
config.max_acceleration = 0.2;                 % (m/s^2)
config.max_alpha = 40*(pi/180);                % (rad/s^2)
config.v_resolution = 0.01;                    % (m/s)
config.omega_resolution = 0.1*(pi/180);        % (rad/s)
config.dt = 0.1;                               % (s)
config.prediction_horizon = 3;                 % (s)
config.to_goal_cost_gain = 1.15;
config.speed_cost_gain = 1;
config.obstacle_cost_gain = 1;
config.stuck_flag_constant = 0.001;
config.robot_radius = 1;                       % (m)
config.obstacle_raduis = 1;
config.D = 1;                                % (m)

%% Main Loop
x = [0;0;0;0];
u = [0;0];
goals = [10;10];
obst = [100,100];
trajectory_f = [x];
pred_traj = [];
u_f = [];
sim_steps = 0;
while sim_steps < 400
[u,trajectory] = dwa_control(x,u,config,goals,obst);
x = forward_step(x,u,config);
trajectory_f = [trajectory_f, x];
pred_traj(:,:,sim_steps+1) = trajectory;
u_f = [u_f , u];

% check for reaching goal
dist_to_goal = hypot(x(1)-goals(1), x(2)-goals(2));
if dist_to_goal<=config.robot_radius
    disp('Reached the goal !!!');
    break;
end
sim_steps = sim_steps+1;
end

Draw_dwa (trajectory_f,pred_traj,config.prediction_horizon/config.dt,goals,2*config.robot_radius,obst(:,1),obst(:,2),2*config.obstacle_raduis);

%% Equation of Motion

function x_next = forward_step(x,u,config)
x_next(1) = x(1) + cos(x(4))*cos(x(3))*u(1)*config.dt;
x_next(2) = x(2) + cos(x(4))*sin(x(3))*u(1)*config.dt;
x_next(3) = x(3) + (-sin(x(4))/config.D)*config.dt;
x_next(4) = u(2)*config.dt;
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
function obst_cost_value = calc_obst_cost(trajectory, obs, config)
    ox = obs(:,1);
    oy = obs(:,2);
    dx = trajectory(1,:) - ox;
    dy = trajectory(2,:) - oy;
    r = hypot(dx,dy);
    if min(min(r))<=(config.robot_radius+config.obstacle_raduis)
        obst_cost_value = inf;
        return;
    end
    min_r = min(min(r));
    obst_cost_value = 1/min_r;
end

%% Calculate To Goal Cost
function cost = calc_to_goal_cost(trajectory,goal)
dx = goal(1) - trajectory(1,end);
dy = goal(2) - trajectory(2,end);
error_angle = atan2(dy,dx);
cost_angle = error_angle - trajectory(3,end);
cost = abs(atan2(sin(cost_angle),cos(cost_angle)));
end

%% Caluclate Optimal Control and Trajectory
function [opt_u,opt_traj] = calc_control_traj(x,Dw,config,goal,obst)
x_init = x;
min_cost = inf;
opt_u = [0;0];
opt_traj = [x];

% start evaluating velocities from the window
for v = Dw(1):config.v_resolution:Dw(2)
    for omega = Dw(3):config.omega_resolution:Dw(4)
        trajectory = predict_trajectory(x_init,[v,omega],config);
        % calculate costs
        to_goal_cost = config.to_goal_cost_gain*calc_to_goal_cost(trajectory,goal);
        speed_cost = config.speed_cost_gain*(config.max_speed-v);
        obst_cost = config.obstacle_cost_gain*calc_obst_cost(trajectory,obst,config);

        final_cost = to_goal_cost + speed_cost + obst_cost;

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
end

%% General Control Function
function [u, trajectory] = dwa_control(x,u,config,goal,obst)
    Dw = calc_dynamic_window(u,config);
    [u,trajectory] = calc_control_traj(x,Dw,config,goal,obst);
end