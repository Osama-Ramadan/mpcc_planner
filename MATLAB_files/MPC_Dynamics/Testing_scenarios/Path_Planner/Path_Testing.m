import casadi.*
clear;clc;
%% Define The Global Map
mapmat = load('Maps/warehouse_map.mat');
mapmat = mapmat.imageOccupancy;
map = occupancyMap(mapmat,5);
%% Generate the Path
goalsp = [[39, 3]; [37, 13]; [35.3, 14.09]; [35.3, 13.4]; [20, 24]];
[Xout, wayps, Q_p] = Generate_Trajectory(goalsp,1,4);
figure(503)
show(map); hold on
plot(Xout(:,1),Xout(:,2),'-k','linewidth',2); % plot the reference trajectory
for l = 1:length(goalsp(:,1))
    plot(goalsp(l,1),goalsp(l,2),'ob','linewidth',2);hold on
end
%% Adjusting the Path
function [goalsp, path_changed] = adjust_path(collision_t,Q_p,map,goalsp,wayps,seg,goal,derivative,spline,goals_orig)
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

M = [d_force_val(1),        0,        -d_s(1) ;...
            0,        d_force_val(4), -d_s(2); ...
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