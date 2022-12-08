clc; clear;

%% Define the map
mapmat = load('Maps/map1.mat');
mapmat = mapmat.mapmat;
map = occupancyMap(mapmat,10);

%% Generate the path
goalsp = [3 10 ; 12 3 ]; % (maps)
lookahead = 1;
currentInt = 1;
[Xout, wayps, Q_p] = Generate_Trajectory(goalsp,currentInt,lookahead);

%% Draw the Path before
figure(100)
show(map); hold on
% Plot waypoints on the global map
for l = 1:length(goalsp(:,1))
    plot(goalsp(l,1),goalsp(l,2),'ob','linewidth',2);hold on
end
% Plot control points
for l = 1:size(Q_p,1)
    plot(Q_p(l,1),Q_p(l,2),'xr','linewidth',2);hold on
end
% Plot Trajectory
plot(Xout(:,1),Xout(:,2),'-g','linewidth',1); % plot the reference trajectory

%% Check for Collision
collision_t = [];
for r = 1:100
    point_index = local2grid(map,[Xout(r,1), Xout(r,2)]);
    if mapmat(point_index(1),point_index(2)) == 1
        collision_t = r-2;
        break;
    end
end

p2 = point_on_path((collision_t*0.01),Q_p);
% bubble points that are used to modify the path 
step_size = 0.01;
p2_index = local2grid(map,[p2(1), p2(2)]);
p2_obst_poses = 1;

while(~isempty(p2_obst_poses))
% obstalces around points
p2_obst_poses = search_near(p2,1,1, map);

% Obstalce forces
p2_obst_force_x = 0 ;
p2_obst_force_y = 0 ;
force_gain = 0.01;

for i = 1:size(p2_obst_poses,1)
    obst = p2_obst_poses(i,:);
    d_x = p2(1) - obst(1);
    d_y = p2(2) - obst(2);
    p2_obst_force_x = p2_obst_force_x + (force_gain/(d_x+0.01));
    p2_obst_force_y = p2_obst_force_y + (force_gain/(d_y+0.01));
end

% update bubble point position
delta_x = p2_obst_force_x*step_size;
delta_y = p2_obst_force_y*step_size;
p2(1) = p2(1)+delta_x;
p2(2) = p2(2)+delta_y;
end

goalsp = [goalsp(1,:);p2';goalsp(2,:)];
lookahead = 2;
[Xout, wayps, Q_p] = Generate_Trajectory(goalsp,currentInt,lookahead);
%% Draw the Path
figure(103)
show(map); hold on
% Plot waypoints on the global map
for l = 1:length(goalsp(:,1))
    plot(goalsp(l,1),goalsp(l,2),'ob','linewidth',2);hold on
end
% Plot control points
for l = 1:size(Q_p,1)
    plot(Q_p(l,1),Q_p(l,2),'xr','linewidth',2);hold on
end
% Plot Trajectory
plot(Xout(:,1),Xout(:,2),'-g','linewidth',1); % plot the reference trajectory

%% Search Neighbour function
function obst_list = search_near(point_index, x_search, y_search, map)
obst_list = [];
mapmat = round(getOccupancy(map));
point_index = local2grid(map,[point_index(1), point_index(2)]);
for i = point_index(2)-x_search*map.Resolution:point_index(2)+x_search*map.Resolution           % Search the whole Area
    for j = point_index(1)-y_search*map.Resolution:point_index(1)+y_search*map.Resolution
        if mapmat(j,i) == 1                                                                     % if you found an obstacle
            obst_pose = grid2local(map,[j,i]);
            obst_list = [obst_list ; obst_pose];
        end
    end
end
end

%% get point on path
function point = point_on_path(t,Qs)
Qx = Qs(:,1);
Qy = Qs(:,2);
x_t= (1/6)*(((1-t).^3*Qx(1)) + (3*t.^3-6*t.^2+4)*Qx(2) + (-3*t.^3+3*t.^2+3*t+1)*Qx(3) + t.^3*Qx(4));
y_t= (1/6)*(((1-t).^3*Qy(1)) + (3*t.^3-6*t.^2+4)*Qy(2) + (-3*t.^3+3*t.^2+3*t+1)*Qy(3) + t.^3*Qy(4));
point = [x_t ; y_t];
end
