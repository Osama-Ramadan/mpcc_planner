import casadi.*
clear;clc;
%% Define The Global Map
mapmat = load('/home/ossama/mpcc_planner/MATLAB_files/MPC_Dynamics/path tracking/Maps/omni_map1.mat');
mapmat = mapmat.imageOccupancy;
map = occupancyMap(mapmat,5);
show(map); hold on
%% Local Path Data Extract

T1 = readtable('paths/Map1/local_path_log_implement/control_pts_orig0.csv');
T2 = readtable('paths/Map1/local_path_log_implement/waypoints_orig0.csv');
T4 = readtable('paths/Map1/local_path_log_implement/path_pts_orig0.csv');

T = readtable('paths/Map1/local_path_log_implement/path_pts1.csv');
T3 = readtable('paths/Map1/local_path_log_implement/waypoints1.csv');

T5 = readtable('paths/Map1/local_path_log_implement/path_pts2.csv');
T6 = readtable('paths/Map1/local_path_log_implement/waypoints2.csv');

T7 = readtable('paths/Map1/local_path_log_implement/path_pts3.csv');
T8 = readtable('paths/Map1/local_path_log_implement/waypoints3.csv');

path_pts = table2array(T(2:end,["Var2","Var3","Var4"]));
path_pts_orig = table2array(T4(2:end,["Var2","Var3","Var4"]));
control_pts = table2array(T1(2:end,["Var2","Var3"]));
waypoints = table2array(T3(2:end,["Var2","Var3"]));
orig_waypoints = table2array(T2(2:end,["Var2","Var3"]));

path_pts2 = table2array(T5(2:end,["Var2","Var3","Var4"]));
waypoints2 = table2array(T6(2:end,["Var2","Var3"]));

path_pts3 = table2array(T7(2:end,["Var2","Var3","Var4"]));
waypoints3 = table2array(T8(2:end,["Var2","Var3"]));

%% Curvature
dx = gradient(path_pts(:,1));
ddx = gradient(dx);
dy = gradient(path_pts(:,2));
ddy = gradient(dy);
num = dx .* ddy - ddx .* dy;
denom = dx .* dx + dy .* dy;
denom = sqrt(denom);
denom = denom .* denom .* denom;
curvatur = num ./ denom;
curvature_val = max(abs(curvatur())) ;
curvature(denom < 0) = NaN;

%% Draw Path -----
for l = 1:size(orig_waypoints,1)
    plot(orig_waypoints(l,1),orig_waypoints(l,2),'or','linewidth',3);hold on
end

 plot(waypoints(l,1),waypoints(l,2),'ob','linewidth',2);hold on
 plot(path_pts(:,1),path_pts(:,2),'-k','linewidth',2);hold on % plot the reference trajectory

 plot(waypoints2(l,1),waypoints2(l,2),'ob','linewidth',2);hold on
 plot(path_pts2(:,1),path_pts2(:,2),'-k','linewidth',2);hold on % plot the reference trajectory

 plot(waypoints3(l,1),waypoints3(l,2),'ob','linewidth',2);hold on
 plot(path_pts3(:,1),path_pts3(:,2),'-g','linewidth',2);hold on % plot the reference trajectory


%plot(path_pts(:,1),path_pts(:,2),'-g','linewidth',2); % plot the reference trajectory
plot(path_pts_orig(:,1),path_pts_orig(:,2),'-k','linewidth',2);hold on % plot the reference trajectory


%plot(path_pts(:,1),path_pts(:,2),'-g','linewidth',2); % plot the reference trajectory