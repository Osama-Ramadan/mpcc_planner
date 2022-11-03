import casadi.*
clear;clc;
%% Define The Global Map
mapmat = load('/home/ossama/mpcc_planner/MATLAB_files/MPC_Dynamics/path tracking/Maps/warehouse_map.mat');
mapmat = mapmat.imageOccupancy;
map = occupancyMap(mapmat,5);
show(map); hold on
%% Local Path Data Extract
T = readtable('paths/local_path_log2/path_pts.csv');
T1 = readtable('paths/local_path_log2/control_pts.csv');
T2 = readtable('paths/local_path_log2/waypoints.csv');
T3 = readtable('paths/local_path_log2/orig_waypoints.csv');
T4 = readtable('paths/local_path_log2/path_pts_orig.csv');


path_pts = table2array(T(2:end,["Var2","Var3","Var4"]));
path_pts_orig = table2array(T4(2:end,["Var2","Var3","Var4"]));
control_pts = table2array(T1(2:end,["Var2","Var3"]));
waypoints = table2array(T2(2:end,["Var2","Var3"]));
orig_waypoints = table2array(T3(2:end,["Var2","Var3"]));

%% Draw Path -----
for l = 1:size(waypoints,1)
    plot(waypoints(l,1),waypoints(l,2),'ob','linewidth',3);hold on
end

for l = 1:size(orig_waypoints,1)
    plot(orig_waypoints(l,1),orig_waypoints(l,2),'og','linewidth',3);hold on
end
plot(path_pts(:,1),path_pts(:,2),'-g','linewidth',2); % plot the reference trajectory
plot(path_pts_orig(:,1),path_pts_orig(:,2),'-k','linewidth',2); % plot the reference trajectory