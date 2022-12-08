clear; clc;

%% Set the Map
mapmat = load('/home/developer/Ossama_Repo/MasterThesisOssama/MATLAB_files/MPC_Dynamics/path tracking/Maps/warehouse_map.mat');
mapmat = mapmat.imageOccupancy;
map = occupancyMap(mapmat,5);
%% Local Path Data Extract
T = readtable('/home/developer/Ossama_Repo/MasterThesisOssama/MATLAB_files/MPC_Dynamics/Testing_scenarios/Navigation/bag_files/Map1_Test11/local_path_log/path_pts.csv');
T1 = readtable('/home/developer/Ossama_Repo/MasterThesisOssama/MATLAB_files/MPC_Dynamics/Testing_scenarios/Navigation/bag_files/Map1_Test11/local_path_log/control_pts.csv');
T2 = readtable('/home/developer/Ossama_Repo/MasterThesisOssama/MATLAB_files/MPC_Dynamics/Testing_scenarios/Navigation/bag_files/Map1_Test11/local_path_log/waypoints.csv');
T3 = readtable('/home/developer/Ossama_Repo/MasterThesisOssama/MATLAB_files/MPC_Dynamics/Testing_scenarios/Navigation/bag_files/Map1_Test11/local_path_log/orig_waypoints.csv');

path_pts = table2array(T(2:end,["Var2","Var3","Var4"]));
control_pts = table2array(T1(2:end,["Var2","Var3"]));
waypoints = table2array(T2(2:end,["Var2","Var3"]));
orig_waypoints = table2array(T2(2:end,["Var2","Var3"]));

%% Generate Safe Boundaries
bounds = [];
for i = 1:size(path_pts,1)
    init_search_x = 2.2;
    init_search_y = 2.2;
    x_search_ext = cos(path_pts(3))*1;
    y_search_ext = sin(path_pts(3))*1;
    max_search_x = init_search_x + x_search_ext;
    min_search_x = init_search_x - x_search_ext;
    max_search_y = init_search_y + y_search_ext;
    min_search_y = init_search_y - y_search_ext;
    
    choose_pt = 131;
    index_in_map = local2grid(map,[path_pts(choose_pt,1), path_pts(choose_pt,2)]);
    min_y_search_index = max(index_in_map(1)-round(max_search_y*map.Resolution),1);
    max_y_search_index = min(index_in_map(1)+round(max_search_y*map.Resolution),size(mapmat,1));
    min_x_search_index = max(index_in_map(2)-round(max_search_x*map.Resolution),1);
    max_x_search_index = min(index_in_map(2)+round(max_search_x*map.Resolution),size(mapmat,2));

    searchmatrix = mapmat(min_y_search_index:max_y_search_index,...
                          min_x_search_index:max_x_search_index);

    search_area_min_x = index_in_map(2)-round(max_search_x*map.Resolution);
    search_area_max_y = index_in_map(1)-round(max_search_y*map.Resolution);
    
    [leftcorner, rightcorner] = getMaxSearchArea(searchmatrix);
    
    Upper_left_corner_local = grid2local(map,[search_area_max_y+leftcorner(1)-1,search_area_min_x+leftcorner(2)-1]);
    Lower_right_corner_local = grid2local(map,[search_area_max_y+rightcorner(1)-1,search_area_min_x+rightcorner(2)-1]);

    safe_zone_min_x1 = Upper_left_corner_local(1)+0.2;
    safe_zone_max_x1 = Lower_right_corner_local(1)-0.2;
    safe_zone_min_y1 = Lower_right_corner_local(2)+0.2;
    safe_zone_max_y1 = Upper_left_corner_local(2)-0.2; 

    bounds = [bounds; safe_zone_min_x1,safe_zone_max_y1,safe_zone_max_x1,safe_zone_min_y1];
end
%% Plot Points
% Show global map on figure
figure(103)
show(map); hold on
% Draw Path -----
plot(path_pts(:,1),path_pts(:,2),'-k','linewidth',2); % plot the reference trajectory
for i = 1:size(bounds,1)
        bound = bounds(i,:);
        P1 = [bound(1)+0.1,bound(2)-0.1];
        P2 = [bound(1)+0.1,bound(4)+0.1];
        P3 = [bound(3)-0.1,bound(4)+0.1];
        P4 = [bound(3)-0.1,bound(2)-0.1];
        plot([P1(1),P2(1),P3(1),P4(1),P1(1)],[P1(2),P2(2),P3(2),P4(2),P1(2)],"Color","b"); hold on
end