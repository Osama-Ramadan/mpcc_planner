clear; clc;

%% Set the Map
mapmat = load('/home/ossama/mpcc_planner/MATLAB_files/MPC_Dynamics/path tracking/Maps/warehouse_map.mat');
mapmat = mapmat.imageOccupancy;
map = occupancyMap(mapmat,5);

%% Simulation Robot Shape
rob_diam = 1;
obst_diam = 1;
%% Ros Bag Data Extract
bag_path = 'Ware_house_Test3';
path_f = strcat('bag_files/',bag_path);
bagreader = ros2bag(path_f);
msgs = readMessages(bagreader);
vehicle_odom = readMessages(select(bagreader,"Topic","/vehicle_odom"));
joint_command = readMessages(select(bagreader,"Topic","/joint_command"));
joint_states = readMessages(select(bagreader,"Topic","/joint_states"));
mpcc_predected_traj = readMessages(select(bagreader,"Topic","/mpcc_predicted_traj"));
mpcc_time = readMessages(select(bagreader,"Topic","/mpcc_time"));
collision_time = readMessages(select(bagreader,"Topic","/collision_time"));
mpcc_feedback = readMessages(select(bagreader,"Topic","/mpcc_feedback"));
obs_predected_traj = readMessages(select(bagreader,"Topic","/obs_predicted_traj"));
collision_pts = readMessages(select(bagreader,"Topic","/collision_pts"));
mpcc_torque = readMessages(select(bagreader,"Topic","/mpcc_torque"));
path_boundary = readMessages(select(bagreader,"Topic","/path_boundary"));


record_time = (bagreader.EndTime - bagreader.StartTime)*10^-9;

sim_data_points = min(length(vehicle_odom),length(joint_states));
controller_data_points = length(joint_command)-2;

sim_timestep = double(record_time)/double(sim_data_points);
controller_timestep = double(record_time)/double(controller_data_points);

sim_time_stamps = (1:sim_data_points)*sim_timestep;
controller_time_stamps = (1:controller_data_points)*controller_timestep;

N = size(mpcc_predected_traj{1}.poses);
N = N(2)-1;

if length(obs_predected_traj) > 0
N_obs = size(obs_predected_traj{1}.poses);
N_obs = N_obs(1);
end

mpcc_cmd = [];
vehicle_traj = [];
mpcc_time_ctr = [];
collision_time_list = [];
for i = 1:controller_data_points
    
    cmd = joint_command{i}.velocity;
    cmd(2) = cmd(2)*0.127;
    cmd(3) = mpcc_torque{i}.data;
    if size(collision_time,1)>i
    coll_time = collision_time{i}.data;
    collision_time_list = [collision_time_list ; coll_time];
    end

    feedback = mpcc_feedback{i}.data;
    
    mpcc_t = (mpcc_time{i}.data)*1000;
    
    mpcc_cmd = [mpcc_cmd ; cmd];
    vehicle_traj = [vehicle_traj ; feedback];
    mpcc_time_ctr = [mpcc_time_ctr ; mpcc_t];
end

vehicle_pred_traj_f = [];
vehicle_pred_traj_b = [];
for i = 1:size(mpcc_predected_traj,1)
    for j = 1:N
        traj_pt = mpcc_predected_traj{i}.poses(j).position;
        directtion = mpcc_predected_traj{i}.header.frame_id;
        pt_orien = mpcc_predected_traj{i}.poses(j).orientation;
        angle = convert_angle(pt_orien.w,pt_orien.z,pt_orien.x,pt_orien.y);
        traj_pt = [traj_pt.x,traj_pt.y,angle];
        if directtion == "f"
            vehicle_pred_traj_f = [vehicle_pred_traj_f ; traj_pt];
        elseif directtion=="b"
            vehicle_pred_traj_b = [vehicle_pred_traj_b ; traj_pt];
        end
    end
end

obst_pred_traj_f = [];
obst_pred_traj_b = [];
for i = 1:size(obs_predected_traj,1)
    for j = 1:1
        traj_pt = obs_predected_traj{i}.poses(j).position;
        directtion = obs_predected_traj{i}.header.frame_id;
        pt_orien = obs_predected_traj{i}.poses(j).orientation;
        angle = convert_angle(pt_orien.w,pt_orien.z,pt_orien.x,pt_orien.y);
        traj_pt = [traj_pt.x,traj_pt.y,angle];
        if directtion(1) == "f"
            obst_pred_traj_f = [obst_pred_traj_f ; traj_pt];
        elseif directtion(1)=="b"
            obst_pred_traj_b = [obst_pred_traj_b ; traj_pt];
        end
    end
end




% Get path Boundary
path_bound_data = path_boundary{1}.data;
bounds = [];
for i = 1:4:size(path_bound_data,1)-4
    bounds = [bounds;[path_bound_data(i),path_bound_data(i+1),path_bound_data(i+2),path_bound_data(i+3)]];
end

%% Local Path Data Extract
T = readtable(strcat('bag_files/',bag_path,'/local_path_log/path_pts.csv'));
T1 = readtable(strcat('bag_files/',bag_path,'/local_path_log/control_pts.csv'));
T2 = readtable(strcat('bag_files/',bag_path,'/local_path_log/waypoints.csv'));
T3 = readtable(strcat('bag_files/',bag_path,'/local_path_log/orig_waypoints.csv'));

path_pts = table2array(T(2:end,["Var2","Var3","Var4"]));
control_pts = table2array(T1(2:end,["Var2","Var3"]));
waypoints = table2array(T2(2:end,["Var2","Var3"]));
orig_waypoints = table2array(T2(2:end,["Var2","Var3"]));

%% Curvature
dx = gradient(vehicle_traj(400:500,1));
ddx = gradient(dx);
dy = gradient(vehicle_traj(400:500,2));
ddy = gradient(dy);
num = dx .* ddy - ddx .* dy;
denom = dx .* dx + dy .* dy;
denom = sqrt(denom);
denom = denom .* denom .* denom;
curvatur = num ./ denom;
curvature_val = max(abs(curvatur)) ;
curv_mean = mean(curvatur)
curvature(denom < 0) = NaN;

%% Check if reached the goal
goal = orig_waypoints(end,:);
distances = sqrt((vehicle_traj(:,1)-goal(1)).^2 + (vehicle_traj(:,2)-goal(2)).^2);
ind = find(distances(:)<3);
time_taken = controller_time_stamps(ind(1))
d = hypot(diff(vehicle_traj(:,1)), diff(vehicle_traj(:,2)));                            % Distance Of Each Segment
d_tot = sum(d)                                         % Total Distance
dp = hypot(diff(path_pts(:,1)), diff(path_pts(:,2)));                            % Distance Of Each Segment
path_tot = sum(dp)   
max_cmd_time = max(mpcc_time_ctr(10:end));
average_cmd_time = mean(mpcc_time_ctr(10:end))
[i,dist] = dsearchn(path_pts(:,1:2),vehicle_traj(:,1:2));
track_error = sum(dist)/length(vehicle_traj(:,1))
%% Data Plot And Scenrion Simulation
Draw_Scenario(vehicle_traj,vehicle_pred_traj_b,mpcc_cmd,N,mpcc_time_ctr,waypoints,path_pts(:,:),rob_diam,obst_diam,map,bounds, obst_pred_traj_b,controller_timestep)

%% Convert Qua to Eul
function angle = convert_angle(w,z,x,y)
        t3 = 2*(w*z + x*y);
        t4 = 1-2*(y*y + z*z);
        angle = atan2(t3,t4);
end