clear; clc;

%% Set the Map
mapmat = load('/home/developer/Ossama_Repo/MasterThesisOssama/MATLAB_files/MPC_Dynamics/path tracking/Maps/warehouse_map.mat');
mapmat = mapmat.imageOccupancy;
map = occupancyMap(mapmat,5);

%% Simulation Robot Shape
rob_diam = 1;
obst_diam = 1;
%% Ros Bag Data Extract
bagreader = ros2bag('bag_files/dw_warehouse_Test1_obst');
msgs = readMessages(bagreader);
dw_cost = readMessages(select(bagreader,"Topic","/dw_cost"));
joint_command = readMessages(select(bagreader,"Topic","/joint_command"));
joint_states = readMessages(select(bagreader,"Topic","/joint_states"));
dw_feedback = readMessages(select(bagreader,"Topic","/dw_feedback"));
dw_predected_traj = readMessages(select(bagreader,"Topic","/dw_predicted_traj"));
dw_time = readMessages(select(bagreader,"Topic","/dw_time"));
obs_predected_traj = readMessages(select(bagreader,"Topic","/obs_predicted_traj"));

record_time = (bagreader.EndTime - bagreader.StartTime)*10^-9;

sim_data_points = min(length(dw_cost),length(joint_states));
controller_data_points = length(joint_command)-2;

sim_timestep = double(record_time)/double(sim_data_points);
controller_timestep = double(record_time)/double(controller_data_points);

sim_time_stamps = (1:sim_data_points)*sim_timestep;
controller_time_stamps = (1:controller_data_points)*controller_timestep;

N = size(dw_predected_traj{1}.poses);
N = N(2)-1;

dw_cmd = [];
vehicle_traj = [];
dw_time_ctr = [];
dw_cost_list = [];
for i = 1:controller_data_points
    
    cmd = joint_command{i}.velocity;
    cmd(2) = cmd(2)*0.127;

    feedback = dw_feedback{i}.data;
    cost = dw_cost{i}.data;
    dw_t = (dw_time{i}.data)*1000;

    dw_cmd = [dw_cmd ; cmd];
    dw_cost_list = [dw_cost_list ; cost];
    vehicle_traj = [vehicle_traj ; feedback];
    dw_time_ctr = [dw_time_ctr ; dw_t];
end

vehicle_pred_traj_f = [];
vehicle_pred_traj_b = [];
for i = 1:size(dw_predected_traj,1)
    for j = 1:N
        traj_pt = dw_predected_traj{i}.poses(j).position;
        directtion = dw_predected_traj{i}.header.frame_id;
        pt_orien = dw_predected_traj{i}.poses(j).orientation;
        angle = convert_angle(pt_orien.w,pt_orien.z,pt_orien.x,pt_orien.y);
        traj_pt = [traj_pt.x,traj_pt.y,angle];
        if directtion == "f"
            vehicle_pred_traj_f = [vehicle_pred_traj_f ; traj_pt];
        elseif directtion=="b"
            vehicle_pred_traj_b = [vehicle_pred_traj_b ; traj_pt];
        end
    end
end

%% Local Path Data Extract
T = readtable('bag_files/dw_warehouse_Test1/local_path_log/path_pts.csv');
T1 = readtable('bag_files/dw_warehouse_Test1/local_path_log/control_pts.csv');
T2 = readtable('bag_files/dw_warehouse_Test1/local_path_log/waypoints.csv');
T3 = readtable('bag_files/dw_warehouse_Test1/local_path_log/orig_waypoints.csv');

path_pts = table2array(T(2:end,["Var2","Var3","Var4"]));
control_pts = table2array(T1(2:end,["Var2","Var3"]));
waypoints = table2array(T2(2:end,["Var2","Var3"]));
orig_waypoints = table2array(T2(2:end,["Var2","Var3"]));


%% Data Plot And Scenrion Simulation
Draw_Scenario(vehicle_traj,vehicle_pred_traj_b,mpcc_cmd,N,mpcc_time_ctr,waypoints,path_pts,rob_diam,obst_diam,map)

%% Convert Qua to Eul
function angle = convert_angle(w,z,x,y)
        t3 = 2*(w*z + x*y);
        t4 = 1-2*(y*y + z*z);
        angle = atan2(t3,t4);
end