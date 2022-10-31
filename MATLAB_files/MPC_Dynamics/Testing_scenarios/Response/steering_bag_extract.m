clear; clc;
bagreader = ros2bag('ros_bags/IgoNeo_Lat_response_S100000T');
msgs = readMessages(bagreader);
bagSel = select(bagreader,"Topic","/odom");
bagSe2 = select(bagreader,"Topic","/joint_command");
bagSe3 = select(bagreader,"Topic","/joint_states");

msgsFiltered = readMessages(select(bagreader,"Topic","/odom"));
msgsFiltered2 = readMessages(select(bagreader,"Topic","/joint_command"));
msgsFiltered3 = readMessages(select(bagreader,"Topic","/joint_states"));
%msgsFiltered4 = readMessages(select(bagreader,"Topic","/tf"));

record_time = (bagreader.EndTime - bagreader.StartTime)*10^-9;
data_points = min(length(msgsFiltered),length(msgsFiltered2));
timestep = double(record_time)/double(data_points);
time_stamps = (1:data_points)*timestep;
vehicle_speed = [];
wheel_speed = [];
command = [];
for i = 1:data_points
    cmd = msgsFiltered2{i}.position;
    wheel = msgsFiltered3{i}.position;

    command = [command ; cmd];
    wheel_speed = [wheel_speed ; wheel];
end
%vehicle_speed = -vehicle_speed(:,1);
vehicle_speed = abs(wheel_speed(:,1));
command = abs(command(:,1));
%wheel_s = wheel_speed(:,4);

figure(35)
plot(time_stamps(:,1:size(command,1)),command,"-" ,"LineWidth",1.5); hold on
%plot(time_stamps,vehicle_speed, "LineWidth", 1.5);hold on
plot(time_stamps(:,1:size(vehicle_speed,1)),vehicle_speed, "LineWidth", 1.5);
title("IgoNeo Model Response")
xlabel("Time (ms)")
ylabel("Linear Speed (s)")
legend("Velociy Command", "Velocity Response")