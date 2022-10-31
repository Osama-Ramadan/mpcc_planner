clear; clc;
bagreader = ros2bag('IgoNeo_resp_500kg_0.1fr');
msgs = readMessages(bagreader);
bagSel = select(bagreader,"Topic","/odom");
bagSe2 = select(bagreader,"Topic","/joint_command");
bagSe3 = select(bagreader,"Topic","/joint_states");

msgsFiltered = readMessages(select(bagreader,"Topic","/odom"));
msgsFiltered2 = readMessages(select(bagreader,"Topic","/joint_command"));
msgsFiltered3 = readMessages(select(bagreader,"Topic","/joint_states"));
msgsFiltered4 = readMessages(select(bagreader,"Topic","/tf"));

record_time = (bagreader.EndTime - bagreader.StartTime)*10^-9;
data_points = min(length(msgsFiltered),length(msgsFiltered2));
timestep = double(record_time)/double(data_points);
time_stamps = (1:data_points)*timestep;
vehicle_speed = [];
wheel_speed = [];
command = [];
for i = 1:data_points
    x_dot = msgsFiltered{i}.twist.twist.linear.x;
    y_dot = msgsFiltered{i}.twist.twist.linear.y;
    cmd = msgsFiltered2{i}.velocity;
    vehicle_speed = [vehicle_speed ; x_dot , y_dot];
    command = [command ; cmd];
    %wheel = msgsFiltered3{i}.velocity;
   % if abs(wheel(4)) < 20
   % vehicle_speed = [vehicle_speed ; x_dot , y_dot];
    %command = [command ; cmd];
        %wheel_speed = [wheel_speed ; wheel];
    %end
end
vehicle_speed = sqrt(vehicle_speed(:,1).^2+vehicle_speed(:,2).^2);
command = -command(:,2)*0.127;
%wheel_s = wheel_speed(:,4);

figure(35)
plot(command,"-" ,"LineWidth",1.5); hold on
%plot(time_stamps,vehicle_speed, "LineWidth", 1.5);hold on
plot(vehicle_speed, "LineWidth", 1.5);
title("IgoNeo Model Response")
xlabel("Time (ms)")
ylabel("Linear Speed (m/s)")
legend("Velociy Command", "Velocity Response")