clear; clc;
%% RosBag1
bagreader = ros2bag('ros_bags/IgoNeo_Long_response_D10T');
msgs = readMessages(bagreader);
bagSel = select(bagreader,"Topic","/odom");
bagSe2 = select(bagreader,"Topic","/joint_command");
bagSe3 = select(bagreader,"Topic","/joint_states");

msgsFiltered = readMessages(select(bagreader,"Topic","/odom"));
msgsFiltered2 = readMessages(select(bagreader,"Topic","/joint_command"));
msgsFiltered3 = readMessages(select(bagreader,"Topic","/joint_states"));

record_time = (bagreader.EndTime - bagreader.StartTime)*10^-9;
data_points = min(length(msgsFiltered),length(msgsFiltered2));
timestep = double(record_time)/double(data_points);
time_stamps = (1:data_points)*timestep;
vehicle_speed = [];
wheel_speed = [];
command = [];
for i = 1:data_points
    x_dot = msgsFiltered{i}.twist.twist.linear.x;
    cmd = msgsFiltered2{i}.velocity;
    wheel = msgsFiltered3{i}.velocity;
    if abs(wheel(4)) < 30
        vehicle_speed = [vehicle_speed ; x_dot];
        command = [command ; cmd];
        wheel_speed = [wheel_speed ; wheel];
    end
end
vehicle_speed1 = -wheel_speed(:,4)*0.127;
vehicle_speed1 = -vehicle_speed;
command1 = -command(:,2)*0.127;
%% RosBag2
bagreader = ros2bag('ros_bags/IgoNeo_Long_response_D100T');
msgs = readMessages(bagreader);
bagSel = select(bagreader,"Topic","/odom");
bagSe2 = select(bagreader,"Topic","/joint_command");
bagSe3 = select(bagreader,"Topic","/joint_states");

msgsFiltered = readMessages(select(bagreader,"Topic","/odom"));
msgsFiltered2 = readMessages(select(bagreader,"Topic","/joint_command"));
msgsFiltered3 = readMessages(select(bagreader,"Topic","/joint_states"));

record_time = (bagreader.EndTime - bagreader.StartTime)*10^-9;
data_points = min(length(msgsFiltered),length(msgsFiltered2));
timestep = double(record_time)/double(data_points);
time_stamps = (1:data_points)*timestep;
vehicle_speed = [];
wheel_speed = [];
command = [];
for i = 1:data_points
    x_dot = msgsFiltered{i}.twist.twist.linear.x;
    cmd = msgsFiltered2{i}.velocity;
    wheel = msgsFiltered3{i}.velocity;
    if abs(wheel(4)) < 30
        vehicle_speed = [vehicle_speed ; x_dot];
        command = [command ; cmd];
        wheel_speed = [wheel_speed ; wheel];
    end
end
vehicle_speed2 = -wheel_speed(:,4)*0.127;
vehicle_speed2 = -vehicle_speed;
command2 = -command(:,2)*0.127;
%% RosBag3
bagreader = ros2bag('ros_bags/IgoNeo_Long_response_D1000T');
msgs = readMessages(bagreader);
bagSel = select(bagreader,"Topic","/odom");
bagSe2 = select(bagreader,"Topic","/joint_command");
bagSe3 = select(bagreader,"Topic","/joint_states");

msgsFiltered = readMessages(select(bagreader,"Topic","/odom"));
msgsFiltered2 = readMessages(select(bagreader,"Topic","/joint_command"));
msgsFiltered3 = readMessages(select(bagreader,"Topic","/joint_states"));

record_time = (bagreader.EndTime - bagreader.StartTime)*10^-9;
data_points = min(length(msgsFiltered),length(msgsFiltered2));
timestep = double(record_time)/double(data_points);
time_stamps = (1:data_points)*timestep;
vehicle_speed = [];
wheel_speed = [];
command = [];
for i = 1:data_points
    x_dot = msgsFiltered{i}.twist.twist.linear.x;
    cmd = msgsFiltered2{i}.velocity;
    wheel = msgsFiltered3{i}.velocity;
    if abs(wheel(4)) < 30
        vehicle_speed = [vehicle_speed ; x_dot];
        command = [command ; cmd];
        wheel_speed = [wheel_speed ; wheel];
    end
end
vehicle_speed3 = -wheel_speed(:,4)*0.127;
vehicle_speed3 = -vehicle_speed;
command3 = -command(:,2)*0.127;

%% RosBag4
bagreader = ros2bag('ros_bags/IgoNeo_Long_response_D10000T');
msgs = readMessages(bagreader);
bagSel = select(bagreader,"Topic","/odom");
bagSe2 = select(bagreader,"Topic","/joint_command");
bagSe3 = select(bagreader,"Topic","/joint_states");

msgsFiltered = readMessages(select(bagreader,"Topic","/odom"));
msgsFiltered2 = readMessages(select(bagreader,"Topic","/joint_command"));
msgsFiltered3 = readMessages(select(bagreader,"Topic","/joint_states"));

record_time = (bagreader.EndTime - bagreader.StartTime)*10^-9;
data_points = min(length(msgsFiltered),length(msgsFiltered2));
timestep = double(record_time)/double(data_points);
time_stamps = (1:data_points)*timestep;
vehicle_speed = [];
wheel_speed = [];
command = [];
for i = 1:data_points
    x_dot = msgsFiltered{i}.twist.twist.linear.x;
    cmd = msgsFiltered2{i}.velocity;
    wheel = msgsFiltered3{i}.velocity;
    if abs(wheel(4)) < 30
        vehicle_speed = [vehicle_speed ; x_dot];
        command = [command ; cmd];
        wheel_speed = [wheel_speed ; wheel];
    end
end
vehicle_speed4 = -wheel_speed(:,4)*0.127;
vehicle_speed4 = -vehicle_speed;
command4 = -command(:,2)*0.127;

%% RosBag5
bagreader = ros2bag('ros_bags/IgoNeo_Long_response_D100000T');
msgs = readMessages(bagreader);
bagSel = select(bagreader,"Topic","/odom");
bagSe2 = select(bagreader,"Topic","/joint_command");
bagSe3 = select(bagreader,"Topic","/joint_states");

msgsFiltered = readMessages(select(bagreader,"Topic","/odom"));
msgsFiltered2 = readMessages(select(bagreader,"Topic","/joint_command"));
msgsFiltered3 = readMessages(select(bagreader,"Topic","/joint_states"));

record_time = (bagreader.EndTime - bagreader.StartTime)*10^-9;
data_points = min(length(msgsFiltered),length(msgsFiltered2));
timestep = double(record_time)/double(data_points);
time_stamps = (1:data_points)*timestep;
vehicle_speed = [];
wheel_speed = [];
command = [];
for i = 1:data_points
    x_dot = msgsFiltered{i}.twist.twist.linear.x;
    cmd = msgsFiltered2{i}.velocity;
    wheel = msgsFiltered3{i}.velocity;
    if abs(wheel(4)) < 30
        vehicle_speed = [vehicle_speed ; x_dot];
        command = [command ; cmd];
        wheel_speed = [wheel_speed ; wheel];
    end
end
vehicle_speed5 = -wheel_speed(:,4)*0.127;
vehicle_speed5 = -vehicle_speed;
command5 = -command(:,2)*0.127;
%% RosBag6
bagreader = ros2bag('ros_bags/IgoNeo_Long_response_D1000000T');
msgs = readMessages(bagreader);
bagSel = select(bagreader,"Topic","/odom");
bagSe2 = select(bagreader,"Topic","/joint_command");
bagSe3 = select(bagreader,"Topic","/joint_states");

msgsFiltered = readMessages(select(bagreader,"Topic","/odom"));
msgsFiltered2 = readMessages(select(bagreader,"Topic","/joint_command"));
msgsFiltered3 = readMessages(select(bagreader,"Topic","/joint_states"));

record_time = (bagreader.EndTime - bagreader.StartTime)*10^-9;
data_points = min(length(msgsFiltered),length(msgsFiltered2));
timestep = double(record_time)/double(data_points);
time_stamps = (1:data_points)*timestep;
vehicle_speed = [];
wheel_speed = [];
command = [];
for i = 1:data_points
    x_dot = msgsFiltered{i}.twist.twist.linear.x;
    cmd = msgsFiltered2{i}.velocity;
    wheel = msgsFiltered3{i}.velocity;
    if abs(wheel(4)) < 30
        vehicle_speed = [vehicle_speed ; x_dot];
        command = [command ; cmd];
        wheel_speed = [wheel_speed ; wheel];
    end
end
vehicle_speed6 = -wheel_speed(:,4)*0.127;
vehicle_speed6 = -vehicle_speed;
command6 = -command(:,2)*0.127;
%% Figure1
figure(35)
Time = ((1:300)*timestep)';
plot(Time,command6(1:300),"-" ,"LineWidth",1.5); hold on
%stairs(Time,vehicle_speed1(1:500), "LineWidth", 1.5);
xlabel("Time (s)")

%% Figure2
%stairs(Time,vehicle_speed2(1:500), "LineWidth", 1.5);

%% Figure3
stairs(Time,vehicle_speed3(1:300), "LineWidth", 1.5);

%% Figure4
stairs(Time,vehicle_speed4(1:300), "LineWidth", 1.5);

%% Figure5
stairs(Time,vehicle_speed5(1:300), "LineWidth", 1.5);

%% Figure6
stairs(Time,vehicle_speed6(1:300), "LineWidth", 1.5);

legend("cmd","D=1000","D=10000","D=100000","D=1000000");hold off
grid on
%% Figure1
figure(36)
Time = ((1:251)*timestep)';
plot(Time,command6(950:1200),"-" ,"LineWidth",1.5); hold on
%stairs(Time,vehicle_speed1(980:1500), "LineWidth", 1.5);


%% Figure2
%stairs(Time,vehicle_speed2(980:1500), "LineWidth", 1.5);

%% Figure3
stairs(Time,vehicle_speed3(950:1200), "LineWidth", 1.5);

%% Figure4
stairs(Time,vehicle_speed4(950:1200), "LineWidth", 1.5);

%% Figure5
stairs(Time,vehicle_speed5(950:1200), "LineWidth", 1.5);

%% Figure6
stairs(Time,vehicle_speed6(950:1200), "LineWidth", 1.5);
xlabel("Time (s)")
legend("cmd","D=1000","D=10000","D=100000","D=1000000")
grid on