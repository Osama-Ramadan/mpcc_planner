clear; clc;
%fileName = '05_steering.json'; % filename in JSON extension
%fid = fopen(fileName); % Opening the file
%raw = fread(fid,inf); % Reading the contents
%str = char(raw'); % Transformation
%fclose(fid); % Closing the file
%data = jsondecode(str); % Using the jsondecode function to parse JSON from string

%data = struct2cell(data);
%stamptime = [];
%steeringangle = [];
%steeringcommand = [];
%for i = 1:length(data)
 %   datapoint = data(i);
  %  stamptime = [stamptime ; datapoint{1}.recTime];
   % steeringangle = [steeringangle ; datapoint{1}.actSteerAngle];
    %steeringcommand = [steeringcommand ; datapoint{1}.steer];
%end


%stamptime(1:end) = stamptime(1:end)-stamptime(1);
%steeringangle = steeringangle*(180/pi);

load("steering_data.mat")

% Up1
time = stamptime(302:418);
input = steeringcommand(302:418);
output = steeringangle(302:418);
%Up2
time1 = stamptime(2546:2657);
input1 = steeringcommand(2546:2657);
output1 = steeringangle(2546:2657);

%Down1
time2 = stamptime(168:300);
input2 = steeringcommand(168:300);
output2 = steeringangle(168:300);


figure(31)
plot(stamptime,steeringcommand,"-" ,"LineWidth",1.5); hold on
plot(stamptime, steeringangle, "LineWidth", 1.5);
title("Steering System Response")
xlabel("Time (ms)")
ylabel("Steering Angle (deg)")
legend("Steering Command", "Steering Response")

figure(32)
plot(time,input,"-" ,"LineWidth",1.5); hold on
plot(time,output, "LineWidth", 1.5);
title("Steering System Response")
xlabel("Time (ms)")
ylabel("Steering Angle (deg)")
legend("Steering Command", "Steering Response")

figure(33)
plot(time1,input1,"-" ,"LineWidth",1.5); hold on
plot(time1,output1, "LineWidth", 1.5);
title("Steering System Response")
xlabel("Time (ms)")
ylabel("Steering Angle (deg)")
legend("Steering Command", "Steering Response")

figure(34)
plot(time2,input2,"-" ,"LineWidth",1.5); hold on
plot(time2,output2, "LineWidth", 1.5);
title("Steering System Response")
xlabel("Time (ms)")
ylabel("Steering Angle (deg)")
legend("Steering Command", "Steering Response")
