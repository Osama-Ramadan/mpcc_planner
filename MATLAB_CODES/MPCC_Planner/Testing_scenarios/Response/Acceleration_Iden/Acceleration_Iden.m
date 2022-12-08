%clear; clc;
%T = readtable('/home/developer/Ossama_Repo/MasterThesisOssama/MATLAB_files/Ossama_Experm/Acceleration_Iden/Data1.csv');
%T1 = readtable('/home/developer/Ossama_Repo/MasterThesisOssama/MATLAB_files/Ossama_Experm/Acceleration_Iden/Data2.csv');

%Data1 = table2array(T(2:end,["Var2","Var3","Var4","Var5","Var6","Var7","Var8"]));
%Data2 = table2array(T1(2:end,["Var2","Var3","Var4","Var5","Var6","Var7","Var8","Var9","Var10","Var11"]));

%% Data description
% Data1 :[SpeedCmd, OmegaCmd, SpeedSetPoint, SteeringSetPoint, BreakReleas]
% Data2 :[SpeedCmd, OmegaCmd, SpeedSetPoint, SteeringSetPoint, BreakReleas, ActualSpeed, ActulaSteering]
clear; clc;
load("Acc_data.mat")
Data1(:,1) = Data1(:,1) - Data1(1,1);
Data2(:,1) = Data2(:,1) - Data2(1,1);

Time = Data2(:,1)*10^-3;
SpeedCommand = Data2(:,3);
SpeedResponse = Data2(:,8);

% Applying Control Limits
for i = 1:length(SpeedCommand)
    if SpeedCommand(i) > 250
        SpeedCommand(i) = 250;
    end
end

%% First Patch Full Data
figure(1)
plot(SpeedCommand,"LineWidth",1.5); hold on
plot(movmean(SpeedResponse,10),"LineWidth",1.5);
title("Acc/Decc Response")
xlabel("Time (s)")
ylabel("Speed (mm/s)")
legend("Speed Command", "Speed Actual")

%% Second Patch
Time1 = Time(1299:2379);
SpeedCommand1 = SpeedCommand(1299:2379);
SpeedResponse1 = SpeedResponse(1299:2379);
%figure(11)
%plot(Time1,SpeedCommand1,"LineWidth",1.5); hold on
%plot(Time1,SpeedResponse1,"LineWidth",1.5);


%% Third Patch
Time2 = Time(2379:2519);
SpeedCommand2 = SpeedCommand(2379:2519);
SpeedResponse2 = SpeedResponse(2379:2519);
%figure(12)
%plot(Time2,SpeedCommand2,"LineWidth",1.5); hold on
%plot(Time2,SpeedResponse2,"LineWidth",1.5);

%% Third Patch
Time2 = Time(4238:4520);
Time2 = Time2-Time(4238);
SpeedCommand2 = -SpeedCommand(4238:4520);
SpeedResponse2 = -SpeedResponse(4238:4520);
ind = max(SpeedResponse2);
%SpeedCommand2(50:end)=1600;
figure(120)
plot(Time2,SpeedCommand2*0.001,"LineWidth",1.5); hold on
stairs(Time2,SpeedResponse2*0.001,"LineWidth",1.5);
xlabel("Time (s)")
ylabel("Speed (m/s)")
legend("Speed Command", "Speed Actual")
grid on
%% Forth Patch
Time2 = Time(4510:4650);
Time2 = Time2-Time(4510);
SpeedCommand2 = -SpeedCommand(4510:4650);
SpeedResponse2 = -SpeedResponse(4510:4650);
figure(130)
plot(Time2,SpeedCommand2*0.001,"LineWidth",1.5); hold on
stairs(Time2,SpeedResponse2*0.001,"LineWidth",1.5);
xlabel("Time (s)")
ylabel("Speed (m/s)")
legend("Speed Command", "Speed Actual")
grid on