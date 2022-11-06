clear; clc;
%% Define The Global Map
mapmat = load('/home/developer/mpcc_planner/MATLAB_files/MPC_Dynamics/path tracking/Maps/omni_map1.mat');
mapmat = mapmat.imageOccupancy;
map = occupancyMap(mapmat,5);
%show(map); hold on


%% Extract Bounding Time

T = readtable('Time.csv');
data = table2array(T(2:end,["Var2","Var3"]));
time = data(:,1);
arae = (2*data(:,2)).^2;
plot(time)
xlabel('Search Area (m^2)')
ylabel('Computation Time (s)')
