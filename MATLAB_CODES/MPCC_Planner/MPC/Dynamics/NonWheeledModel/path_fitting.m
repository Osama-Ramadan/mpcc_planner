clear; clc;
waypoints = [0 0; ...
	50 20; ...
	100 0; ...
	150 10];
goalsp = [0 0 0 1/1.82;5 -2 0 1/1.82;10 -7 0 1/1.82];

goalsp = [0 0 0; 10 -5 0; 15 5 0; 10 15 pi/3; 0 15 pi/2];

refPath = referencePathFrenet(goalsp);
connector = trajectoryGeneratorFrenet(refPath);

initState = [0 0 0 0 0 0];  % [S ds ddS L dL ddL]
termState = [70 0 0 0 0 0]; % [S ds ddS L dL ddL]
[~,trajGlobal] = connect(connector,initState,termState,30);
globalTraj = frenet2global(refPath,trajGlobal.Trajectory);
nearestPathPoint = closestPoint(refPath,goalsp(2,1:2));

show(refPath);
hold on
axis equal
plot(trajGlobal.Trajectory(:,1),trajGlobal.Trajectory(:,2),'b')
legend(["Waypoints","Reference Path","Trajectory to 30m"])

%% new way
waypoints = [0 0 0; 10 -5 0; 15 5 0; 10 15 pi/3; 0 15 pi/2];
waypoints=unique(waypoints,'rows','stable'); % <- this is required because your example as duplicated points
% Spline interpolation
t=cumsum([0;sqrt(sum(diff(waypoints,1,1).^2,2))]);
ti=linspace(t(1),t(end),128);
xyzi=interp1(t,waypoints,ti,'spline');
%plot
figure
plot(waypoints(:,1),waypoints(:,2),'or',xyzi(:,1),xyzi(:,2),'b');
axis equal