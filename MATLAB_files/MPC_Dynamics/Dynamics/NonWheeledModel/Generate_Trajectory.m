function [Xout, refPath, goalsp] = Generate_Trajectory(l, Tvec)

goalsp = [0 0 0; 10 -5 0; 15 5 0; 10 15 pi/3; 0 15 pi/2];
refPath = referencePathFrenet(goalsp);
connector = trajectoryGeneratorFrenet(refPath);

initState = [0 0 0 0 0 0];  % [S ds ddS L dL ddL]
termState = [70 0 0 0 0 0]; % [S ds ddS L dL ddL]
[~,trajGlobal] = connect(connector,initState,termState,30);


Xout = trajGlobal.Trajectory(:,1:3);

t=cumsum([0;sqrt(sum(diff(goalsp,1,1).^2,2))]);
ti=linspace(t(1),t(end),128);
Xout=interp1(t,goalsp,ti,'spline');
%% Plotting Your Results
%figure(1)
%plot(Xout(:,1),Xout(:,2), 'linewidth', 1)
%title('Plot of x vs y')
%axis ([-1 3 -1.5 2.5])


