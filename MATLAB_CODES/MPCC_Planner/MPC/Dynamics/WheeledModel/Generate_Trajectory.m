function [Xout] = Generate_Trajectory(l, Tvec)

% Vert. Up
X01 = [0,0,pi/2,0,0,0,0,0,0,0,0,0];
v_ref  = 0.5;
omega_ref = 0;
traj1 = @(~,x)Forkleft_ODE(l,v_ref,omega_ref,x);
[Tout1, Xout1] = ode45( traj1, Tvec(:,1:floor(length(Tvec)/2)), X01);

%Xout = Xout1;
% Hor. Right
X02 = [0,Xout1(end,2),0,0,0,0,0,0,0,0,0,0];
traj2 = @(~,x)Forkleft_ODE(l,v_ref,omega_ref,x);
[Tout2, Xout2] = ode45( traj2, Tvec(:,1:floor(length(Tvec)/2)), X02);

Xout = [Xout1 ; Xout2];
%% Plotting Your Results
%figure(1)
%plot(Xout(:,1),Xout(:,2), 'linewidth', 1)
%title('Plot of x vs y')
%axis ([-1 3 -1.5 2.5])


