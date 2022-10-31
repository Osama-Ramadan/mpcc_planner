function Draw_Scenario(vehicle_traj,vehicle_predicted_traj,cmd,N,t_mpc,waypoints,path_pts,rob_diam,obs_diam,map,bounds)
%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];


r = rob_diam/2;     % robot radius
r_o = obs_diam/2;   % obstacle raduius
r_ob = 6;           % observation circle radius

% Robot shape parameters
h_t = 0.3; w_t=0.2; % triangle parameters

% Draw a Circle for robot
ang=0:0.005:2*pi;

xp=r*cos(ang);
yp=r*sin(ang);

% Draw a Circle for obstacles
xp_o=r_o*cos(ang);
yp_o=r_o*sin(ang);

% Draw a Circle for observation
xp_ob=r_ob*cos(ang);
yp_ob=r_ob*sin(ang);

figure(103)
line_width = 1.5;
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

count = 1;          % Variable to Draw the predictions
n_predictions = size(vehicle_predicted_traj,1);

for k = 1:5:size(vehicle_traj,1)
    
    % Current robot position and orientation
    x1 = vehicle_traj(k,1);  y1 = vehicle_traj(k,2); th1 = vehicle_traj(k,3);
    x2 = vehicle_traj(k,10); y2 = vehicle_traj(k,11);

    % Show global map on figure
    figure(103)
    show(map); hold on

    % Plot waypoints on the global map
    for l = 1:size(waypoints,1)
        plot(waypoints(l,1),waypoints(l,2),'ob','linewidth',2);hold on
    end
    
    % Draw Path -----
    plot(path_pts(:,1),path_pts(:,2),'-k','linewidth',2); % plot the reference trajectory
    for i = 1:size(bounds,1)
        bound = bounds(i,:);
        P1 = [bound(1)+0.2,bound(2)-0.2];
        P2 = [bound(1)+0.2,bound(4)+0.2];
        P3 = [bound(3)-0.2,bound(4)+0.2];
        P4 = [bound(3)-0.2,bound(2)-0.2];
        plot([P1(1),P2(1),P3(1),P4(1),P1(1)],[P1(2),P2(2),P3(2),P4(2),P1(2)],"Color","b"); hold on
    end

    % store traveled trajectory so far -----
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on
    
    % Plot the predicted trajecttory ----

    plot(vehicle_predicted_traj(count:min(count+N+1,n_predictions),1),vehicle_predicted_traj(count:min(count+N+1,n_predictions),2),'r--*')
    count = count+5*N;

    % Create the triangle shape of the robot
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    x2_tri = [ x2+h_t*cos(th1), x2+(w_t/2)*cos((pi/2)-th1), x2-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y2_tri = [ y2+h_t*sin(th1), y2-(w_t/2)*sin((pi/2)-th1), y2+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    fill(x1_tri, y1_tri, 'r'); hold on% plot robot position
    fill(x2_tri, y2_tri, 'r'); hold on% plot robot position
    plot([x1 x2],[y1,y2],"-k" ,"LineWidth",4)

    hold off

end
end

