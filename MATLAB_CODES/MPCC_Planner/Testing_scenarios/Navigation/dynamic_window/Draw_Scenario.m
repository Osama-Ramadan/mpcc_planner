function Draw_Scenario(vehicle_traj,vehicle_predicted_traj,cmd,N,t_mpc,waypoints,path_pts,rob_diam,obs_diam,map,obst_traj,sim_step,dw_cost,vel)


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

iterations = size(vehicle_traj,1)-50;
%iterations = 5;
for k = 1:5:iterations
    
    % Current robot position and orientation
    x1 = vehicle_traj(k,1);  y1 = vehicle_traj(k,2); th1 = vehicle_traj(k,3);
    x2 = vehicle_traj(k,5); y2 = vehicle_traj(k,6);

    % Show global map on figure
    figure(103)
    show(map); hold on

    % Plot waypoints on the global map
    for l = 1:size(waypoints,1)
        plot(waypoints(l,1),waypoints(l,2),'ob','linewidth',2);hold on
    end
        plot(waypoints(1,1),waypoints(1,2),'ok','linewidth',2);hold on
        plot(waypoints(end,1),waypoints(end,2),'og','linewidth',2);hold on
    
    % Draw Path -----
    plot(path_pts(:,1),path_pts(:,2),'-k','linewidth',2); % plot the reference trajectory
    
    % Draw Obst Trajectory
    if length(obst_traj)>0
    %plot(obst_traj(1,1),obst_traj(1,2),'ok','linewidth',3);hold on
    %plot(obst_traj(end,1),obst_traj(end,2),'og','linewidth',3);hold on
    %plot(obst_traj(1:end,1),obst_traj(1:end,2),'-r','linewidth',2); % plot the reference trajectory
    end

    % store traveled trajectory so far -----
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on
    
    % Plot the predicted trajecttory ----

    %plot(vehicle_predicted_traj(count:min(count+N+1,n_predictions),1),vehicle_predicted_traj(count:min(count+N+1,n_predictions),2),'r--*')
    count = count+5*N;

    % Create the triangle shape of the robot
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    x2_tri = [ x2+h_t*cos(th1), x2+(w_t/2)*cos((pi/2)-th1), x2-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y2_tri = [ y2+h_t*sin(th1), y2-(w_t/2)*sin((pi/2)-th1), y2+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    %fill(x1_tri, y1_tri, 'r'); hold on% plot robot position
    %fill(x2_tri, y2_tri, 'r'); hold on% plot robot position
    %plot([x1 x2],[y1,y2],"-k" ,"LineWidth",4)

    hold off

end

start = 1;

 xx = vehicle_traj(start:end,:);
 gamma = xx(:,4);
 gamma = gamma*(180/pi);
 theta = xx(:,3);
 velocity = -(cmd(start:end,2));
 theta = theta*(180/pi);

 fig_num = 5;
 fig_num2 = 3;
    
 c_pts = abs(length(xx)-length(obst_traj));
 test = obst_traj(c_pts:end,1);
 dist = sqrt((xx(:,1)-obst_traj(c_pts:end-1,1)).^2 + (xx(:,2)-obst_traj(c_pts:end-1,2)).^2);
 t = (1:length(xx))*sim_step;
 
 cmds = -(movmean(cmd(start:end,2),10));
 blue_color = [0,0.45,0.74];
 brown_color = [0.64,0.08,0.18];
 orange_color = [0.85,0.33,0.1];
 green_color = [0.39,0.83,0.07];

final_pt = min(length(cmds),length(vel));
vel_cmd = cmds(40:final_pt);
act_vel = movmean(vel(40:final_pt),10);
dist_obst = movmean(dist(40:final_pt),10);
t = ((1:length(vel_cmd))*sim_step)';
figure (105)
    subplot(fig_num2,1,1)
        stairs(t,vel_cmd,'Color',blue_color,'linewidth',1.5);
        ylabel('velocity cmd (m/s)')
        grid on
    subplot(fig_num2,1,2)
        stairs(t,act_vel,'Color',blue_color,'linewidth',1.5);
        ylabel('velocity(m/s)')
        grid on
    subplot(fig_num2,1,3)
        stairs(t,dist_obst,'Color',orange_color,'linewidth',1.5);
        ylabel('Distance to Obstacles (m)')
        xlabel('Time (seconds)')
        grid on
t = ((1:length(vel_cmd))*sim_step)';
figure (106)
    subplot(fig_num2,1,1)
        stairs(t,movmean(cmd(start:length(t),1),10),'Color',blue_color,'linewidth',1.5);
        ylabel('\Omega (rad/s)')
        grid on
    subplot(fig_num2,1,2)
        stairs(t,movmean(gamma(1:length(t)),10),'Color',orange_color,'linewidth',1.5);
        ylabel('\gamma (rad)')
        grid on
    subplot(fig_num2,1,3)
        stairs(t,theta(1:length(t)),'Color',orange_color,'linewidth',1.5);
        xlabel('time (seconds)')
        ylabel('\theta (rad)')
        grid on



figure (102)
    subplot(fig_num,1,1)
        stairs(cmds,'b','linewidth',1.5);
        ylabel('velocity cmd (m/s)')
        grid on
    subplot(fig_num,1,2)
        stairs(movmean(cmd(start:end,1),10),'r','linewidth',1.5);
        ylabel('\Omega (rad/s)')
        grid on
    subplot(fig_num,1,3)
        stairs(theta,'b','linewidth',1.5);
        ylabel('\theta (rad)')
        grid on
    subplot(fig_num,1,4)
        stairs(movmean(gamma,10),'g','linewidth',1.5);
        ylabel('\gamma (rad)')
        grid on
   subplot(fig_num,1,5)
        stairs(t_mpc(start:end,:),'b','linewidth',1.5);
        xlabel('time (seconds)')
        ylabel('solver Time (ms)')
        grid on
end
