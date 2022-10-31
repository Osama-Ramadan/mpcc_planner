function Draw_dwa (trajectory,u,predicted_traj,N,goals,rob_diam,obs_diam, map,Xout,predicted_obs_poses)

x_r_1 = [];
y_r_1 = [];

x_limits = [-5 12];
y_limits = [-5 12];

% one circle
ang=0:0.005:2*pi;

% robot circle init
r = rob_diam/2; 
r_ob = 5;           % observation circle radius
xp=r*cos(ang);
yp=r*sin(ang);

% Draw a Circle for observation
xp_ob=r_ob*cos(ang);
yp_ob=r_ob*sin(ang);

% obstalce circle init
r_o = obs_diam/2;
xp_o=r_o*cos(ang);
yp_o=r_o*sin(ang);

% number of obstacles


% figure configurations
figure(400)
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

for k = 1:size(trajectory,2)
    figure(400)
    show(map); hold on
    % triangle parameters
    h_t = 0.4; w_t=0.2; 
    
    % Draw Goals Position
    for i = 1:size(goals,1)
    x1 = goals(i,1); y1 = goals(i,2);
    plot(x1, y1, 'go', LineWidth=2); % plot reference state
    hold on;
    end
    
    % get robot position and formulate triangle
    x1 = trajectory(1,k); y1 = trajectory(2,k); th1 = trajectory(3,k);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    plot(x_r_1,y_r_1,'-r','linewidth',1.5);hold on
    fill(x1_tri, y1_tri, 'r'); hold on % plot robot position
    plot(x1+xp,y1+yp,'--r'); hold on% plot robot circle

    % Plot the predicted trajecttory ----
    if k < size(predicted_traj,3) 
        plot(predicted_traj(1,1:N,k),predicted_traj(2,1:N,k),'r--*')
    end

    % Plot Generated Path
    plot(Xout(:,1),Xout(:,2),'-k','linewidth',2); % plot the reference trajectory
    
    % Plot Dynamic Obstacle Observation Circle
    o = [0.8500 0.3250 0.0980];
    plot(x1+xp_ob, y1+yp_ob,'Color',o); hold on

    % Plot Dynamic Obstacles
    if k < size(predicted_obs_poses,3)
        predicted_obs_pos = predicted_obs_poses(:,:,k);
        plot(predicted_obs_pos(1,1)+xp_o,predicted_obs_pos(2,1)+yp_o,'-r','LineWidth',2); hold on % plot obstacle circle   
        for j = 2:size(predicted_obs_pos,2)
            plot(predicted_obs_pos(1,j)+xp_o,predicted_obs_pos(2,j)+yp_o,'-b', 'LineWidth',0.5); hold on % plot obstacle circle   
        end
    end
    
    % Plot the obstacles
    %for j = 1:num_obs
     %   plot(obs_x(j)+xp_o,obs_y(j)+yp_o,'--b'); hold on% plot obstacle circle   
    %end

    %axis([x_limits(1) x_limits(2) y_limits(1) y_limits(2)])
    pause(0.05)
    hold off 
end

%% Get the Paramters Graphs
 gamma = trajectory(4,:);
 gamma = gamma*(180/pi);
 theta = trajectory(3,:);
 theta = theta*(180/pi);

figure(401)
    subplot(4,1,1)
        stairs(u(1,:),'k','linewidth',1.5);
        ylabel('Speed (m/s)')
       grid on
    subplot(4,1,2)
        stairs(u(2,:),'r','linewidth',1.5);
        ylabel('\Omega (rad/s)')
        grid on
    subplot(4,1,3)
        stairs(gamma,'g','linewidth',1.5);
        ylabel('\gamma (rad)')
        grid on
    subplot(4,1,4)
        stairs(theta,'b','linewidth',1.5);
        ylabel('\theta (rad)')
        grid on
