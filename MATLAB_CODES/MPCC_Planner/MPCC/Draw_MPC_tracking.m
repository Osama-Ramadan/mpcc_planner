function Draw_MPC_tracking (t,xx,xx1,u_cl,xs,Orig_Xout,N,rob_diam,obs_diam,predicted_obs_poses ,t_mpc, waypoints, collision_points, map, safe_zone_f,safe_zone_b, Q_p)

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;


%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];


r = rob_diam/2;     % robot radius
r_o = obs_diam/2;   % obstacle raduius
r_ob = 6;           % observation circle radius

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
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);


for k = 1:size(xx,2)-1

    % Current robot position and orientation
    x1 = xx(1,k,1);  y1 = xx(2,k,1); th1 = xx(3,k,1);
    x2 = xx(10,k,1); y2 = xx(11,k,1);
    % Show global map on figure
    figure(103)
    show(map); hold on
    % Plot waypoints on the global map
    for l = 1:length(waypoints(:,1))
        plot(waypoints(l,1),waypoints(l,2),'ob','linewidth',2);hold on
    end

    %for l = 1:size(Q_p,1)
     %   plot(Q_p(l,1),Q_p(l,2),'xr','linewidth',2);hold on
    %end
    
    % Plot points of collision on global map
    %for l = 1:size(collision_points,2)
     %   plot(collision_points(1,l),collision_points(2,l),'xb','linewidth',2);hold on
    %end

    % Robot shape parameters
    h_t = 0.3; w_t=0.2; % triangle parameters
    
    % Draw Trajectory Modified-----
    lookahead = 1;
    spaceing = 101;
    n_seg = length(xs)/(spaceing*lookahead);
    colors = ['-g', '-c', '-y'];
    for h = 1:n_seg
    plot(xs((h-1)*101+1:(h)*101,1),xs((h-1)*101+1:(h)*101,2),'-g','linewidth',2); % plot the reference trajectory
    end

    % Draw Trajectory Original-----
    lookahead = 1;
    spaceing = 101;
    n_seg = length(Orig_Xout)/(spaceing*lookahead);
    colors = ['-c', '-y'];
    for h = 1:n_seg
    plot(Orig_Xout((h-1)*101+1:(h)*101,1),Orig_Xout((h-1)*101+1:(h)*101,2),'-k','linewidth',2); % plot the reference trajectory
    end


    % store traveled trajectory so far -----
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on
    
    % Plot the predicted trajecttory ----
    if k < size(xx1,3) % plot prediction
        plot(xx1(1:N,1,k),xx1(1:N,2,k),'r--*')
    end
    
    % Create the triangle shape of the robot
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    x2_tri = [ x2+h_t*cos(th1), x2+(w_t/2)*cos((pi/2)-th1), x2-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y2_tri = [ y2+h_t*sin(th1), y2-(w_t/2)*sin((pi/2)-th1), y2+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    fill(x1_tri, y1_tri, 'r'); hold on% plot robot position
    fill(x2_tri, y2_tri, 'r'); hold on% plot robot position

    % Plot the local map rectangle around the robot
    c = [0 0.4470 0.7410];
    for i = (k-1)*2+1:k*2
    safe_zone_k = safe_zone_f(min(i,size(safe_zone_f,1)),:);
    xLeft = safe_zone_k(1);
    yBottom = safe_zone_k(3);
    width = safe_zone_k(2) - safe_zone_k(1);
    hight = safe_zone_k(4) - safe_zone_k(3);
    %RotateArea([xLeft+width/2;yBottom+hight/2],width,hight,th1);
    %rectangle('Position', [xLeft, yBottom, width, hight], 'EdgeColor','k', 'LineWidth', 1);hold on
    end

    for i = (k-1)*2+1:2*N
    safe_zone_k = safe_zone_b(min(i,size(safe_zone_b,1)),:);
    xLeft = safe_zone_k(1);
    yBottom = safe_zone_k(3);
    width = safe_zone_k(2) - safe_zone_k(1);
    hight = safe_zone_k(4) - safe_zone_k(3);
    %RotateArea([xLeft+width/2;yBottom+hight/2],width,hight,th1);
    rectangle('Position', [xLeft, yBottom, width, hight], 'EdgeColor',c, 'LineWidth', 1);
    end
    %plot(x1+xp, y1+yp,'Color',c); hold on
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

    
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    %axis([x_limit_min x_limit_max y_limit_min y_limit_max])
    %pause(0.05)
    
    % Start Drawing the local map
    %figure(102)
    %localmat = occupancyMatrix(map);
    %resultion = map.Resolution;
    %get current robot indeceses in the global map
    %ind = local2grid(map,[x1 y1]);

    % Adapted setting the boundary for the local map
    %lower_x_boarder = min(round(x1-safe_zone_k(1))*resultion,ind(2)-1);
    %upper_x_boarder = min(round(safe_zone_k(2)-x1)*resultion,size(localmat,1)-ind(2));
    %lower_y_boarder = min(round(y1-safe_zone_k(3))*resultion,ind(1)-1);
    %upper_y_boarder = min(round(safe_zone_k(4)-y1)*resultion,size(localmat,2)-ind(1));

    % Fixed setting the boundary for the local map
    %lower_x_boarder = min(2*resultion,ind(2)-1);
    %upper_x_boarder = min(2*resultion,size(localmat,2)-ind(2));
    %lower_y_boarder = min(2*resultion,ind(1)-1);
    %upper_y_boarder = min(2*resultion,size(localmat,2)-ind(1));

    % Extract the local map
    %localmap = occupancyMap(localmat(ind(1)-upper_y_boarder:ind(1)+lower_y_boarder,ind(2)-lower_x_boarder:ind(2)+upper_x_boarder),resultion);
    %localmap.LocalOriginInWorld = [x1-2,y1-2];
    %show(localmap); hold on;
    
    % Create the triangle shape of the robot
    %x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    %y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    %for h = 1:n_seg
    %plot(xs((h-1)*spaceing*lookahead+1:(h)*spaceing*lookahead,1),xs((h-1)*spaceing*lookahead+1:(h)*spaceing*lookahead,2),colors(1),'linewidth',2); % plot the reference trajectory
    %end
    %fill(x1_tri, y1_tri, 'r'); hold on % plot robot position
    % Plot the predicted trajecttory ----
    %if k < size(xx1,3) % plot prediction
        %plot(xx1(1:N,1,k),xx1(1:N,2,k),'r--*')
    %end
    %if k < size(predicted_obs_poses,3)
        %predicted_obs_pos = predicted_obs_poses(:,:,k);
        %plot(predicted_obs_pos(1,1)+xp_o,predicted_obs_pos(2,1)+yp_o,'-r','LineWidth',2); hold on % plot obstacle circle   
        %for j = 2:size(predicted_obs_pos,2)
            %plot(predicted_obs_pos(1,j)+xp_o,predicted_obs_pos(2,j)+yp_o,'-b', 'LineWidth',0.5); hold on % plot obstacle circle   
        %end
    %end
    hold off
    
    %figure(103)
    %hold off

end

 xx = xx(:,1:length(xx)-1);
 velocity = sqrt(xx(5,:).^2 + xx(6,:).^2);
 gamma = xx(4,:);
 gamma = gamma*(180/pi);
 theta = xx(3,:);
 theta = theta*(180/pi);

figure
    subplot(6,1,1)
        stairs(t,u_cl(:,1),'k','linewidth',1.5);
        ylabel('Torque (N)')
       grid on
    subplot(6,1,2)
        stairs(t,u_cl(:,2),'r','linewidth',1.5);
        ylabel('\Omega (rad/s)')
        grid on
    subplot(6,1,3)
        stairs(t,velocity,'b','linewidth',1.5);
        ylabel('velocity (m/s)')
        grid on
    subplot(6,1,4)
        stairs(t,gamma,'g','linewidth',1.5);
        ylabel('\gamma (rad)')
        grid on
    subplot(6,1,5)
        stairs(t,theta,'b','linewidth',1.5);
        ylabel('\theta (rad)')
        grid on
   subplot(6,1,6)
        stairs(t,t_mpc*1000,'b','linewidth',1.5);
        xlabel('time (seconds)')
        ylabel('solver_Time (ms)')
        grid on