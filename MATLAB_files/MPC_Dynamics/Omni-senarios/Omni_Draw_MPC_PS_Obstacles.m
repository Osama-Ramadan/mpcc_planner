function Omni_Draw_MPC_PS_Obstacles (t,xx,xx1,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,mpc_costs, obst_pose, t_mpc, model_flag, waypoints, pred_obs, log_n)

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];

x_limits = [-6 16];
y_limits = [-6 16];



r = rob_diam/2;  % obstacle radius
r_o = obs_diam/2;
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);

xp_o=r_o*cos(ang);
yp_o=r_o*sin(ang);

num_obs = size(obs_x,1);

figure(500)
% Animate the robot motion
%figure;%('Position',[200 200 1280 720]);
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

count = 1;
pred_hor = size(pred_obs,1)/log_n;
for k = 1:(size(xx,1)-1)

    for l = 1:length(waypoints(:,1))
        plot(waypoints(l,1),waypoints(l,2),'ob','linewidth',2);hold on
    end
    h_t = 0.4; w_t=0.2; % triangle parameters
    
    plot(xs(:,1),xs(:,2),'-g','linewidth',1.5);hold on % plot the reference trajectory
    %plot(xs(61:end,1),xs(61:end,2),'-c','linewidth',1.5);hold on % plot the reference trajectory

    x1 = xx(k,1); y1 = xx(k,2); th1 = xx(k,3);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
    plot(xx1(count:count+N+1,1),xx1(count:count+N+1,2),'r--*');hold on
    
    
    fill(x1_tri, y1_tri, 'r'); % plot robot position
    
    plot(x1+xp,y1+yp,'--r'); % plot robot circle
    
    x_obst = pred_obs((k)*pred_hor+1:(k+1)*pred_hor,1);
    y_obst = pred_obs((k)*pred_hor+1:(k+1)*pred_hor,2);
    plot(x_obst,y_obst,'--b*'); hold on % plot robot circle
    
    for j = 1:num_obs
        plot(obs_x(j)+xp_o,obs_y(j)+yp_o,'--b'); % plot obstacle circle   
    end
        if length(obst_pose)>k
            for i = 1:(length(obst_pose(1,:)))/2
                plot(obst_pose(k,i)+xp_o,obst_pose(k,i+length(obst_pose(1,:))/2)+yp_o, '--b'); hold on
            end
        end


    hold off 

    %figure(500)
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([x_limits(1) x_limits(2) y_limits(1) y_limits(2)])
    pause(0.05)
    box on;
    grid on
    %aviobj = addframe(aviobj,gcf);
    drawnow
    % for video generation
    %F(k) = getframe(gcf); % to get the current frame
    count = count + N+1;
end
 xx = xx';
 gamma = xx(4,:);
 gamma = gamma(1:length(gamma))*(180/pi);
 theta = xx(3,:);
 theta = theta(1:length(theta))*(180/pi);

if model_flag == 0 
    velocity = sqrt(xx(5,:).^2 + xx(6,:).^2);
else
    velocity = 0;
end

t = 1:size(t_mpc,1);

figure
    subplot(4,1,1)
        stairs(t,velocity,'b','linewidth',1.5);
        ylabel('velocity (m/s)')
        grid on
    subplot(4,1,2)
        stairs(t,gamma,'g','linewidth',1.5);
        ylabel('\gamma (rad)')
        grid on
    subplot(4,1,3)
        stairs(t,theta,'b','linewidth',1.5);
        ylabel('\theta (rad)')
        grid on
    subplot(4,1,4)
        stairs(t,t_mpc*1000,'b','linewidth',1.5);
        ylabel('Solver Time (ms)')
        grid on