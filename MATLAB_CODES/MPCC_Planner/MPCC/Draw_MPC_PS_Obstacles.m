function Draw_MPC_PS_Obstacles (t,xx,xx1,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam, obst_pose,t_mpc)

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];

x_limits = [-17 17];
y_limits = [-17 17];



r = rob_diam/2;  % obstacle radius
r_o = obs_diam/2;
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);

xp_o=r_o*cos(ang);
yp_o=r_o*sin(ang);

num_obs = size(obs_x,1);

figure(400)
% Animate the robot motion
%figure;%('Position',[200 200 1280 720]);
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

for k = 1:size(xx,2)
    h_t = 0.4; w_t=0.2; % triangle parameters
    
    for i = 1:size(xs,2)
    x1 = xs(1,i); y1 = xs(2,i); th1 = xs(3,i);
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'g'); % plot reference state
    hold on;
    end
    x1 = xx(1,k); y1 = xx(2,k); th1 = xx(3,k);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(xx,2) % plot prediction
        plot(xx1(1:N,1,k),xx1(1:N,2,k),'r--*')
        for j = 2:N+1
            plot(xx1(j,1,k)+xp,xx1(j,2,k)+yp,'--r'); % plot robot circle
        end
    end
    
    fill(x1_tri, y1_tri, 'r'); % plot robot position
    
    plot(x1+xp,y1+yp,'--r'); % plot robot circle
    
    for j = 1:num_obs
        plot(obs_x(j)+xp_o,obs_y(j)+yp_o,'--b'); % plot obstacle circle   
    end
        if length(obst_pose)>k
            for i = 1:(length(obst_pose(1,:)))/2
                plot(obst_pose(k,i)+xp_o,obst_pose(k,i+length(obst_pose(1,:))/2)+yp_o, '--b')
            end
        end


    hold off 

    %figure(500)
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([x_limits(1) x_limits(2) y_limits(1) y_limits(2)])
    pause(0.1)
    box on;
    grid on
    %aviobj = addframe(aviobj,gcf);
    drawnow
    % for video generation
    %F(k) = getframe(gcf); % to get the current frame
end

%close(gcf)
%viobj = close(aviobj)
%video = VideoWriter('exp.avi','Uncompressed AVI');

 %video = VideoWriter('exp.avi','Motion JPEG AVI');
 %video.FrameRate = 5;  % (frames per second) this number depends on the sampling time and the number of frames you have
 %open(video)
 %writeVideo(video,F)
 %close (video)
    
 velocity = sqrt(xx(5,:).^2 + xx(6,:).^2);
 gamma = xx(4,:);
 gamma = gamma(1:length(gamma)-1)*(180/pi);
 theta = xx(3,:);
 theta = theta(1:length(theta)-1)*(180/pi);

figure(4)
    subplot(6,1,1)
        stairs(t,u_cl(:,1),'k','linewidth',1.5);
        ylabel('Torque (N)')
       grid on
    subplot(6,1,2)
        stairs(t,u_cl(:,2),'r','linewidth',1.5);
        ylabel('\Omega (rad/s)')
        grid on
    subplot(6,1,3)
        stairs(t,velocity(1:length(velocity)-1),'b','linewidth',1.5);
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
        ylabel('Solver_time (ms)')
        grid on
