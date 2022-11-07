function Draw_MPC_PS_Obstacles (t,xx,xx1,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,mpc_costs, obst_pose)

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];

x_limits = [-3 10];
y_limits = [-3 10];



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

for k = 1:size(xx,2)
    h_t = 0.2; w_t=0.12; % triangle parameters
    
    x1 = xs(1); y1 = xs(2); th1 = xs(3);
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'g'); % plot reference state
    hold on;
    x1 = xx(1,k,1); y1 = xx(2,k,1); th1 = xx(3,k,1);
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

%figure
  %  subplot(211)
      %  stairs(t,u_cl(:,1),'k','linewidth',1.5); axis([0 t(end) -1 1])
       % ylabel('v (m/s)')
        %grid on
   % subplot(212)
        %stairs(t,u_cl(:,2),'r','linewidth',1.5); axis([0 t(end) -1.5 1.5])
        %xlabel('time (seconds)')
        %ylabel('\omega (rad/s)')
        %grid on
%figure
    %plot(mpc_costs,'linewidth',1.5)
    %xlabel('$x$-time steps','interpreter','latex','FontSize',fontsize_labels)
    %ylabel('$y$-MPC cost','interpreter','latex','FontSize',fontsize_labels)
    %grid on