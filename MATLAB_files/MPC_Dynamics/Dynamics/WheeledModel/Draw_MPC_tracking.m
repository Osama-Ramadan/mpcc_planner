function Draw_MPC_tracking (t,xx,xx1,u_cl,xs,N,rob_diam,obs_diam ,obs_x, obs_y)

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

x_limit = 20;
y_limit = 20;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];

num_obs = size(obs_x,1);

r = rob_diam/2;  % obstacle radius
r_o = obs_diam/2;
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);

xp_o=r_o*cos(ang);
yp_o=r_o*sin(ang);

figure(500)
% Animate the robot motion
%figure;%('Position',[200 200 1280 720]);
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

for k = 1:size(xx,2)

    h_t = 0.5; w_t=0.35; % triangle parameters

    plot(xs(:,1),xs(:,2),'-g','linewidth',1.2);hold on % plot the reference trajectory
    x1 = xx(1,k,1); y1 = xx(2,k,1); th1 = xx(3,k,1);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(xx1,3) % plot prediction
        plot(xx1(1:N,1,k),xx1(1:N,2,k),'r--*')
    end
    
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'r'); % plot robot position
    %plot(x1,y1,'-sk','MarkerSize',25)% plot robot position

    for j = 1:num_obs
        plot(obs_x(j)+xp_o,obs_y(j)+yp_o,'-r'); % plot obstacle circle   
    end

    hold off
    %figure(500)
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([-x_limit x_limit -y_limit y_limit])
    pause(0.2)
    box on;
    grid on
    %aviobj = addframe(aviobj,gcf);
    drawnow
end
%close(gcf)

 xx = xx(:,1:length(xx)-1);
 velocity = sqrt(xx(5,:).^2 + xx(6,:).^2);
 gamma = xx(4,:);
 gamma = gamma*(180/pi);
 theta = xx(3,:);
 theta = theta*(180/pi);

figure
    subplot(5,1,1)
        stairs(t,u_cl(:,1),'k','linewidth',1.5);
        ylabel('Torque (N)')
       grid on
    subplot(5,1,2)
        stairs(t,u_cl(:,2),'r','linewidth',1.5);
        ylabel('\Omega (rad/s)')
        grid on
    subplot(5,1,3)
        stairs(t,velocity,'b','linewidth',1.5);
        ylabel('velocity (m/s)')
        grid on
    subplot(5,1,4)
        stairs(t,gamma,'g','linewidth',1.5);
        ylabel('\gamma (rad)')
        grid on
    subplot(5,1,5)
        stairs(t,theta,'b','linewidth',1.5);
        xlabel('time (seconds)')
        ylabel('\theta (rad)')
        grid on