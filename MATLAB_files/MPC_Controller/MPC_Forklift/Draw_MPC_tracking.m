function Draw_MPC_tracking (t,xx,xx1,u_cl,xs,N,rob_diam)


set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

x_limit = 10;
y_limit = 10;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];

r = rob_diam/2;  % obstacle radius
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);

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
    if k < size(xx,2) % plot prediction
        plot(xx1(1:N,1,k),xx1(1:N,2,k),'r--*')
    end
    
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'r'); % plot robot position
    %plot(x1,y1,'-sk','MarkerSize',25)% plot robot position
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
close(gcf)

figure
    subplot(211)
        stairs(t,u_cl(:,1),'k','linewidth',1.5); axis([0 t(end) -4 4])
        ylabel('v (m/s)')
        grid on
    subplot(212)
        stairs(t,u_cl(:,2),'r','linewidth',1.5); axis([0 t(end) -2 2])
        xlabel('time (seconds)')
        ylabel('\omega (rad/s)')
        grid on