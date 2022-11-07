function Draw_MPC_PS_Obstacles (xx,xs,rob_diam)

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];

x_limits = [-10 5];
y_limits = [-10 5];



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

for k = 1:50:size(xx,1)
    h_t = 0.2; w_t=0.12; % triangle parameters
    
    x1 = xs(1); y1 = xs(2); th1 = xs(3);
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'g'); % plot reference state
    hold on;
    x1 = xx(k,1); y1 = xx(k,2); th1 = xx(k,3);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
    
    fill(x1_tri, y1_tri, 'r'); % plot robot position  
    plot(x1+xp,y1+yp,'--r'); % plot robot circle


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