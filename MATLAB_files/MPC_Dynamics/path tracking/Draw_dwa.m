function Draw_dwa (trajectory,predicted_traj,N,goals,rob_diam,obs_x,obs_y,obs_diam)



x_r_1 = [];
y_r_1 = [];

x_limits = [-5 12];
y_limits = [-5 12];

% one circle
ang=0:0.005:2*pi;

% robot circle init
r = rob_diam/2; 
xp=r*cos(ang);
yp=r*sin(ang);

% obstalce circle init
r_o = obs_diam/2;
xp_o=r_o*cos(ang);
yp_o=r_o*sin(ang);

% number of obstacles
num_obs = size(obs_x,1);

% figure configurations

set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

for k = 1:size(trajectory,2)
    figure(400)
    % triangle parameters
    h_t = 0.4; w_t=0.2; 
    
    % Draw Goals Position
    for i = 1:size(goals,2)
    x1 = goals(1,i); y1 = goals(2,i);
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
    
    % Plot the obstacles
    for j = 1:num_obs
        plot(obs_x(j)+xp_o,obs_y(j)+yp_o,'--b'); hold on% plot obstacle circle   
    end

    axis([x_limits(1) x_limits(2) y_limits(1) y_limits(2)])
    pause(0.05)
    hold off 
end
