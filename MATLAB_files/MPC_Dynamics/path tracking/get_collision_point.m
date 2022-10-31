function [Xc,Tc] = get_collision_point(Xv,Xo,Nv,path,seg,dt)
% Predicted Vehicle Trajectory
    x_veh = Xv(1); y_veh = Xv(2); xd_veh = Xv(5); yd_veh = Xv(6);
    curr_veh_state = [x_veh;y_veh;xd_veh;yd_veh];
    Xc = 1000;
    Tc = -1;
    for i = 1:Nv
        F = [1,0,dt,0;...
             0,1,0,dt;...
             0,0,1,0;...
             0,0,0,1];
        next_state = F*curr_veh_state;
        pred_v_st(:,i)= next_state;
        curr_veh_state = next_state;
    end
% Predicted Obstacle Trajectory
    pred_obs_st = Xo;
    v_speed =sqrt(Xv(5)^2+Xv(6)^2); 
    o_speed = sqrt(Xo(3,1)^2+Xo(4,1)^2);

% Geometrical representation
    rv = 0.5;
    ro = 0.5;

% Check for collision
    for i = (seg-1)*100+1:(seg)*100+1
        for j = size(pred_obs_st,2):-1:1
            dist = sqrt((path(i,1)-pred_obs_st(1,j))^2+(path(i,2)-pred_obs_st(2,j))^2);
            if dist < rv+ro
                %Cx = (pred_v_st(1,i)*ro + pred_obs_st(1,j)*rv)/(rv+ro);
                %Cy = (pred_v_st(2,i)*ro + pred_obs_st(2,j)*rv)/(rv+ro);

                Cx = pred_obs_st(1,j);
                Cy = pred_obs_st(2,j);
                
                tv = sqrt((pred_v_st(1,1)-Cx)^2+(pred_v_st(2,1)-Cy)^2)/v_speed;
                to = sqrt((pred_obs_st(1,1)-Cx)^2+(pred_obs_st(2,1)-Cy)^2)/o_speed;
                Tc = [tv;to];
                Xc = [Cx;Cy];
                return;
            end
        end
    end
end