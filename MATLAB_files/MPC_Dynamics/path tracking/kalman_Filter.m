function [pred_x_N, estimated_x, estimated_p] = kalman_Filter(x_obs, N, dt, estimated_x_old, estimated_p_old)

u = 0.0;

A = [1,0,dt,0;...
     0,1,0,dt;...
     0,0,1,0;...
     0,0,0,1];

B = [0.5*dt^2;0.5*dt^2;dt;dt];

C = [1,0,0,0;...
     0,1,0,0];

    % Prediction
    Q_pose = 1000*0.5*dt^2;
    Q_vel = 1000*dt;
    Q = [Q_pose,0,0,0;...
            0,  Q_pose, 0, 0;...
            0,     0,   Q_vel, 0;...
            0,     0,     0,   Q_vel];
    R_noise = 0.0001;

    estimated_state = A*estimated_x_old+B*u;
    estimated_cov_s = estimated_p_old;
    estimated_cov = A*estimated_cov_s*A'+Q;

    %correction
    pos_residual = x_obs(1:2,:) - C*estimated_state;
    S = C*estimated_cov*C'+ R_noise;
    K = estimated_cov*C'*(S^-1);

    %new estimations
    estimated_x = estimated_state+K*pos_residual;
    estimated_p = (eye(4)-K*C)*estimated_cov;

    %Predicted Trajectory
    curr_state = estimated_x;
    for i = 1:N
        F = [1,0,3*dt,0;...
             0,1,0,3*dt;...
             0,0,1,0;...
             0,0,0,1];
        next_state = F*curr_state;
        pred_x_N(:,i)= next_state;
        curr_state = next_state;
    end
end