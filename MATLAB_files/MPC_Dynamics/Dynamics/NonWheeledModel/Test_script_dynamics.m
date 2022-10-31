
%% 2 Obstacles Go to Goal Failing
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Dynamics/2SH_FAIL.mat')
Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose,t_mpc)

%% 2 Obstacles Go to Goal Success
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Dynamics/2SH_WORK.mat')
Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose,t_mpc)

%% Corner Tarcking with 2 Obstacles
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Dynamics/trackOb_corner_dyn.mat')
Draw_MPC_tracking (t,x_hist,predicted_x,u_cl,Xout,N,rob_diam, obs_diam,obs_x, obs_y, t_mpc)