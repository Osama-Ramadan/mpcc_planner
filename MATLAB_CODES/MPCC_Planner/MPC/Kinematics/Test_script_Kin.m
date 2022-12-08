
%% 4 Dynamic Obstacles Go to Goal one layer
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Kinematics/4DH.mat')
Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose,t_mpc)

%% 4 Dynamic Obstacles Go to Goal one Layer appear
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Kinematics/4DH_appearing.mat')
Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose,t_mpc)

%% 4 Dynamic Obstacles Go to Goal one Layer Disappear
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Kinematics/4DH_disappearing.mat')
Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose,t_mpc)

%% 4 Dynamic Obstacles Go to Goal one Layer with Gap
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Kinematics/4DHwG.mat')
Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose,t_mpc)

%% 4 Static Obstacles Go to Goal one Layer
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Kinematics/4SHM.mat')
Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose,t_mpc)

%% 4 Static Obstacles Go to Goal one Layer with Gap
load('/C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Kinematics/4SHMwG.mat')
Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose,t_mpc)

%% 8 Dynamic Obstacles Go to Goal 2 Layers
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Kinematics/8DH2L.mat')
Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose,t_mpc)

%% 8 Dynamic Obstacles Go to Goal 2 Layers shifted
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Kinematics/8DH2L_shifted.mat')
Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose,t_mpc)

%% 8 Static Obstacles Go to Goal 2 Layers
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Kinematics/8SH2L.mat')
Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose,t_mpc)

%% 12 Static Obstacles Go to Goal 3 Layers
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Kinematics/12SH3L.mat')
Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,obst_pose,t_mpc)


%% Tracking corner with obstacles
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/experiments/Kinematics/trackOb_corner_kin.mat')
Draw_MPC_tracking (t,x_hist,predicted_x,u_cl,Xout,N,rob_diam, obs_diam, obs_x, obs_y, t_mpc)