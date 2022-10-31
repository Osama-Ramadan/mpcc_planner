
%% Dynamic Model - Warehouse navigation scenario
load('C:\Users\A0072685\Documents\MasterThesisOssama\MATLAB_files\MPC_Dynamics\Omni-senarios\mpc_dynamics.mat')
Omni_Draw_MPC_PS_Obstacles (t,actual_x,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,mpc_costs, obst_pose, solver_time, 0)

%% Kinematic Model - Warehouse navigation scenario
load('C:\Users\A0072685\Documents/MasterThesisOssama/MATLAB_files/MPC_Dynamics/Omni-senarios/mpc_kinematics.mat')
Omni_Draw_MPC_PS_Obstacles (t,actual_x,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,mpc_costs, obst_pose, solver_time, 1)