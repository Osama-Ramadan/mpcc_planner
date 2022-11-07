clear; clc;
T = readtable('/home/developer/isaac_oss/sdk/apps/Test/IgoNeo_MPC/Components/states.csv');
T1 = readtable('/home/developer/isaac_oss/sdk/apps/Test/IgoNeo_MPC/Components/real_state.csv');

actual_x = table2array(T1(2:end,["Var2","Var3","Var4","Var5"]));
predicted_x = table2array(T(2:end,["Var2","Var3","Var4","Var5"]));

obst_pose = [];
mpc_costs = 0;
obs_diam = 1;
rob_diam = 1;
obs_x = [];
obs_y = [];
N = 70;

xs = [5; -13; 0.0 ; 0.0];
u_cl = [];
t = 0;
Omni_Draw_MPC_PS_Obstacles (t,actual_x,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,mpc_costs, obst_pose)