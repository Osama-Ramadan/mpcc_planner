clear; clc;
T = readtable('/home/developer/ros_ws/logs/states.csv');
T1 = readtable('/home/developer/ros_ws/logs/real_state.csv');
T2 = readtable('/home/developer/ros_ws/logs/solver_time.csv');
T3 = readtable('/home/developer/ros_ws/logs/path.csv');
T4 = readtable('/home/developer/ros_ws/logs/obstacle.csv');


kin_flag = 0;

if kin_flag == 0    
    actual_x = table2array(T1(2:end,["Var2","Var3","Var4","Var5","Var6","Var7","Var8","Var9"]));
    predicted_x = table2array(T(2:end,["Var2","Var3","Var4","Var5","Var6","Var7","Var8","Var9"]));
    solver_time = table2array(T2(2:end,["Var2"]));
    path = table2array(T3(2:end,["Var2","Var3"]));
    pred_obst = table2array(T4(2:end,["Var2","Var3"]));
else
    actual_x = table2array(T1(2:end,["Var2","Var3","Var4","Var5"]));
    predicted_x = table2array(T(2:end,["Var2","Var3","Var4","Var5"]));
    solver_time = table2array(T2(2:end,["Var2"]));
    path = table2array(T3(2:end,["Var2","Var3"]));
    pred_obst = table2array(T4(2:end,["Var2","Var3"]));
end

log_n = size(solver_time,1);
obst_pose = [];
mpc_costs = 0;
obs_diam = 1;
rob_diam = 2;
obs_x = [];
obs_y = [];
N = 35;
D = 1.827;

goalsp = [-5 0; 10 0; 15 0];
u_cl = [];
t = 20;

Omni_Draw_MPC_PS_Obstacles (t,actual_x,predicted_x,u_cl,path,N,rob_diam,obs_x,obs_y,obs_diam,mpc_costs, obst_pose, solver_time, kin_flag, goalsp, pred_obst, log_n)

