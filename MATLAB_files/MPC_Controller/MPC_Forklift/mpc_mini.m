import casadi.*
clear
clc
%% Defining the CasADi variables and robot model
                                % ------------------------------------------------

T = 0.2;                    %[s]  sampling time 
N = 5;                     % prediction horizon

rob_diam = 0.3;             % robot diamter used for visualization
l = 0.2;                    % the distance between the steering wheel and the body point

v_max = 1;                % Input constraints on both linear speed and steering speed
v_min = -v_max;
omega_max = pi/2; 
omega_min = -omega_max;

x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta'); phi= SX.sym('phi');   % define the symbols for CasADi
v = SX.sym('v'); omega = SX.sym('omega');

states = [x;y;theta;phi];       % system states
n_states = length(states);

controls = [v;omega];           % system control inputs
n_controls = length(controls);

x_dot = [v*cos(theta)*cos(phi);v*sin(theta)*cos(phi);v*sin(phi)/l; omega];      % system dynamics 

f = Function('f',{states,controls},{x_dot});                                    % nonlinear mapping function f(x,u) takes the states and controls values and output the value of the x_dot vector
U = SX.sym('U',n_controls,N);                                                   % Decision variables (controls)
P = SX.sym('P',n_states + n_states);                                            % parameters (which include at the initial state of the robot and the reference state)
X = SX.sym('X',n_states,(N+1));                                                 % A vector that represents the states over the optimization problem.

%% Setting the MPC Problem 
                % -----------------------------------------------------------------

obj = 0;                                                                        % Objective function
g = [];                                                                         % constraints vector


Q = diag([5, 5, 0.1, 0.1]);                                                    % weighing matrices (states)
R = diag([1,1]);                                                          % weighing matrices (controls)
S = 10;

st  = X(:,1);                                                                   % initial state
g = [g;st-P(1:n_states)];                                                       % initial condition constraints

% Add constraints for collision avoidance

obs_x =    [0;3];         
obs_y =    [0;3];  
      
num_obs = size(obs_x);
obs_diam = 0.6;                     % for visualization



for k = 1:N
    st = X(:,k);  con = U(:,k);
    st_next = X(:,k+1);                 % get the next state symbols from the Big vector X
    f_value = f(st,con);                % get the dynamics expressed in the states st
    st_next_euler = st+ (T*f_value);    % get the next state by  euler method
    g = [g;st_next-st_next_euler];      % add the difference between the state obtained from the dynamics and the decision variable to the constraints
end

for k = 1:N+1                       % In this loop we add the condition to ensure collision free path to the constraints
    for j = 1:num_obs(1)
        g = [g ; -sqrt((X(1,k)-obs_x(j))^2+(X(2,k)-obs_y(j))^2) + (rob_diam/2 + obs_diam/2)];
    end
end


g_static = g;

OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_controls*N,1)];        % make the decision variable one column  vector

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);                % struct to define the optimization problem

opts = struct;                                                                  % struct to stablish the optimization options 
opts.ipopt.max_iter = 200;
opts.ipopt.print_level =0;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);     % The actual solver

args = struct;
                % These constraints make sure that the states will follow the dynamics
args.lbg(1:n_states*(N+1)) = 0;     % equality constraints 
args.ubg(1:n_states*(N+1)) = 0;     % equality constraints

                % These constraints make sure the robot will not collide
                % with obstacles           
args.lbg(n_states*(N+1)+1 : n_states*(N+1)+ num_obs(1)*(N+1)) = -100;   % inequality constraints
args.ubg(n_states*(N+1)+1 : n_states*(N+1)+ num_obs(1)*(N+1)) = -0.1;      % inequality constraints

                % Constraints over the states 
args.lbx(1:n_states:n_states*(N+1),1) = -30; %state x lower bound
args.ubx(1:n_states:n_states*(N+1),1) = 30;  %state x upper bound

args.lbx(2:n_states:n_states*(N+1),1) = -30; %state y lower bound
args.ubx(2:n_states:n_states*(N+1),1) = 30;  %state y upper bound

args.lbx(3:n_states:n_states*(N+1),1) = -10; %state theta lower bound
args.ubx(3:n_states:n_states*(N+1),1) = 10;  %state theta upper bound

args.lbx(4:n_states:n_states*(N+1),1) = -10; %state alpha lower bound
args.ubx(4:n_states:n_states*(N+1),1) = 10;  %state alpha upper bound

                % Constraints over the control inputs
args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = 0.0; %v lower bound
args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = v_max; %v upper bound

args.lbx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = omega_min; %omega lower bound
args.ubx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = omega_max; %omega upper bound
%----------------------------------------------



%% THE SIMULATION LOOP SHOULD START FROM HERE
                    %-------------------------------------------
t0 = 0;
x0 = [1 ; -2 ; 0.0 ; 0.0];                              % initial state.
xs = [6 ; 6 ; pi/4 ; 0.0];                           % Reference posture.

xx_hist(:,1) = x0;                                      % contains the history of states
t(1) = t0;

u0 = zeros(N,2);                                       % control inputs for  robot
X0 = repmat(x0,1,N+1)';                                % initialization of the states decision variables

sim_tim = 40;                                          % Maximum simulation time in (s)

mpciter = 0;                                           % Number of iterations

predicted_x = [];                                      % The predicted solution from the controller
u_cl=[];                                               % Input control calculated from controller

t_mpc = zeros(sim_tim / T,1);                          % Store times needed to solve the problem
mpc_cost = 0;                                          % Store total cost
mpc_costs = [];                                        % Store costs at each time step
x_x = [];
obst_pose = [];
x_obs = 2;
y_obs = 0;
ff= 1;
y_obs1 = [];
x_obs1 = [];
% Main Simulation Loop ------

while(mpciter < sim_tim/T)

g = g_static;

if x_obs >= 6
    ff= -1;
end
if x_obs <= 2
    ff = 1;
end

y_obs1 = [
];
x_obs1 = [
];

x_obs = x_obs+(T/4*ff);
[~,num_obs_dyn] = (size(x_obs1));

if ~isempty(x_obs1)
    obst_pose(mpciter+1,:) = [x_obs1, y_obs1];
end

st  = X(:,1);                                                                   % initial state
obj = 0;                                                                        % Objective function 
obstacles = ones(num_obs(1),num_obs(1))*1000;
dist_ref = 5;

w_xy_terminal = 20;
w_xy = 1.5;
w_dist = 1;
S = 2;

for k = 1:N
 st = X(:,k);  con = U(:,k);
 if  not(isempty(x_x))
     state_k = x_x(k,:);
     dist_ref = sqrt((state_k(1)-xs(1))^2+(state_k(2)-xs(2))^2);
 end
 
    if k < N
    Q = diag([w_xy, w_xy, 0.1, 0.1]);                                                    % weighing matrices (states)
    obj = obj+(st-P(n_states+1:2*n_states))'*Q*(st-P(n_states+1:2*n_states)) + con'*R*con; % calculate obj
    else
    Q = diag([w_xy_terminal, w_xy_terminal, 1, 1]);                                      % weighing matrices (states)
    obj = obj+(st-P(n_states+1:2*n_states))'*Q*(st-P(n_states+1:2*n_states)) + con'*R*con; % calculate obj
    end
    
   if not(isempty(x_x))
  
    for i = 1:num_obs
        state_k = x_x(k,:);
        obstacles(i,i) = sqrt((state_k(1)-obs_x(i))^2+(state_k(2)-obs_y(i))^2);
    end

obstacles1 = obstacles;

    for i = 1:min([num_obs(1),4])
        minimum= min(min(obstacles1));
        [min_x, min_y] = find(obstacles1==minimum);
        dist_obstacle = (st(1)-obs_x(min_x))^2+(st(2)-obs_y(min_y))^2+0.1;
        R_dash = diag([1/dist_obstacle, 0.1]) * w_dist;
        obj = obj + con' * (R_dash*num_obs(1)) * con ;
        obstacles1(min_x,min_y) = 1000;
    end

obstacles1 = obstacles;

    if dist_ref > 2

        for j = 1:min([num_obs(1),4])
                minimum= min(min(obstacles1));
                [min_x, min_y] = find(obstacles1==minimum);
                dist_obstacle = (st(1)-obs_x(min_x))^2+(st(2)-obs_y(min_y))^2+0.1;
                obj = obj + S*num_obs(1)*(1/dist_obstacle);
                obstacles1(min_x,min_y) = 1000;
        end
    else
        args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = v_min; %v lower bound
    end
   end

    for j = 1:num_obs_dyn
        g = [g ; -sqrt((X(1,k)-x_obs1(j))^2+(X(2,k)-y_obs1(j))^2) + (rob_diam/2 + obs_diam/2)];
    end

end

  
    nlp_prob.f = obj; 
    nlp_prob.g = g;
    solver = nlpsol('solver', 'ipopt', nlp_prob,opts);     % The actual solver

    args.lbg(n_states*(N+1)+ num_obs(1)*(N+1)+1:(n_states*(N+1)+ num_obs(1)*(N+1))+(num_obs_dyn*(N))) = -100;      % inequality constraints
    args.ubg(n_states*(N+1)+ num_obs(1)*(N+1)+1:(n_states*(N+1)+ num_obs(1)*(N+1))+(num_obs_dyn*(N))) = -0.2;      % inequality constraints

    args.p   = [x0;xs];                             % set the values of initial state and the reference state
                                                    % initial value of the optimization variables
    args.x0  = [reshape(X0',n_states*(N+1),1);reshape(u0',n_controls*N,1)];
    
    tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...  % the optimization problem is solved here .. 
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    t_mpc(mpciter+1) = toc;                                           % get time needed to solve the problem

                                                    % extract controls only from the solution
    u = reshape(full(sol.x(n_states*(N+1)+1:end))',n_controls,N)'; 

    x_x(:,1:n_states) = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)';
                                                    % extract the predicted TRAJECTORY
    predicted_x(:,1:n_states,mpciter+1)= reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; 
    u_cl= [u_cl ; u(1,:)];                          % get first control input
    t(mpciter+1) = t0;
    [t0, x0, u0] = shift(T, t0, x0, u,f);           % Apply control input and get the next initial state from the dynamics
    

    mpc_cost = mpc_cost + ((x0-xs)'*Q*(x0-xs)) + (u(1,:)*R*u(1,:)');        % Store the total cost
    mpc_costs(mpciter+1) = ((x0-xs)'*Q*(x0-xs)) + (u(1,:)*R*u(1,:)');       % store cost at each time step

    xx_hist(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; % extract the predicted TRAJECTORY
    X0 = [X0(2:end,:);X0(end,:)];                               % Shift trajectory to initialize the next step
    
    mpciter = mpciter + 1;
end

%% Plotting and Data logging 

ss_error = norm((x0-xs),2);
fprintf('Steady State Error: %g\n', ss_error);
fprintf('Total cost using MPC: %g\n', mpc_cost);
fprintf('Mean solver time :    %g [ms]\n', mean(t_mpc)*1e3);
fprintf('Median solver time :  %g [ms]\n', median(t_mpc)*1e3);
fprintf('Max solver time :     %g [ms]\n', max(t_mpc)*1e3);

Draw_MPC_PS_Obstacles (t,xx_hist,predicted_x,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam,mpc_costs, obst_pose)