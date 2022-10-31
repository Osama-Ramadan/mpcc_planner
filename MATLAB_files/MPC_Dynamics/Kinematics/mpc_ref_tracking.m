import casadi.*
clear all
%% Defining the CasADi variables and robot model
                                % -------------------------------
T = 0.2;                    %[s]  sampling time 
N = 50;                     % prediction horizon
rob_diam = 1;             % robot diamter used for visualization
obs_diam = 1;
l = 1.827;                    % the distance between the steering wheel and the body point

v_max = 0.5;                % Input constraints on both linear speed and steering speed
v_min = -v_max;
omega_max = pi/4; 
omega_min = -omega_max;

x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta'); phi= SX.sym('phi');      % define the symbols for CasADi
v = SX.sym('v'); omega = SX.sym('omega');

states = [x;y;theta;phi];       % system states
n_states = length(states);

controls = [v;omega];           % system control inputs
n_controls = length(controls);

x_dot = [v*cos(theta)*cos(phi);v*sin(theta)*cos(phi);-v*sin(phi)/l; omega];      % system dynamics 

f = Function('f',{states,controls},{x_dot});                                    % nonlinear mapping function f(x,u) takes the states and controls values and output the value of the x_dot vector
U = SX.sym('U',n_controls,N);                                                   % Decision variables (controls)
P = SX.sym('P',n_states + N*(n_states));                             % parameters (which include at the initial state of the robot and the reference state)
X = SX.sym('X',n_states,(N+1));                                                 % A vector that represents the states over the optimization problem.

%% Setting the MPC Problem 
                % --------------------------------------------------------

obj = 0;                                                                        % Objective function
g = [];                                                                         % constraints vector
Q = diag([5, 5, 5]);                                                    % weighing matrices (states)
R = diag([0.05,0.05]);                                                           % weighing matrices (controls)

st  = X(:,1);                                                                   % initial state
g = [g;st-P(1:n_states)];                                                       % initial condition constraints

for k = 1:N
    st = X(:,k);  con = U(:,k);
    st_obj = X(1:3,k);
    obj = obj+(st_obj-P(k*(n_states)+1:k*(n_states)+3))'*Q*(st_obj-P(k*(n_states)+1:k*(n_states)+3)) + (con)'*R*(con) ;
    st_next = X(:,k+1);                 % get the next state symbols from the Big vector X
    f_value = f(st,con);                % get the dynamics expressed in the states st
    st_next_euler = st+ (T*f_value);    % get the next state by  euler method
    g = [g;st_next-st_next_euler];      % add the difference between the state obtained from the dynamics and the decision variable to the constraints
end

obs_x = [0;7];
obs_y = [7;17.5];
num_obs = size(obs_x);

for k = 1:N+1                       % In this loop we add the condition to ensure collision free path to the constraints
    for j = 1:num_obs(1)
        g = [g ; -sqrt((X(1,k)-obs_x(j))^2+(X(2,k)-obs_y(j))^2) + (rob_diam/2 + obs_diam/2)];
    end
end

OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_controls*N,1)];        % make the decision variable one column  vector

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);                % struct to define the optimization problem

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);     % The actual solver

args = struct;
                % These constraints make sure that the states will follow the dynamics
args.lbg(1:n_states*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:n_states*(N+1)) = 0;  % 1e-20   % Equality constraints

args.lbg(n_states*(N+1)+1 : n_states*(N+1)+ num_obs(1)*(N+1)) = -100;   % inequality constraints
args.ubg(n_states*(N+1)+1 : n_states*(N+1)+ num_obs(1)*(N+1)) = -0.1;   % inequality constraints
               
                % Constraints over the states 
args.lbx(1:n_states:n_states*(N+1),1) = -30; %state x lower bound % new - adapt the bound
args.ubx(1:n_states:n_states*(N+1),1) = 30; %state x upper bound  % new - adapt the bound

args.lbx(2:n_states:n_states*(N+1),1) = -30; %state y lower bound
args.ubx(2:n_states:n_states*(N+1),1) = 30; %state y upper bound

args.lbx(3:n_states:n_states*(N+1),1) = -inf; %state theta lower bound
args.ubx(3:n_states:n_states*(N+1),1) = inf; %state theta upper bound

args.lbx(4:n_states:n_states*(N+1),1) = -inf; %state theta lower bound
args.ubx(4:n_states:n_states*(N+1),1) = inf; %state theta upper bound

                % Constraints over the control inputs
args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+2*N,1) = v_min; %v lower bound
args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+2*N,1) = v_max; %v upper bound

args.lbx(n_states*(N+1)+2:n_controls:n_states*(N+1)+2*N,1) = omega_min; %omega lower bound
args.ubx(n_states*(N+1)+2:n_controls:n_states*(N+1)+2*N,1) = omega_max; %omega upper bound



%% THE SIMULATION LOOP SHOULD START FROM HERE
                %-------------------------------------------



% MPC Controller 

t0 = 0;
x0 = [0 ; 0 ; pi/2 ; 0];                              % initial state.
v_ref = 2; 
omega_ref = 0;

x_hist(:,1) = x0;                                      % contains the history of states
t(1) = t0;

u0 = zeros(N,2);                                       % control inputs for  robot
X0 = repmat(x0,1,N+1)';                                % initialization of the states decision variables

sim_tim = 20;                                          % Maximum simulation time in (s)

mpciter = 0;                                           % Number of iterations

predicted_x = [];                                      % The predicted solution from the controller
u_cl=[];                                               % Input control calculated from controller

t_mpc = zeros(sim_tim / T,1);                          % Store times needed to solve the problem
mpc_cost = 0;                                          % Store total cost
mpc_costs = [];                                        % Store costs at each time step

% Genetrate the Circular Trajectory
Tinit = 0;
Tfinal = sim_tim;
Tvec=(Tinit : T : Tfinal+(N));
Xout = Generate_Trajectory(l,Tvec);

while(mpciter < length(Xout)-N) 

    current_time = mpciter*T;                           % get the current time

    args.p(1:n_states) = x0;                            % initial condition of the robot 

    for k = 1:N                                         % Update the values of the reference trajecotry
        args.p(k*(n_states)+1:k*(n_states)+4) = [Xout(k+mpciter,1), Xout(k+mpciter,2), Xout(k+mpciter,3), Xout(k+mpciter,4)];
        %args.p(k*(n_states+2)+3:k*(n_states+2)+4) = [v_ref, omega_ref];
    end
 
                                                         % initial value of the optimization variables
    args.x0  = [reshape(X0',n_states*(N+1),1);reshape(u0',n_controls*N,1)];
    
    tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...    % The problem is solver here ..                                 
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    t_mpc(mpciter+1) = toc; 

                                                    % extract controls only from the solution
    u = reshape(full(sol.x(n_states*(N+1)+1:end))',n_controls,N)'; 
                                                    % extract the predicted TRAJECTORY
    predicted_x(:,1:n_states,mpciter+1)= reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; 

    u_cl= [u_cl ; u(1,:)];                          % get first control input
    t(mpciter+1) = t0;
    [t0, x0, u0] = shift(T, t0, x0, u,f);           % Apply control input and get the next initial state from the dynamics
    x_hist(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; % extract the predicted TRAJECTORY
    X0 = [X0(2:end,:);X0(end,:)];                               % Shift trajectory to initialize the next step

    mpciter = mpciter + 1;

end
%% Plotting and Data logging 

fprintf('Mean solver time :    %g [ms]\n', mean(t_mpc)*1e3);
fprintf('Median solver time :  %g [ms]\n', median(t_mpc)*1e3);
fprintf('Max solver time :     %g [ms]\n', max(t_mpc)*1e3);

Draw_MPC_tracking (t,x_hist,predicted_x,u_cl,Xout,N,rob_diam, obs_diam, obs_x, obs_y, t_mpc)
