
%% State Space Model and Matrices
% A = @(x1,x2)[x1,x1*x2;x1^2,x2^2]

A = @(Ts, v_d, l, theta_d, phi_d)[1 0 -Ts*sin(theta_d)*cos(phi_d)*v_d     -Ts*cos(theta_d)*sin(phi_d)*v_d ;
                                  0 1  Ts*cos(theta_d)*cos(phi_d)*v_d     -Ts*sin(theta_d)*sin(phi_d)*v_d ;
                                  0 0                 1                    Ts*cos(phi_d)*v_d/l                ;
                                  0 0                 0                               1
                                 ];

B = @(Ts, v_d, l, theta_d, phi_d)[Ts*cos(theta_d)*cos(phi_d)     0 ;
                                  Ts*sin(theta_d)*cos(phi_d)     0 ;
                                  Ts*sin(phi_d)/l                0 ;
                                           0                    Ts ;
                                ];
nx     = size(A(0, 0, 0, 0,0), 1);
nu     = size(B(0, 0, 0, 0,0), 2);

x0    = [0; 0; 0; atan(1)];
u_bar  = [5 ; 2];

%% Path Generation

% Circular Path
v_d = 1;
l   = 1;
Ts = 0.1;

Tinit = 0;
Tfinal = 10;

Tvec=[Tinit : Ts : Tfinal];
X0 = [0,0,0,atan(1)];
[Tout, Xout] = ode45(@circle_trajectory, Tvec, X0);
% plotting your results
%figure(1)
plot(Tout, Xout);
plot(Tout, Xout, 'linewidth',1.5)
title('Time response of the system','fontsize',16)
xlabel('time (s)')
ylabel('magnitude of states')
%figure(2)
plot(Xout(:,1),Xout(:,2))
xlabel('x','fontsize',14)
ylabel('y','fontsize',14)
title('Plot of x vs y - Ref. Trajectory')

%% MPC Controller Settings

N = 100;
Q = diag([1000, 1000, 10, 10]);
R = diag([10,10]);

% YALMIP variables
U     = sdpvar(repmat(nu,1,N),ones(1,N));
X     = sdpvar(repmat(nx,1,N),ones(1,N));
x_hat = sdpvar(nx,1);

constraints = [];
objective   = 0;

% Objective and Constraints equations
x = x_hat;
for k = 1:N
    A_lin = A(Ts,v_d,l, Xout(k,3), Xout(k,4));
    B_lin = B(Ts,v_d,l, Xout(k,3), Xout(k,4));
    x = A_lin*x + B_lin*U{k};
    objective   = objective + x'*Q*x + U{k}'*R*U{k};
    constraints = [constraints,-u_bar <= U{k} <= u_bar, x_hat == A_lin*x + B_lin*U{k}];
end

% Prepare Solver
opts = sdpsettings();
opts.quadprog.Algorithm = 'interior-point-convex';
controller = optimizer(constraints, objective, opts, x_hat, U{1});

%% Simulation
Nsim  = ceil(Tfinal / Ts);

x_mpc = zeros(nx, Nsim+1);
u_mpc = zeros(nu, Nsim);
t_mpc = zeros(Nsim,1);
x_mpc(:,1) = x0;

cost_mpc = 0;

for k = 1:Nsim
     tic
     [uk, err] = controller{x_mpc(:,k)};
     t_mpc(k)  = toc;
     
     if err ~= 0
        error('YALMIP can not find a solution')
     end
     u_mpc(:,k) = clamp(uk, u_bar);
     odefun = @(~, x) fork_ode(x,uk);
     [~, x] = ode45(odefun, [(k-1)*Ts, k*Ts], x_mpc(:,k));
     x_mpc(:,k+1) = x(end,:);
     
     cost_mpc = cost_mpc + x_mpc(:,k)'*Q*x_mpc(:,k) + u_mpc(:,k)'*R*u_mpc(:,k);
end

fprintf('Total cost using MPC: %g\n', cost_mpc);
fprintf('Mean solver time :    %g [ms]\n', mean(t_mpc)*1e3);
fprintf('Median solver time :  %g [ms]\n', median(t_mpc)*1e3);
fprintf('Max solver time :     %g [ms]\n', max(t_mpc)*1e3);

%% Plot Results
figure(1)
stairs(0:Ts:Tfinal-Ts, u_mpc(1,:), 'r')
yline( u_bar, 'k--')
yline(-u_bar, 'k--')
ylim(1.1*[-u_bar(1), u_bar(1)])
title('Control Trajectory')
xlabel('Time t')
ylabel('Linear Speed')

figure(2)
stairs(1:Nsim, t_mpc*1e3)
title('MPC Solver Time')
xlabel('Simulation step k')
ylabel('Time t_{solv} in ms')

figure(3)
plot(x_mpc(1,:),x_mpc(2,:))
xlabel('x','fontsize',14)
ylabel('y','fontsize',14)
title('Plot of x vs y - Ref. Trajectory')
