%% Model definition
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

%% Controller Setup
N = 40;
Q = diag([1000, 1000, 10, 10]);
R = diag([10,10]);

h = blkdiag(R, Q);
H = kron(eye(N), h);
g = zeros(N*(nx+nu),1);

Aeq = zeros(N*nx, N*(nx+nu));
beq = zeros(N*nx, 1);

aeq = [ -B              eye(nx) ;
         zeros(nx,nu)  -A       ];
for k = 1:N-1
    Aeq((k-1)*nx+1:(k+1)*nx, (k-1)*(nx+nu)+1:k*(nx+nu)) = aeq;
end
Aeq(end-nx+1:end, end-nx-nu+1:end) = aeq(1:nx,:);

acons = [  eye(nu)  zeros(nu,nx) ; 
          -eye(nu)  zeros(nu,nx) ];
Acons = kron(eye(N), acons);
bcons = kron(ones(2*N,1), u_bar);

%% Run simulation with MPC
Tf   = 2;
Nsim = ceil(Tf / T);

x_mpc = zeros(nx, Nsim+1);
u_mpc = zeros(nu, Nsim);
t_mpc = zeros(Nsim,1);
x_mpc(:,1) = x0;

cost_mpc = 0;
for k = 1:Nsim
    tic
    beq(1:nx) = A*x_mpc(:,k);
    
    z = quadprog(H, g, Acons, bcons, Aeq, beq);
    t_mpc(k)   = toc;
    if isempty(z)
        error('Quadprog failed to find a solution')
    end
    u_mpc(:,k) = clamp(z(1:nu), u_bar);
    
    odefun = @(~, x) adip_ode(x,u_mpc(:,k));
    [~, x] = ode45(odefun, [(k-1)*T, k*T], x_mpc(:,k));
    x_mpc(:,k+1) = x(end,:);
    
    cost_mpc = cost_mpc + x_mpc(:,k)'*Q*x_mpc(:,k) + u_mpc(:,k)'*R*u_mpc(:,k);
end

fprintf('Total cost using MPC: %g\n', cost_mpc);
fprintf('Mean solver time :    %g [ms]\n', mean(t_mpc)*1e3);
fprintf('Median solver time :  %g [ms]\n', median(t_mpc)*1e3);
fprintf('Max solver time :     %g [ms]\n', max(t_mpc)*1e3);

%% Plot the results
figure(1)
stairs(0:T:Tf-T, u_mpc, 'r')
yline( u_bar, 'k--')
yline(-u_bar, 'k--')
ylim(1.1*[-u_bar, u_bar])

title('Control Trajectory')
xlabel('Time t')
ylabel('Torque \tau')

figure(2)
stairs(1:Nsim, t_mpc*1e3)
title('MPC Solver Time')
xlabel('Simulation step k')
ylabel('Time t_{solv} in ms')

figure(3)
pause(0.1)
for k = 1:Nsim
    draw_adip(x_mpc(:,k), 'r')
    
    xlim([-0.1, 0.25])
    ylim([-0.1, 0.25])
    
    title('Movement of the Pendulum')
    xlabel('x Coordinate')
    ylabel('y Coordinate')
    
    drawnow limitrate
end

figure(4)
spy(H)
title('Sparsity pattern of the Hessian')

figure(5)
spy(Aeq)
title('Sparsity pattern of the constraint')
