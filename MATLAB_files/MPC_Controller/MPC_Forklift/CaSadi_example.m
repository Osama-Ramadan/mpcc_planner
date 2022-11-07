import casadi.*

x = SX.sym('w');  % SX is a data type that represent matrices whose elements consists of symbolic expressions
obj = x^2-6*x+13;
obj1 = exp(0.2*x).*sin(x);

g = [];
P = [];

OPT_variables = x;
nlp_prob = struct('f', obj1, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct ;
opts.ipopt.max_iter = 1000;
opts.ipopt.print_level = 0;
opts.print_time = 0;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

args = struct ;
args.lbx = 0;
args.ubx = 4*pi;
args.lbg = -inf;
args.ubg = inf;

args.p = [];
args.x0 = 10;

sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, ...
    'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);

x_sol = full(sol.x)
min_value = full(sol.f)