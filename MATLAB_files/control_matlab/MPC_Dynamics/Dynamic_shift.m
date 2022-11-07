function [x0] = Dynamic_shift(dt,x0,con,f)
u = con(1,:);
st = x0(1:6);
st_dot = x0(7:length(x0));
f_value = f(x0,u);
st_dot(1:5) = full(st_dot(1:5) + (dt*f_value(1:5)));
st_dot(6) = full(f_value(6));
x0(7:length(x0))=full(st_dot);
st = st+(dt*st_dot);
x0(1:6) = full(st);
end