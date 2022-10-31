function [x0, u0] = dynamic_red_shift(dt,x0,con,f)
u = con(1,:);
st = x0(1:4);
st_dot = x0(5:length(x0));
f_value = f(x0,u);
st_dot(1:3) = full(st_dot(1:3) + (dt*f_value(1:3)));
st_dot(4) = full(f_value(4));
x0(5:length(x0))=full(st_dot);
st = st+(dt*st_dot);
x0(1:4) = full(st);

u0 = [con(2:size(con,1),:);con(size(con,1),:)];
end