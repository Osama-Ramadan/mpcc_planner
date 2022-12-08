function [x0, u0] = dynamic_red_shift(dt,x0,con,f,D)
u = con(1,:);
st = x0(1:4);
st_dot = x0(5:length(x0)-3);
f_value = f(x0,u);
st_dot(1:3) = full(st_dot(1:3) + (dt*f_value(1:3)));
st_dot(4) = full(f_value(4));
x0(5:length(x0)-3)=full(st_dot);
st = st+(dt*st_dot);
x0(1:4) = full(st);

x0(9) = full(x0(9)+u(3));

x0(10) = x0(1)-D*cos(x0(3));
x0(11) = x0(2)-D*sin(x0(3));
u0 = [con(2:size(con,1),:);con(size(con,1),:)];
end