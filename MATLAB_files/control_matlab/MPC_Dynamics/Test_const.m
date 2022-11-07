Test_num = 50;
q_dd = [];
Cdd = [];
for i=1:length(xx_hist)-1
    [tem] = ODE_fork(xx_hist(:,i)', u_cl(i,:));
    q_dd = [q_dd,tem];
end
q_dd = q_dd';
%state = Z(Test_num,:);
diff_acc = [];
diff_vel = [];

for j = 1:length(xx_hist)-1
state = xx_hist(:,j);
acc = q_dd(j,:);
[diff, diff1] = constraints_test(state',acc);
diff_acc = [diff_acc, diff];
diff_vel = [diff_vel, diff1];
end
%plot(sum(diff_acc,1)/4)
stairs(sum(diff_vel,1)/4,'b','linewidth',1.5);
grid on;
axis([0 length(diff_vel) -0.1 0.1]);
