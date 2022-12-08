
%% Parameters Values ----
vR = 1.7;
Lb = 1.3;
Ll = 0.0;
g = 9.81;
hl = 1.2;
hb = 0.6;
mB = 1300;
ml = 100;
w = 0.4;
D = 1.827;
theta_dd = 0.5;
theta_d = 0;

%% Equations ---- 
% Longitodinal Stability ----
Ml = (Lb*mB+Ll*ml)*g;

acc_max = abs((Ml-(hb*mB))/(hl*ml));

% Lateral Stability ----
Db = w/2*((D-Lb)/D);
Dl = w/2*((D-Ll)/D);
Mt = (Db*mB+Dl*ml)*g;

direction = sign(1);
theta_dd = -direction*theta_dd;
k_max = (Mt + direction*(theta_dd*(hl*ml*Ll+hb*mB*Lb)))/(vR^2*(hl*ml+hb*mB));
gamma_max = atan2(k_max*D,1)*(180/pi);