function [zdot]=DD_EOM(t,z)


zdot=zeros(12,1);

theta=z(3);
thetadot=z(9);

if z(1) > 60
    T1=0;  
else
    T1 =5;
end
T1 = 1.00;
T=[0;0;0;T1;0;0];

qdot=[z(7);z(8);z(9);z(10);z(11); z(12)];

alpha = z(6);
alphadot = z(12);

mw=1; % mass of wheels
mB=50; % mass of the chassis
d=-0.5; % distance from the center of the wheels to the center of mass of chassis
D = 3;
IB=0.5*mB*D^2; % moment of inertia of the chassis
ro=0.5; % radius of the wheels
mT=mB+2*mw; % total mass
IT=IB+mB*d^2;
Iyy=mw*(ro^2)/2;
mx = mB*d+mw*D;

Fn = mT*9.81*0.2;

M=[mT                   0            mx*sin(theta)      0   0   0;...
   0                    mT          -mx*cos(theta)      0   0   0;...
   mx*sin(theta) -mx*cos(theta)         IT              0   0   0;...
   0                    0               0               Iyy 0   0;...
   0                    0               0               0  Iyy  0;...
   0                    0               0               0   0   1];

Df = diag([Fn*cos(theta),Fn*sin(theta),0,0,0])*[sign(z(2));sign(z(4));0;0;0];

B= (mx*thetadot^2*[cos(theta);sin(theta);0;0;0;0]);

C=[1 0 D*sin(theta)  -cos(theta+alpha)*ro 0  0;...
   0 1 -D*cos(theta) -sin(theta+alpha)*ro 0  0;...
   1 0 0 0 -cos(theta)*ro  0;...
   0 1 0 0 -sin(theta)*ro  0];

Cdot=[0 0 D*thetadot*cos(theta) sin(theta+alpha)*ro*(thetadot+alphadot) 0   0 ;...
      0 0 D*thetadot*sin(theta) -cos(theta+alpha)*ro*(thetadot+alphadot) 0  0;...
      0 0 0 0  sin(theta)*thetadot*ro  0;...
      0 0 0 0 -cos(theta)*thetadot*ro  0];




lambdas=-inv(C*inv(M)*C')*(C*inv(M)*(T-B)+Cdot*qdot);

temp=inv(M)*(T-B+C'*lambdas);
%temp=inv(M)*(T-B);

zdot(1)=z(7);
zdot(7)=temp(1);
zdot(2)=z(8);
zdot(8)=temp(2);
zdot(3)=z(9);
zdot(9)=temp(3);
zdot(4)=z(10);
zdot(10)=temp(4);
zdot(5)=z(11);
zdot(11)=temp(5);
zdot(6)=z(12);
zdot(12)=0;