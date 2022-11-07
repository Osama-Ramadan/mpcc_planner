function [zdot]=DD_EOM(t,z)


zdot=zeros(10,1);

theta=z(5);
thetadot=z(6);

if z(1) > 60
    T1=0;  
else
    T1 =5;
end
T1 = 2.00;
T=[0;0;0;T1;0];

qdot=[z(2);z(4);z(6);z(8);z(10)];

alpha = 0;
alphadot = 0;

mw=0.2; % mass of wheels
mB=3; % mass of the chassis
d=0.0; % distance from the center of the wheels to the center of mass of chassis
D = 0.5;
IB=0.5*mB*D^2; % moment of inertia of the chassis
ro=0.5; % radius of the wheels
mT=mB; % total mass
IT=IB+mB*d^2;
Iyy=mw*(ro^2)/2;
mx = mB*d+mw*D;

M=[mT                   0            mx*sin(theta)      0   0;...
   0                    mT          -mx*cos(theta)      0   0;...
   mx*sin(theta) -mx*cos(theta)         IT              0   0;...
   0                    0               0               Iyy 0;...
   0                    0               0               0  Iyy];

B= mx*thetadot^2*[cos(theta);sin(theta);0;0;0];

C=[1 0 D*sin(theta)  -cos(theta+alpha)*ro 0;...
   0 1 -D*cos(theta) -sin(theta+alpha)*ro 0;...
   1 0 0 0 -cos(theta)*ro;...
   0 1 0 0 -sin(theta)*ro];

Cdot=[0 0 D*thetadot*cos(theta) sin(theta+alpha)*ro*(thetadot+alphadot) 0;...
      0 0 D*thetadot*sin(theta) -cos(theta+alpha)*ro*(thetadot+alphadot) 0;...
      0 0 0 0  sin(theta)*thetadot*ro;...
      0 0 0 0 -cos(theta)*thetadot*ro];


lambdas=-inv(C*inv(M)*C')*(C*inv(M)*(T-B)+Cdot*qdot);

temp=inv(M)*(T-B+C'*lambdas);
%temp=inv(M)*(T-B);

zdot(1)=z(2);
zdot(2)=temp(1);
zdot(3)=z(4);
zdot(4)=temp(2);
zdot(5)=z(6);
zdot(6)=temp(3);
zdot(7)=z(8);
zdot(8)=temp(4);
zdot(9)=z(10);
zdot(10)=temp(5);