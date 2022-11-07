function zdot=DD_EOM_D(t,z)

zdot=zeros(10,1);

theta=z(5);
thetadot=z(6);
qdot=[z(2);z(4);z(6);z(8);z(10)];

global Lambdaa

mw=0.2; % mass of wheels
mB=3; % mass of the chassis
d=0.1; % distance from the center of the wheels to the center of mass of chassis
W=0.15; % half of wheel-to-wheel distance 
IB=0.5*mB*W^2; % moment of inertia of the chassis
ro=0.127; % radius of the wheels
t=0.01; % thickness of the wheels
Izz=mw*(3*ro^2+t^2)/12; % moment of inertia of wheels about the z-axis
Iyy=mw*(ro^2)/2; % moment of inertia of wheels about the y-axis
mT=mB+2*mw; % total mass
IT=IB+mB*d^2+2*mw*W^2+2*Izz; % total moment of inertia


M=[mT 0 -mB*d*sin(theta) 0 0;...
   0 mT mB*d*cos(theta) 0 0;...
   -mB*d*sin(theta) mB*d*cos(theta) IT 0 0;...
   0 0 0 Iyy 0;...
   0 0 0 0 Iyy];

B=-mB*d*thetadot^2*[cos(theta);sin(theta);0;0;0];

C=[cos(theta) sin(theta) 0 ro/2 -ro/2;...
   -sin(theta) cos(theta) 0 0 0;...
   0 0 1 0.5*ro/W 0.5*ro/W];

Cdot=[-sin(theta)*thetadot cos(theta)*thetadot 0 0 0;...
      -cos(theta)*thetadot -sin(theta)*thetadot 0 0 0;...
      0 0 0 0 0];


% input torques
T1=2;
T2=2;

T=[0;0;0;T1;T2];

lambdas=-inv(C*inv(M)*C')*(C*inv(M)*(T-B)+Cdot*qdot);
Lambdaa = [Lambdaa, lambdas];
temp=inv(M)*(T-B+C'*lambdas);

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