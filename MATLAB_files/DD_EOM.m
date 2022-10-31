function zdot=DD_EOM(t,z)

zdot=zeros(10,1);

theta=z(5);
thetadot=z(6);
qdot=[z(2);z(4);z(6);z(8);z(10)];

alpha = 0;
alphadot = 0;

global mT mB d IT Iyy ro W T1 D;

M=[mT 0 -mB*d*sin(theta) 0 0;...
   0 mT mB*d*cos(theta) 0 0;...
   -mB*d*sin(theta) mB*d*cos(theta) IT 0 0;...
   0 0 0 Iyy 0;...
   0 0 0 0 Iyy];

B=-mB*d*thetadot^2*[cos(theta);sin(theta);0;0;0];

C=[1 0 D*sin(theta) -cos(theta+alpha)*ro 0;...
   0 1 -D*cos(theta) -sin(theta+alpha)*ro 0;...
   1 0 0 0 -cos(theta)*ro;...
   0 1 0 0 -sin(theta)*ro];

Cdot=[1 0 D*thetadot*cos(theta) sin(theta+alpha)*ro*(thetadot+alphadot) 0;...
      0 1 D*thetadot*sin(theta) -cos(theta+alpha)*ro*(thetadot+alphadot) 0;...
      1 0 0 0 sin(theta)*thetadot*ro;...
      0 1 0 0 -cos(theta)*thetadot*ro];
  
T=[0;0;0;T1;0];

lambdas=-inv(C*inv(M)*C')*(C*inv(M)*(T-B)+Cdot*qdot);

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