function [zdot]=ODE_fork_red(t,z)

zdot=zeros(8,1);

theta=z(3);
theta_dot=z(7);

gamma = z(4);
gamma_dot  = z(8);


qdot=[z(5);z(6);z(7);z(8)];


mB= 5;                     % mass of the chassis
d= 0.0;                      % distance from the center of the wheels to the center of mass of chassis
D=1.8;                     % half of wheel-to-wheel distance 
IB=0.5*mB*D^2;              % moment of inertia of the chassis
IT=IB+mB*d^2;

tau = 0.50;
omega = 0;
T = [tau*cos(theta+gamma);tau*sin(theta+gamma);-tau*sin(gamma)*D;omega];           % system control inputs

M=[mB                   0            mB*d*sin(theta)      0   ;...
   0                    mB          -mB*d*cos(theta)      0   ;...
   mB*d*sin(theta) -mB*d*cos(theta)         IT            0   ;...
   0                    0               0                 1  ];

B= mB*d*theta_dot^2*[cos(theta);sin(theta);0;0];

C=[sin(theta+gamma)   sin(theta) ; ...
  -cos(theta+gamma)  -cos(theta) ; ...
   (D)*cos(gamma)        0     ; ...
        0                  0     ];

C = C';

Cdot=[cos(theta+gamma)*(theta_dot+gamma_dot)       cos(theta)*theta_dot ; ...
      sin(theta+gamma)*(theta_dot+gamma_dot)       sin(theta)*theta_dot ; ...
      -(D)*sin(gamma)*gamma_dot                                 0         ; ...
                 0                                            0         ];
Cdot  = Cdot';


lambdas=-inv(C*inv(M)*C')*(C*inv(M)*(T-B)+Cdot*qdot);

temp=inv(M)*(T-B+C'*lambdas);
%temp=inv(M)*(T-B);

zdot(1)=z(5);
zdot(5)=temp(1);
zdot(2)=z(6);
zdot(6)=temp(2);
zdot(3)=z(7);
zdot(7)=temp(3);
zdot(4)=z(8);
zdot(8)=temp(4);