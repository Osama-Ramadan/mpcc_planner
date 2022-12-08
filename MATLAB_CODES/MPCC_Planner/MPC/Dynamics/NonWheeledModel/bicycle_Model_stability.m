%% Rear car parameters
m = 1573; % [kg]
Iz = 2873; %[kg*m^2]
l = 2.68; %[m]
WF = 0.4; % front weight ratio
lR = WF*l; % distance from CG to rear axle
lF = l-lR; % distance from CG to front axle
Cf = 80000; %[N/rad] combinded conrnering stifness of front tires
Cr = 0.5*Cf; %[N/rad] combinded conrnering stifness of rear tires

V = 15; % constant velocity [m/s]

Ybeta = -(Cr + Cf);
Yr = (Cr*lR+Cf*lF)/V;
Ydelta = Cr;
Nbeta = -(Cr*lR)+(Cf*lF);
Nr = (-Cf*lF^2+Cr*lR^2)/V;
Ndelta = Cr*lR;

A = [Ybeta/(m*V), Yr/(m*V)-1; ...
    Nbeta/Iz, Nr/Iz];

B = [Ydelta/(m*V); ...
    Ndelta/Iz];

lambda1 = eig(A)

%% passenger car parameters
m = 1573; % [kg]
Iz = 2873; %[kg*m^2]
l = 2.68; %[m]
WF = 0.6; % front weight ratio
lR = WF*l; % distance from CG to rear axle
lF = l-lR; % distance from CG to front axle
Cf = 80000; %[N/rad] combinded conrnering stifness of front tires
Cr = Cf; %[N/rad] combinded conrnering stifness of rear tires

V = 50*0.44704; % constant velocity [m/s]

Ybeta = -(Cr + Cf);
Yr = (Cr*lR-Cf*lF)/V;
Ydelta = Cf;
Nbeta = Cr*lR-Cf*lF;
Nr = -(Cf*lF^2+Cr*lR^2)/V;
Ndelta = Cf*lR;

A = [Ybeta/(m*V), Yr/(m*V)-1; ...
    Nbeta/Iz, Nr/Iz];

B = [Ydelta/(m*V); ...
    Ndelta/Iz];

lambda2 = eig(A)