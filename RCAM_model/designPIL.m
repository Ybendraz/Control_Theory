%########################
%     LINEAR MODELS
%########################

% Select flight point, trim & linearize
% -------------------------------------
%
%  StateName = u v w p q r x y z phi teta psi xth xa xe xr
%  InputName = dth da de dr wx wy wz
%  OutputName = nx ny nz p q r phi teta psi ...
%               alfa beta vc va vgr chi vz hlg dx dy dz
%
%  VC=70, XCG=0.21, MASS=150,
%  A/C is initialized 30 m below nominal trajectory and 20 m on the right
%
%
% Extract Longitudinal & Lateral Models
% -------------------------------------
%  
%        | inputs  : dth,de (1,3)
%  LONGI | outputs : Vc,Vz,q,theta,nz (11,14,5,8,3)
%        | states  : u,w,q,teta,xth,xe (1,3,5,11,13,15)
%
%        | inputs :  da,dr (2,4)
%  LAT   | outputs : ny,beta,p,r,phi (2,24,4,6,7)
%        | states : v,p,r,phi,xa,xr (2,4,6,10,14,16)
%

clearvars
clearvars -global
close all

%% Define Constants:

g = 9.81;
X0 = [85;       % approx 165 knots
    0;
    0; 
    0;
    0;
    0;
    0;
    0.1;        % approx 5.73 deg
    0];
XYZ0 = [0;0;0]; % Initial NED postion of the COG
U0 = [0;
        -0.1;      % approx -5.73 deg
        0;
        0.08;      % recall minimum for throttles are 0.5*pi/180 = 0.0087
        0.08];

TF = 60;

%% Defining the indexes of the longitudinal/lateral states and inputs/outputs

LONstatesElim=[1 3 5 8 9];
LATstatesElim=[2 4 6 7 9];

LONouputs=[1,3,5,8]; % u,w,q,theta
LONinputs=[2,4,5];         % de,dth1,dth2
LATouputs=[2,4,6,7];  % v,p,r,phi
LATinputs=[1 3];         % da,dr


%% Trimming the model:
run trimRCAM.m

%% Linearizing the models:
run LinearizeSymmetricDifference.m
temp = load("linear_evolution_model");
sys.a = temp.A;
sys.b = temp.B;
sys.c = eye(length(sys.a));
sys.d = zeros(size(sys.a,1),size(sys.b,2));

sys = ss(sys.a,sys.b,sys.c,sys.d);
sys.InputName = ["da","de","dr","dth1","dth2"];
sys.OutputName = ["u","v","w","p","q","r","phi","theta","psi"];

%% Extarcting the longitudinal & lateral models using model reduction (truncate method)

sysLON = modred(sys(LONouputs,LONinputs),LATstatesElim,'truncate');
damp(sysLON)
sysLAT = modred(sys(LATouputs,LATinputs),LONstatesElim,'truncate');
damp(sysLAT)

%% Time-analysis:

figure("Name","Longitudinal step response")
step(sysLON)

figure("Name","Lateral step response")
step(sysLAT)

%% Observability and controllability:
display(rank(obsv(sysLON.a,sysLON.c)))
display(rank(ctrb(sysLON.a,sysLON.b)))

display(rank(obsv(sysLAT.a,sysLAT.c)))
display(rank(ctrb(sysLAT.a,sysLAT.b)))

%  (x,y,z,phi : guidance variables) are not observable.
%  All states are controllable.

%% Control design requirements:

%% Contrôle longitudinal:

% Airspeed response:
% rise time tr = 12s & settling time ts = 45s;
% Mp<5% at altitude > 300m;
% Mp>30% at lower altitudes;
%Error_airspeed < 2.6m.s_1 for more than 5s in the presence of a wind step.
% Error = 0 due to constant wind disturbances.
% Decoupling of gamma and Vair.

[VLon,~] = eig(sysLON.a);









%% Contrôle latéral:

[VLat,~] = eig(sysLAT.a);