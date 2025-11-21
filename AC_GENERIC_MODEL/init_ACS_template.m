%% Initial scripts:
clc; close all; clear;
%% Constants definition:
KT2MS = 0.5144;
FT2M = 0.3048;
NM2M = 1852;
%% Adding paths:
addpath("01_Models\")
addpath("02_Scripts\")
addpath("03_Docs\")
addpath("04_Data\csv\template\")

%% Initialize the ACSP:
global ACSP

MASS=[150 120 120 180 180];
XCG=[21 15 40 15 40];
VCAPP=2.572+0.5144*max(119*sqrt(MASS/140),118);  

LONOLEV=[]; LATOLEV=[];

flightpar.VC=VCAPP(1);
flightpar.MASS=MASS(1);
flightpar.XCG=XCG(1);
flightpar.dZ=-5000*FT2M;
flightpar.dY=4*NM2M;

sys=ACStrim(flightpar);

%%
LONstateElim=[1 3 5 7 8 9 11 12 13 15];
LATstateElim=[2 4 6 7 8 9 10 12 14 16];
LONouputs=[11,14,5,8,3]; % Vc,Vz,q,theta,nz
LONinputs=[1 3];         % dth,de
LATouputs=[2,24,4,6,7];  % ny,beta,p,r,phi
LATinputs=[2 4];         % da,dr

sysLON1=modred(sys(LONouputs,LONinputs),LATstateElim,'truncate');
sysLON0=ss(sysLON1.a(1:4,1:4),sysLON1.a(1:4,5:6),sysLON1.c(:,1:4),sysLON1.c(:,5:6), ...
    'StateName', ["u", "w", "q", "theta"], 'InputName', ["dth", "de"], 'OutputName', ["Vc", "Vz", "q", "theta", "nz"]);

sysLAT1 = modred(sys(LATouputs,LATinputs),LONstateElim,'truncate'); 
sysLAT0 = ss(sysLAT1.a(1:4,1:4),sysLAT1.a(1:4,5:6),sysLAT1.c(:,1:4),sysLAT1.c(:,5:6), ...
    'StateName', ["v", "p", "r", "phi"], 'InputName', ["da", "dr"], 'OutputName', ["ny", "beta", "p", "r", "phi"]);

%% Longitudinal : auto-throttle & vertical autopilot
%% Longitudinal : open loop analysis
figure("Name","Longitudinal open loop time response")
step(sysLON0,25)

%% Lateral autopilot: roll and lateral acceleration autopilot