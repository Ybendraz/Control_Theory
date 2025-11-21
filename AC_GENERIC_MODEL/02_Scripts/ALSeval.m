%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        AUTOLAND EVALUATION                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
global ACSP
initACSP;
initALS;

% ----------------------------------------------------------
% SIMULATION #1 : Nominal Case without wind
% ----------------------------------------------------------

clear flightpar
flightpar.MASS=150;
flightpar.XCG=21;
flightpar.VC=2.572+0.5144*max(119*sqrt(flightpar.MASS/140),118);
flightpar.WX=0;
flightpar.altRWY=0;
flightpar.gamRWY=0;
flightpar.T0=15;
flightpar.Z=300;
flightpar.gamK=-3;
flightpar.gamGLD=-3;
flightpar.dZ=-30;
flightpar.dY=50;
ACStrim(flightpar);

[t,~,z]=sim('ALS');
nfig=1;
ALplots(t,z,nfig);


% ----------------------------------------------------------
% SIMULATION #2 : Nominal Case with deterministic wind steps
% ----------------------------------------------------------
ACSP.TURBW.WindStepTime=[30 50 70];
ACSP.TURBW.WindStepMag=[-8 5 1];
[t,~,z]=sim('ALS');
nfig=1;
ALplots(t,z,nfig);


% ----------------------------------------------------------
% SIMULATION #3 : Nominal Case with turbulences & ILS noise
% ----------------------------------------------------------

WX33=-30*0.5144; 
WY33=20*0.5144;   
MASS=150;

Vref=0.5144*max(119*sqrt(MASS/140),118);
if WX33<0
    Vapp=Vref+max(2.572,-WX33/3);
else
    Vapp=Vref+2.572;
end;

clear flightpar
flightpar.MASS=MASS;
flightpar.XCG=21;
flightpar.VC=Vapp;
flightpar.WX=1.6*WX33; 
flightpar.altRWY=0;
flightpar.gamRWY=0;
flightpar.T0=15;
flightpar.Z=300;
flightpar.gamK=-3;
flightpar.gamGLD=-3;
flightpar.dZ=-30;
flightpar.dY=50;

ACStrim(flightpar);

ACSP.TURBW.WindStepTime=[30 50 70];
ACSP.TURBW.WindStepMag=[0 0 0];
ACSP.TURBW.WX33=WX33;
ACSP.TURBW.WY33=WY33;
ACSP.TURBW.sigu=0.15*sqrt(WX33^2+WY33^2);
ACSP.TURBW.sigw=0.77;
ACSP.TURBW.seedwx=1;
ACSP.TURBW.seedwy=2;
ACSP.TURBW.seedwz=3;
ACSP.NOISE.devLOC=0;
ACSP.NOISE.sigLOC=2.5;
ACSP.NOISE.sigGLD=10;
ACSP.NOISE.seedLOC=4;
ACSP.NOISE.seedGLD=5;

% run simulation and plot the results
[t,~,z]=sim('ALS');
nfig=1;
ALplots(t,z,nfig);


% ----------------------------------------------------------
% SIMULATION #4 : Nominal Case with turbulences &  Windsteps
% ----------------------------------------------------------
WX33=-30*0.5144; 
WY33=20*0.5144; 
Vref=0.5144*max(119*sqrt(flightpar.MASS/140),118);
if WX33<0
    Vapp=Vref+max(2.572,-WX33/3);
else
    Vapp=Vref+2.572;
end;

clear flightpar
flightpar.MASS=150;
flightpar.XCG=21;
flightpar.VC=Vapp;
flightpar.WX=1.6*WX33; 
flightpar.altRWY=0;
flightpar.gamRWY=0;
flightpar.T0=15;
flightpar.Z=300;
flightpar.gamK=-3;
flightpar.gamGLD=-3;
flightpar.dZ=-30;
flightpar.dY=50;

ACStrim(flightpar);

ACSP.TURBW.WindStepTime=[30 50 70];
ACSP.TURBW.WindStepMag=[-10 15 1];
ACSP.TURBW.WX33=WX33;
ACSP.TURBW.WY33=WY33;
ACSP.TURBW.sigu=0.15*sqrt(WX33^2+WY33^2);
ACSP.TURBW.sigw=0.77;
ACSP.TURBW.seedwx=1;
ACSP.TURBW.seedwy=2;
ACSP.TURBW.seedwz=3;
ACSP.NOISE.devLOC=0;
ACSP.NOISE.sigLOC=0;
ACSP.NOISE.sigGLD=0;
ACSP.NOISE.seedLOC=4;
ACSP.NOISE.seedGLD=5;

[t,~,z]=sim('ALS');
nfig=1;
ALplots(t,z,nfig);


% ----------------------------------------------------------
% SIMULATION #5 : Max. Mass, with turbulences & ILS noise 
%                 + descending runway
% ----------------------------------------------------------

% set mean wind at 33ft to trong level
MASS=180;
WX33=-30*0.5144; % max level head wind
WY33=20*0.5144;   % max level lateral wind
Vref=0.5144*max(119*sqrt(MASS/140),118);
if WX33<0
    Vapp=Vref+max(2.572,-WX33/3);
else
    Vapp=Vref+2.572;
end;

clear flightpar
flightpar.MASS=MASS;
flightpar.XCG=21;
flightpar.VC=Vapp;
flightpar.WX=1.6*WX33; 
flightpar.altRWY=0;
flightpar.gamRWY=-2;
flightpar.T0=15;
flightpar.Z=300;
flightpar.gamK=-3;
flightpar.gamGLD=-3;
flightpar.dZ=-30;
flightpar.dY=50;

ACStrim(flightpar);

ACSP.TURBW.WX33=WX33;
ACSP.TURBW.WY33=WY33;
ACSP.TURBW.sigu=0.15*sqrt(WX33^2+WY33^2);
ACSP.TURBW.sigw=0.77;
ACSP.TURBW.seedwx=1;
ACSP.TURBW.seedwy=2;
ACSP.TURBW.seedwz=3;
ACSP.NOISE.devLOC=0;
ACSP.NOISE.sigLOC=2.5;
ACSP.NOISE.sigGLD=10;
ACSP.NOISE.seedLOC=4;
ACSP.NOISE.seedGLD=5;

[t,~,z]=sim('ALS');
nfig=1;
ALplots(t,z,nfig);


% ----------------------------------------------------------
%    MONTE-CARLO :  AVERAGE RISKS WITH REDUCED DISPERSION
% ----------------------------------------------------------
clear all
global ACSP;
initACSP;
initALS;

N=100;
FPscale=0.2; 
[lstat,prob] = MCsim('ALS',N,FPscale);


% ----------------------------------------------------------
%    MONTE-CARLO :  AVERAGE RISKS WITH STANDARD DISPERSION
% ----------------------------------------------------------
clear all
global ACSP;
initACSP;
initALS;

N=100;
FPscale=1;
[lstat,prob] = MCsim('ALS',N,FPscale);



% ----------------------------------------------------------
%    MONTE-CARLO :  AVERAGE RISKS WITH DOUBLED WY33
% ----------------------------------------------------------
N=1000;
clear FPscale
FPscale.WY33=2;
[lstat,prob] = MCsim('ALS',N,FPscale);


% ----------------------------------------------------------
%    MONTE-CARLO :  LIMIT RISKS WITH MAX & MIN WEIGHT
% ----------------------------------------------------------
N=1000;
clear FPscale
FPscale=1;
LRpar.MASS=180;
[lstat,prob] = MCsim('ALS',N,FPscale,LRpar);

LRpar.MASS=120;
[lstat,prob] = MCsim('ALS',N,FPscale,LRpar);

