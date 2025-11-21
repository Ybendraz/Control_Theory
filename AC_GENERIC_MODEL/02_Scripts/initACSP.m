function [] = initACSP
%%
% initACSP                              Initialize the global variable ACSP
% -------------------------------------------------------------------------
% The global variable ACSP is a structured parameter with fields:
%    ACSP.U0      : initial inputs
%    ACSP.X0      : initial states
%    ACSP.Y0      : initial outputs
%    ACSP.MIG     : Mass, Intertia & Geometry of the aircraft
%    ACSP.COEF    : Aerodynamic Coefficients
%    ACSP.ATM     : Atmosphere data
%    ACSP.ENG     : Engines parameters
%    ACSP.ACT     : Actuators characteristics
%    ACSP.ILS     : ILS data
%    ACSP.RWY     : Runway data (height, slope)
%    ACSP.TURBW   : Windsteps, Turbulences & Mean Wind characteristics
%    ACSP.NOISE   : ILS noises
%    ACSP.ALS     : AutoLand System parameters
%
% CALL 
% ----
% global ACSP; initACSP;
%
% SEE ALSO
% --------
%  * ACStrim :  trimming & linearization routine for ACS model
%  * ACS     :  nonlinear simulink AirCraft Simulation model
%
% -------------------------------------------------------------------------
% SMAC Toolbox - ONERA The French Aerospace Lab, Toulouse, France
% Multi-Objective Landing Aircraft Benchmark.
% http://w3.onera.fr/smac
% -------------------------------------------------------------------------

global ACSP 
ACSP.U0=[];
ACSP.X0=[];
ACSP.Y0=[];
ACSP.MIG=[];
ACSP.COEF=[];
ACSP.ATM=[];
ACSP.ENG=[];
ACSP.ACT=[];
ACSP.ILS=[];
ACSP.RWY=[];
ACSP.TURBW=[];
ACSP.NOISE=[];
ACSP.ALS=[];


ACSP.TURBW.WindStepTime=[0 0 0];
ACSP.TURBW.WindStepMag=[0 0 0];
ACSP.TURBW.WX33=0;
ACSP.TURBW.WY33=0;
ACSP.TURBW.sigu=0;
ACSP.TURBW.sigw=0;
ACSP.TURBW.seedwx=1;
ACSP.TURBW.seedwy=2;
ACSP.TURBW.seedwz=3;
ACSP.NOISE.devLOC=0;
ACSP.NOISE.sigLOC=0;
ACSP.NOISE.sigGLD=0;
ACSP.NOISE.seedLOC=4;
ACSP.NOISE.seedGLD=5;
