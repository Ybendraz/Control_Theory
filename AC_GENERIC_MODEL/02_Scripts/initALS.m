function [] = initALS
%%
% initALS                           Initialize Baseline AutoLand Parameters
% -------------------------------------------------------------------------
% The Autoland Parameters are stored in ACSP.ALS
% Note that initACSP must be called first.
%
% CALL 
% ----
% initALS;
%
% SEE ALSO
% --------
%  * initACSP    : initialization of ACSP
%  * ACStrim     : trimming & linearization routine for ACS model
%  * ACS         : nonlinear simulink AirCraft Simulation model
%  * ALS         : closed-loop with Baseline AutoLand Controller
%
% -------------------------------------------------------------------------
% SMAC Toolbox - ONERA The French Aerospace Lab, Toulouse, France
% Multi-Objective Landing Aircraft Benchmark.
% http://w3.onera.fr/smac
% -------------------------------------------------------------------------

global ACSP;

ACSP.ALS.KLON=[ -0.0005   0.0113  -0.0279  -0.0222   0         0;
                  0.0100 -0.0344  -0.0185   0.1296   4.1864    0.1198 ];
ACSP.ALS.KLAT=[ -0.2892  -0.4220  -0.3183   1.1855  -1.6355   1.9629;
                 0.1111  -0.1029  -0.2264  -0.3743   2.0005   0.0996];
ACSP.ALS.HLON= [ 0  -0.0015 ; 0       0.0056 ];     
ACSP.ALS.HLAT= [ 0   0.0570 ; 0.1020  0];   
ACSP.ALS.kpGLD = 0.1300;
ACSP.ALS.kpLOC = 6.0000e-04;
ACSP.ALS.kdLOC = 0.0120;
ACSP.ALS.HFL = 15;
ACSP.ALS.HDC = 9;

