%%
% ACStrim       Parameters Setting, Triming & Linearization of the Aircraft
% -------------------------------------------------------------------------
%
% According to the chosen flight parameters defined by the different fields
% of the  structured "flightpar" input  variable, this  function trims  the
% nonlinear aircraft simulation model (ACS) and computes  all the  required
% parameters to perform approach and landing.  These are stored in a global
% structured variable ACSP. Based on the standard linmod routine,  a linear
% model about the trimmed conditions is also optionnaly provided.
%
% CALLS 
% -----
% global ACSP;                      % define ACSP as a global variable
% ACStrim(flightpar);               % trim and store results in ACSP 
% [sys,x0,y0]=ACStrim(flightpar);   % trim & linearize
%
% INPUT ARGUMENTS
% ---------------
%  * flightpar : flight conditions structured variable with fields
%    .MASS (t)    A/C mass                   : [120,180]    (default = 150)
%    .XCG (%)     CoG location               : [15, 41]      (default = 21)
%    .VC (m/s)    Conventional Airspeed      : [60,90]       (default = 70)
%    .WX(m/s)     Initial x-Wind Input (Wx)  : [-25,10]       (default = 0)
%    .altRWY(m)   Runway elevation           : [-300, 2800]   (default = 0)
%    .gamRWY(%)   Runway slope               : [-2, 2]        (default = 0)
%    .T0(degC)    Temperature at sealevel    : [-55,55]      (default = 15)
%    .Z(m)        Initial LG height / runway : [0,300]      (default = 300)
%    .gamK(deg)   Initial Kinematic slope    : [-6,0]        (default = -3)
%    .gamGLD(deg) Glide Slope                : [-3.15,-2.85] (default = -3)
%    .dZ(m)       Vertical deviation / glide : [-30,30]       (default = 0)
%    .dY(m)       Lateral deviation / loc    : [-30,30]       (default = 0)
%
%
% OUTPUT ARGUMENTS
% ----------------
%  * sys      linearized system in state-space object format
%             16 states : V,OM,X,PHI,ENGine,ACTuators
%                V=[ub vb wb]', OM=[p q r]'
%                X=[x y z]',    PHI=[phi theta psi]'
%                ENG=xEng,      ACT=[xa xe xr]'
%  * x0       structured trimmed state vector (size=16)
%  * y0       structured trimmed output vector (size=24)
%
% SEE ALSO
% --------
%  * ACS    :  nonlinear simulink AirCraft Simulation model
%  * ALS    :  Approach & Landing Simulation model
%
% -------------------------------------------------------------------------
% SMAC Toolbox - ONERA The French Aerospace Lab, Toulouse, France
% Multi-Objective Landing Aircraft Benchmark.
% http://w3.onera.fr/smac
% -------------------------------------------------------------------------
