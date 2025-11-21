%%
% MCPLOTS                     Aircraft Landing Monte-Carlo Plots & Analysis
% -------------------------------------------------------------------------
%
% Based on Gaussian approximations, from a given set of landing simulations
% this routine computes the following 6 probabilities:
%  p1 = prob(HTP60 < 0)
%  p2 = prob(XTP > 915 m)
%  p3 = prob(VZTP > 10 ft/s)
%  p4 = prob(|YTP| > 15 m)
%  p5 = prob(|PHI| > 12 deg)
%  p6 = prob(|SSTP| > 14 deg)
%  p7 = prob(VZTP > 12 ft/s)
% with the standard notation:
%  HTP60 : height of the wheels 60m after runway threshold      (m)
%  XTP   : x-coordinate at touchdown point                      (m)
%  VZTP  : vertical speed at touchdown point                    (ft/s)
%  YTP   : lateral deviation at touchdown point                 (m)
%  PHI   : roll angle at touchdown point                        (deg)
%  SSTP  : main landing gear sidelip angle at touchdown point   (deg)
%
% By default, the routine also plots the corresponding histograms, gaussian
% interpolations and cumulative distribution functions.
%
% The average/limit risks to be demonstrated are :
%    * pi_avr < 10^-6, pi_lim < 10^-5     for i = 1,2,4,6
%    * p3_avr < 10^-6, P7_lim < 10^-5 
%    * p5_avr < 10^-8, p5_lim < 10^-7   
%
% CALLS
% -----
% proba = MCplots(L);
% proba = MCplots(L,opt);
%
% INPUT ARGUMENTS
% ---------------
%  L     6-by-N matrix describing N landings with N >= 100
%        L(1,i)=HTP60i, L(2,i)=XTPi, L(3,i)=VZTPi
%        L(4,i)=YTPï,   L(5,i)=PHIi, L(6,i)=SSTPi
%  opt   optional boolean variable
%        opt=1 : plots are displayed (default setting)
%        opt=0 : no plot (this option may be used for control optimization)
%
% OUTPUT ARGUMENT
% ---------------
%  proba   6-by-1 vector:  [p1 p2 p3 p4 p5 p6]'
%
%
% SEE ALSO
% --------
%  * MCsim   Monte-Carlo Simulations
%  * ACS     nonlinear simulink AirCraft Simulation model
%  * ALS2    Aircraft Landing Simulation with Turbulences & Noises Inputs
%
% -------------------------------------------------------------------------
% SMAC Toolbox - ONERA The French Aerospace Lab, Toulouse, France
% Multi-Objective Landing Aircraft Benchmark.
% http://w3.onera.fr/smac
% -------------------------------------------------------------------------
