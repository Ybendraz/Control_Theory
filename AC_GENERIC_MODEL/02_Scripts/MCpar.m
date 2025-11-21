%%
% MCpar               Parameters Random Generation for Monte-Carlo Analysis
% -------------------------------------------------------------------------
%
% This routine randomly generates parameters for approach-and-landing Monte
% Carlo Simulations.  The  distributions for each  parameter affecting  the
% landing performances are as follows:
%   * WX33   : normal, m=7.5kts, sig=7.3kts, min=10T(+10),max=30H(-30)
%   * WY33   : normal, m=0kts,   sig=7kts,   min=-20, max=+20
%   * MASS   : uniform, min=120t, max=180t
%   * XCG    : uniform, min=0.15, max=0.40
%   * ISA    : uniform, min=-69deg, max=40deg
%   * gamGLD : normal, m=3deg, sig=0.075%, min=-2.85deg, max=-3.15deg
%   * devLOC : normal, m=0, sig=2.5muA, min=-5 muA, max=5muA
%   * gamRWY : normal, m=0, sig=0.4%, min=-2%, max=2%
%   * altRWY : specific 
%             [-1000ft,250ft] : 50.00%
%             [250ft,750ft]   : 28.33%
%             [750ft,1250ft]  : 13.33%
%             [1250ft,1750ft] :  3.33%
%             [1750ft,2500ft] :  1.67%
%             [2500ft,3500ft] :  1.00%
%             [3500ft,4500ft] :  0.67%
%             [4500ft,9200ft] :  1.67%
%
% The optional parameter "scalings" permits to shrink or expand the distri-
% bution of each parameter by the same factor or a specific one (see below)
%
% CALLS 
% -----
% [ALRP] = MCpar(N,scalings)
% [ALRP] = MCpar(N)
%
% INPUT ARGUMENTS
% ---------------
% N         number of samples
% scalings  optional scaling coefficients (by default, scalings=1)
%           this scaling parameter is either:
%           - a positive real scalar: the same factor applies to all
%           - a structured variable with fields.
%               Example: scalings.WX33 applies specifically to WX33
%
% OUTPUT ARGUMENTS
% ----------------
% ALRP           structured variable with fields containing all parameters
%   ALRP.WX33    Longitudinal Wind (m/s) 
%   ALRP.WY33    Lateral Wind (m/s) 
%   ALRP.MASS    Mass  (t) 
%   ALRP.XCG     Center-of-Gravity Location (%)
%   ALRP.ISA     Temperature variations vs ISA (deg)
%   ALRP.gamGLD  GLIDE Slope (deg)
%   ALRP.devLOC  LOCALIZER deviation (muA)
%   ALRP.gamRWY  Runway Slope (%)
%   ALRP.altRWY  Runway Altitude (m)
%
% SEE ALSO
% --------
%  * MCsim   : Monte-Carlo Simulations
%  * MCplots : Monte-Carlo Analysis & Plots
%
% -------------------------------------------------------------------------
% SMAC Toolbox - ONERA The French Aerospace Lab, Toulouse, France
% Multi-Objective Landing Aircraft Benchmark.
% http://w3.onera.fr/smac
% -------------------------------------------------------------------------
