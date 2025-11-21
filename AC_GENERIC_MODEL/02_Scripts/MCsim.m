function [lstat,prob] = MCsim(filename,N,FPscale,LRpar)
%%
% MCsim                      Monte Carlo Simulations & statistical analysis
% -------------------------------------------------------------------------
%
% Based on  the nonlinear  closed-loop simulation diagram  described in the 
% Simunlink file "filename" and the  following randomly distributed parame-
% ters (see MCpar):
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
%   * sigWZ  : Vertical turbulence intensity weighting (1 =>sigWZ=0.77 m/s)
%   * sigGLD : Glide signal noise intensity weighting (1 =>sigGLD=10 muA)
%   * sigLOC : LOC signal noise intensity weighting (1 =>sigGLD=2.5 muA)
%
% this routine  runs N landings and  performs a statistical analysis on the
% following six variables: HTP60, XTP, VZTP, YTP, PHI and  SSTP in order to
% characterize the landing performance.
%
% CALL
% ----
% [lstat,prob] = MCsim(filename,N,FPscale,LRpar)
%
% INPUT ARGUMENTS
% ---------------
% filename   name of the nonlinear closed-loop simulink file
%            the simulink diagram (see ALSW.slx for details)  must  include 
%            turbulent Wind & ILS Noises blocks and should have 35 outputs.          
% N          number of simulations
% FPscale    flight parameters scale (by default 100% is applied)
%            FPscale=1 : the whole range is considered for all parameters
%            FPscale=0.5 : half range is considered for all parameters
%            FPscale.x=0.3 : 30% of the range applies to parameter x
%              Examples : FPscale.WX33=0.3 
%                             => -9kts (front) < WX33 < 3kts (tail)
%                         FPscale.WY33=1.5
%                             => -30kts (left) < WY33 < 30kts (right)
% LRpar      Limit Risks parameters
%              Example : LRpar.MASS=180 : the mass is fixed to 180
%
% OUTPUT ARGUMENTS
% ----------------
% lstat          landing statistics
% prob           corresponding probabilities 
%
% SEE ALSO
% --------
%  * ALSW    : Approach & Landing Simulation with turbulent Wind & noises
%  * MCpar   : Monte-Carlo parameters generation
%  * MCplots : Monte-Carlo Analysis & Plots
%
% -------------------------------------------------------------------------
% SMAC Toolbox - ONERA The French Aerospace Lab, Toulouse, France
% Multi-Objective Landing Aircraft Benchmark.
% http://w3.onera.fr/smac
% -------------------------------------------------------------------------

global ACSP;

if nargin<1,
    error('MCsim : no Simulink file has been specified');
elseif nargin<2,
    N=1000;
elseif nargin<3
    FPscale=1;
elseif nargin<4
    LRpar=0;
end;

scalD=FPscale;   

ALRP = MCpar(N,scalD);  % random parameters generation

% no specific windstep is applied
ACSP.TURBW.WindStepTime=[0 0 0];
ACSP.TURBW.WindStepMag=[0 0 0];

if isreal(scalD)
    ACSP.TURBW.sigw=0.77*scalD;
    ACSP.NOISE.sigLOC=2.5*scalD;
    ACSP.NOISE.sigGLD=10*scalD;
else
    if isfield(scalD,'sigWZ')
        ACSP.TURBW.sigw=0.77*scalD.sigWZ;
    else
        ACSP.TURBW.sigw=0.77;
    end;
    if isfield(scalD,'sigLOC')
        ACSP.TURBW.sigLOC=2.5*scalD.sigLOC;
    else
        ACSP.TURBW.sigLOC=2.5;
    end;
    if isfield(scalD,'sigGLD')
        ACSP.TURBW.sigGLD=10*scalD.sigGLD;
    else
        ACSP.TURBW.sigGLD=10;
    end;           
end;

LSTAT=zeros(6,N);
flightpar.dZ=-30;
flightpar.dY=20;
flightpar.Z=300;

for ii=1:N,
    fprintf('%i-',ii);
    if round(ii/20)==ii/20 fprintf('\n'); end;
    if isfield(LRpar,'MASS'),
        flightpar.MASS= LRpar.MASS;          
    else
        flightpar.MASS= ALRP.MASS(ii);
    end;
    if isfield(LRpar,'XCG'),
        flightpar.XCG=LRpar.XCG;
    else
        flightpar.XCG=ALRP.XCG(ii);
    end;
    if isfield(LRpar,'ISA'),
        flightpar.T0=LRpar.ISA+15;
    else
        flightpar.T0=ALRP.ISA(ii)+15;
    end
      
    if isfield(LRpar,'gamGLD'),
        flightpar.gamGLD=LRpar.gamGLD;
    else
        flightpar.gamGLD=ALRP.gamGLD(ii);
    end;
  
    if isfield(LRpar,'altRWY'),
        flightpar.altRWY=LRpar.altRWY;
    else
        flightpar.altRWY=ALRP.altRWY(ii);
    end;
    
    if isfield(LRpar,'gamRWY'),
        flightpar.gamRWY=LRpar.gamRWY;
    else
        flightpar.gamRWY=ALRP.gamRWY(ii);
    end;
    
    if isfield(LRpar,'WX33'),
        WX33i=LRpar.WX33;                     
    else
        WX33i=ALRP.WX33(ii);
    end;
    if isfield(LRpar,'WY33'),
        WY33i=LRpar.WY33;                     
    else
        WY33i=ALRP.WY33(ii);
    end;       
        
    flightpar.WX=1.6*WX33i; 
    Vref=0.5144*max(119*sqrt(flightpar.MASS/140),118); 
    if WX33i<0
        Vapp=Vref+max(2.572,-WX33i/3);
    else
        Vapp=Vref+2.572;
    end;
    flightpar.VC=Vapp;
    ACStrim(flightpar);    
    U33i=sqrt(WX33i^2+WY33i^2);
    ACSP.TURBW.WX33=WX33i;
    ACSP.TURBW.WY33=WY33i;
    ACSP.TURBW.sigu=0.15*U33i;
    ACSP.TURBW.seedwx=1+3*ii;
    ACSP.TURBW.seedwy=100+5*ii;
    ACSP.TURBW.seedwz=700+8*ii;
    if isfield(LRpar,'devLOC'),
        ACSP.NOISE.devLOC=LRpar.devLOC;
    else
        ACSP.NOISE.devLOC=ALRP.devLOC(ii);
    end;       
    ACSP.NOISE.seedLOC=900+3*ii;
    ACSP.NOISE.seedGLD=800+4*ii;
       
    eval(['[t,x,z]=sim(''',filename,''');']);
    [mm,ind60]=min(abs(z(:,21)-60));
    HTP60=z(ind60,16 );
    XTP=z(end,21);
    VZTP=-z(end,20)/0.3048;
    YTP=z(end,22);
    PHI=z(end,7)*57.3;
    SSTP=z(end,23)*57.3;
    LSTAT(:,ii)=[HTP60 XTP VZTP YTP PHI SSTP]';
end;
prob=MCplots(LSTAT);
lstat=LSTAT;

