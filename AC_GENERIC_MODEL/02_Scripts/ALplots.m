function [] = alplots(t,z,nfig)
%%
% alplots                                          Approach & Landing Plots
% -------------------------------------------------------------------------
%
% After approach & landing simulation is performed with ALS1 or ALS2:
%   [t,~,z]=sim('ALS1');
%   [t,~,z]=sim('ALS2');
% this routine plots the main results on two figures.
%
% CALLS 
% -----
% alplots(t,z)
% alplots(t,z,nfig)
%
% INPUT ARGUMENTS
% ---------------
% t  N-by-1 time history vector
% z  N-by-36 simulation outputs vector (see ALS1, ALS2).
%
%
% SEE ALSO
% --------
%  * ALS1      Simulink Model without Turbulence & Noise
%  * ALS2      Simulink Model with Turbulence & Noise
%
% -------------------------------------------------------------------------
% SMAC Toolbox ONERA The French Aerospace Lab, Toulouse, France
% Multi-Objective Landing Aircraft Benchmark.
% Author: J-M. Biannic
% Last revision: February 2016 by J-M. Biannic
% http://w3.onera.fr/smac
% -------------------------------------------------------------------------
global ACSP
tmax=max(t);

if nargin<3
    figure; clf; 
else
    figure(nfig); clf;
end;

subplot(321); plot(t,z(:,11)); 
xlabel('Time (sec)'); ylabel('Vc (m/s)'); grid;
axis([0 tmax min(z(:,11)-1) max(z(:,11))+1]);

subplot(322); plot(t,z(:,20)); 
xlabel('Time (sec)'); ylabel('VzLG (m/s)'); grid;
axis([0 tmax min(z(:,20))-0.5 max(z(:,20))+0.5]);

subplot(323); nz=z(:,3)/9.81; plot(t,nz); 
xlabel('Time (sec)'); ylabel('Nz (g)'); grid;
axis([0 tmax min(nz)-0.05 max(nz)+0.05]);

subplot(324); zi=z(:,8)*57.3; plot(t,zi); 
xlabel('Time (sec)'); ylabel('\theta (deg)'); grid;
axis([0 tmax min(zi)-0.5 max(zi)+0.5]);

subplot(325); zic=z(:,25)+ACSP.U0.epr; zi=z(:,26)+ACSP.U0.epr;
plot(t,zic); hold on; plot(t,zi,'r'); 
axis([0 tmax min([zic;zi])-0.05 max([zic;zi])+0.05]);
xlabel('Time (sec)'); ylabel('EPR'); grid;


subplot(326); zic=z(:,29)*57.3; zi=z(:,30)*57.3;
plot(t,zic); hold on; plot(t,zi,'r'); 
axis([0 tmax min([zic;zi])-1 max([zic;zi])+1]);
xlabel('Time (sec)'); ylabel('dE (deg)'); grid;

subplot(321);
title('                                                  LONGITUDINAL VARIABLES',...
    'FontSize',14);




if nargin<3
    figure; clf; 
else
    figure(nfig+1); clf;
end;

subplot(321); zi=z(:,7)*57.3; plot(t,zi);
xlabel('Time (sec)'); ylabel('\phi (deg)'); grid;
axis([0 tmax min(zi)-0.5 max(zi)+0.5]);

subplot(322); zi=z(:,9)*57.3; plot(t,zi);
xlabel('Time (sec)'); ylabel('\psi (deg)'); grid;
axis([0 tmax min(zi)-0.5 max(zi)+0.5]);

subplot(323); ny=z(:,4); plot(t,ny); 
xlabel('Time (sec)'); ylabel('Ny (m/s^2)'); grid;
axis([0 tmax min(ny)-0.01 max(ny)+0.01]);

subplot(324); zi=z(:,24)*57.3; plot(t,zi); 
xlabel('Time (sec)'); ylabel('\beta (deg)'); grid;
axis([0 tmax min(zi)-0.5 max(zi)+0.5]);

subplot(325); zic=z(:,27)*57.3; zi=z(:,28)*57.3;
plot(t,zic); hold on; plot(t,zi,'r'); 
axis([0 tmax min([zic;zi])-1 max([zic;zi])+1]);
xlabel('Time (sec)'); ylabel('dA (deg)'); grid;

subplot(326);  zic=z(:,31)*57.3; zi=z(:,32)*57.3;
plot(t,zic); hold on; plot(t,zi,'r'); 
axis([0 tmax min([zic;zi])-1 max([zic;zi])+1]);
xlabel('Time (sec)'); ylabel('dR (deg)'); grid;

subplot(321);
title('                                                 LATERAL VARIABLES',...
    'FontSize',14);


if nargin<3
    figure; clf; 
else
    figure(nfig+2); clf;
end;


subplot(311); xi=z(:,21); zi=z(:,16); plot(xi,zi);
[~,indH15]=min(abs(z(:,16)-15));
[~,indX0]=min(abs(xi));
z0r=z(indX0,16)-z(indX0,19);
x0r=xi(indX0);
zr=max(z0r-ACSP.ILS.tanggld*(xi-x0r),0);
hold on; plot(xi,zr,'r--'); 
axis([min(xi) max(xi) min([zi;zr])-1 max([zi;zr])+1]);
legend('Achieved trajectory','Glide reference','Location','NorthEast');
xlabel('D (m)'); ylabel('HLG (m)'); grid;


subplot(312); 
plot([min(xi) max(xi)],[0 0],'g','LineWidth',2);
hold on; zi=z(:,22); plot(xi,zi);
zi=z(:,18); plot(xi,zi,'r--'); 
axis([min(xi) max(xi) min(zi)-1 max(zi)+1]);
legend('Runway axis','Y_{LG}',' \DeltaY_{loc}','Location','NorthEast');
xlabel('D (m)'); ylabel('Y_{LG} , \DeltaY_{loc} (m)'); grid;

subplot(313); zi=z(:,19); plot(xi,zi); 
axis([min(xi) max(xi) min(zi)-1 max(zi)+1]);
hold on; plot([0 0],[min(zi) max(zi)],'r','LineWidth',2);
xlabel('D (m)'); ylabel('\DeltaZ_{glide} (m)'); grid;

subplot(311); 
title('TRAJECTORY','FontSize',14);



winds=z(:,[33 34 35]);

if nargin<3
    figure; clf;
else
    figure(nfig+3); clf;
end;

subplot(311); zi=z(:,33);
plot(t,zi);
axis([0 tmax min(zi)-1 max(zi)+1]);
xlabel('Time (sec)'); ylabel('Wx (m/s)'); grid;
title('WIND INPUTS','FontSize',14);

subplot(312); zi=z(:,34);
plot(t,zi);
axis([0 tmax min(zi)-1 max(zi)+1]);
xlabel('Time (sec)'); ylabel('Wy (m/s)'); grid;

subplot(313); zi=z(:,35);
plot(t,zi);
axis([0 tmax min(zi)-1 max(zi)+1]);
xlabel('Time (sec)'); ylabel('Wz (m/s)'); grid;


