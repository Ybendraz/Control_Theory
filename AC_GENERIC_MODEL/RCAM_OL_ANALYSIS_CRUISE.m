%% linearization point:
% Linearize around a given altitude, speed, angular rates, and attitude
% angles.
% + around an initial control inputs setting: dT,dE,dA,dR
% We will consider multiple trim points within the flight envelope Env =
% f(h,V,Mach)
% For simplification purposes, one point will be considered for an initial
% analysis and we will proceed based on its results to other trim points.

%% Constant definition:
CONV_FT2M = 0.3048;
CONV_KT2MS = 0.514444;
CONV_DEG2RAD = deg2rad(1);

run RCAM_model_trim.m  
% Decoupling criteria is verified
%% OPEN loop eigenvalue analysis :

[V_long,Diag_long,W_long] = eig(RCAM_LIN_LONG.a);
disp("Eigen values of the longitudinal linear model:")
damp(RCAM_LIN_LONG)
disp("absolute value of eigen vectors: ")
abs(V_long)

figure("Name","Zeros & poles of longitudinal linear model")
pzmap(RCAM_LIN_LONG);grid on;

[V_lat,Diag_lat,W_lat] = eig(RCAM_LIN_LAT.a);
disp("Eigen values of the lateral linear model:")
damp(RCAM_LIN_LAT)
disp("absolute value of eigen vectors: ")
abs(V_lat)

figure("Name","Zeros & poles of lateral linear model")
pzmap(RCAM_LIN_LAT);grid on;


%% 

%% OPEN loop frequency response

%% Time response - Longitudinal model
% Testing a +/-1deg elevetor doublet from the trim position
Elevator_doublet = 1; %
Throttle_man = 0;
Tsim = 60; 
sim("RCAM_OP_LONG.slx")
% Plot results:
t = ans.tout;

u1 = ans.simU_LONGI.Data(:,1);
u2 = ans.simU_LONGI.Data(:,2);
u3 = ans.simU_LONGI.Data(:,3);

y1 = ans.simY_LONGI.Data(:,1);
y2 = ans.simY_LONGI.Data(:,2);
y3 = ans.simY_LONGI.Data(:,3);
y4 = ans.simY_LONGI.Data(:,4);

figure("Name","Longitudinal inputs")
subplot(1,3,1)
plot(t,u1)
grid on
xlabel("Time(s)")
ylabel("\deltaT: elevator deflection")
subplot(1,3,2)
plot(t,u2)
grid on
xlabel("Time(s)")
ylabel("\deltaTh1: throttle one's deflection")
subplot(1,3,3)
plot(t,u3)
grid on
xlabel("Time(s)")
ylabel("\deltaTh2: throttle two's deflection")

figure("Name","Longitudinal outputs")
subplot(2,2,1)
plot(t,y1)
grid on
xlabel("Time(s)")
ylabel("\DeltaV: Airspeed")
subplot(2,2,2)
plot(t,y2)
grid on
xlabel("Time(s)")
ylabel("\Delta\alpha: AoA")
subplot(2,2,3)
plot(t,y3)
grid on
xlabel("Time(s)")
ylabel("\Delta\gamma: flight path")
subplot(2,2,4)
plot(t,y4)
grid on
xlabel("Time(s)")
ylabel("\DeltaQ: pitch rate")


% Testing a 1->0 deg throttles positionning:

Elevator_doublet = 0; %
Throttle_man = 1;
Tsim = 200; 
sim("RCAM_OP_LONG.slx")
% Plot results:
t = ans.tout;

u1 = ans.simU_LONGI.Data(:,1);
u2 = ans.simU_LONGI.Data(:,2);
u3 = ans.simU_LONGI.Data(:,3);

y1 = ans.simY_LONGI.Data(:,1);
y2 = ans.simY_LONGI.Data(:,2);
y3 = ans.simY_LONGI.Data(:,3);
y4 = ans.simY_LONGI.Data(:,4);

figure("Name","Longitudinal inputs")
subplot(1,3,1)
plot(t,u1)
grid on
xlabel("Time(s)")
ylabel("\deltaT: elevator deflection")
subplot(1,3,2)
plot(t,u2)
grid on
xlabel("Time(s)")
ylabel("\deltaTh1: throttle one's deflection")
subplot(1,3,3)
plot(t,u3)
grid on
xlabel("Time(s)")
ylabel("\deltaTh2: throttle two's deflection")

figure("Name","Longitudinal outputs")
subplot(2,2,1)
plot(t,y1)
grid on
xlabel("Time(s)")
ylabel("\DeltaV: Airspeed")
subplot(2,2,2)
plot(t,y2)
grid on
xlabel("Time(s)")
ylabel("\Delta\alpha: AoA")
subplot(2,2,3)
plot(t,y3)
grid on
xlabel("Time(s)")
ylabel("\Delta\gamma: flight path")
subplot(2,2,4)
plot(t,y4)
grid on
xlabel("Time(s)")
ylabel("\DeltaQ: pitch rate")

