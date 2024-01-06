%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D orientation estimation template
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; close all;
%% CONFIGURATION and INITIALISATION
dt=0.01;
COM='COM7';
useSimulation = true;
useSimulation_visuals = useSimulation;
%% CALIBRATION
% To re-run calibration, just clear MAG0 or run the calibration script
% separately:
if ~exist('MAG0', 'var')
    runcalibration;
end
%% affichage du parallélépipède
launch3D;

%% estimation attitude
qa = 10; %Bruit d'état orientation
ra = 10; %Bruit de mesure accéléro
rg = 1; %Bruit de mesure gyro
Q = diag([qa]);
R = diag([rg]);

X=[0]'; %Etat : rotation selon x (rad)
Error = [];
Kgain = [];
P = deg2rad(10)^2;
Covariance_trace = [sum(diag(P))];
tp=0;
ii=0;
obs = [];
xtrue = [];
xhat = [];
if useSimulation
    imu411('open',COM, 'simimu_2Dmaneuver');
else
    imu411('open',COM);
end

while(1)
    ii=ii+1;
    % Read sensor
    [d, xt] = imu411('read'); %cette lecture est bloquante et cadence la boucle a 0.01s
    obs(:, end+1) = d;
    xtrue(:, end+1) = xt;

    % Rappel :
    % d(1)    = Time                (s)
    % d(2:4)  = Gyroscope     X,Y,Z (°/s)
    % d(5:7)  = Accelerometer X,Y,Z (g)
    % d(8:10) = magnetometer  X,Y,Z (T)
    % d(11)   = barometer           (mbar)
    % d(12)   = ultrasound          (cm)
    t=d(1);
    % Predict
    F = 1;
    X = X;
    P = F*P*F'+Q;
    % Update
    if ~isnan(t)
        Y = d(2);
        Yhat = X;
        H = 1;
        S = H*P*H'+R;
        K = P*H'/S;
        X = X+K*(Y-Yhat);
        P = (eye(length(X))-K*H)*P;
    end
    
    Kgain = [Kgain K];
    Error = [Error norm(K*(Y-Yhat))];
    Covariance_trace = [Covariance_trace sum(diag(P))];
    
    % Update Visualisation:
    xhat(:, end+1) = [angle2quat(0, 0, X)'; zeros(3,1)];
    DCM_k = angle2dcm(0, 0, X, 'ZYX')';
    update3D;
end


% Plot the error as well as the state-estimate covariance trace:
% figure()
% subplot(1,3,1)
% title("Inovation/Observation error")
% plot(Error)
% xlabel("samples")
% ylabel("Error's value")
% 
% subplot(1,3,2)
% title("State-estimate covariance")
% plot(Covariance_trace)
% xlabel("samples")
% ylabel("State-estimate covariance")
% 
% subplot(1,3,3)
% title("Kalman gain")
% plot(Kgain)
% xlabel("samples")
% ylabel("Kalman's gain norm")