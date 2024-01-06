%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D orientation estimation template
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%% affichage du parall�l�pip�de
launch3D;

%% estimation attitude
qa = 1; %Bruit d'�tat orientation
ra = 10*ACCVAR(2:3); %Bruit de mesure acc�l�ro
rg = GYRVAR(1); %Bruit de mesure gyro
Q = diag([qa qa]);
R = diag([ra' rg]);
X=[0 0]'; %Etat : rotation selon x (rad)
P = deg2rad(10)^2;

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
    % d(2:4)  = Gyroscope     X,Y,Z (�/s)
    % d(5:7)  = Accelerometer X,Y,Z (g)
    
    t=d(1);
    % Predict
    X = X;
    F = 1;
    P = F*P*F'+Q;
    % Update
    if ~isnan(t)
        Y = [d(6:7)];
        Yhat = [sin(X(1));cos(X(1));X(2)];
        H = [cos(X(1)) 0;-sin(X(1)) 0; 0 1];
        S = H*P*H'+R;
        K = P*H'*inv(S);
        X = X+K*(Y-Yhat);
        P = (eye(length(X))-K*H)*P;
    end
    
    % Update Visualisation:
    xhat(:, end+1) = [angle2quat(0, 0, X(1))'; zeros(3,1)];
    DCM_k = angle2dcm(0, 0, X(1), 'ZYX')';
    update3D;
end