%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3D orientation estimation template
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CONFIGURATION and INITIALISATION
dt=0.01;
COM='COM24';
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
qa = 0.001;    %Bruit d'état
ra = mean(ACCVAR); %Bruit de mesure accéléro
rmag = mean(MAGVAR);
rgyr = mean(MAGVAR);
Q = diag([qa qa qa qa]);
R = diag([ra' rmag']);
X=[1 0 0 0]'; %Etat : quaternion + vitesses de rot
P = 0.01*eye(4);

Error = [];
Covariance_trace = [sum(diag(P))];

tp=0;
ii=0;
obs = [];
xtrue = [];
xhat = [];
if useSimulation
    imu411('open',COM, 'simimu_3Dmaneuver');
else
    imu411('open',COM);
end
while(1)
    ii=ii+1;
    % read sensor
    [d, xt] = imu411('read'); %cette lecture est bloquante et cadence la boucle a 0.01s
    obs(:, end+1) = d;
    xtrue(:, end+1) = xt;    %Rappel
    % d(1)    = Time                (s)
    % d(2:4)  = Gyroscope     X,Y,Z (°/s)
    % d(5:7)  = Accelerometer X,Y,Z (g)
    % d(8:10) = Magnetometer  X,Y,Z (Gauss)
    t=d(1);
    % Predict
    q0=X(1);
    q1=X(2);
    q2=X(3);
    q3=X(4);
    F = eye(4);
    X = F*X;
    P = F*P*F'+Q;
    % Update
    if ~isnan(t)
        Y =[d(5)    %accX
            d(6)    %accY
            d(7)    %accZ
            d(8)    % Magx
            d(9)    % Magy
            d(10)]; % Magz]; % Gyrz
                   
        M=quat2M(X(1:4))';
        Yhat = [M*ACC0
            M*MAG0]; % Gyro give measurements in the inertial frame!
        J1=2*[q0 q1 -q2 -q3
            -q3 q2 q1 -q0
            q2 q3 q0 q1];
        J2=2*[q3 q2 q1 q0
            q0 -q1 q2 -q3
            -q1 -q0 q3 q2];
        J3=[-q2 q3 -q0 q1
            q1 q0 q3 q2
            q0 -q1 -q2 q3];

        %Hacc=J1*ACC0(1)+J2*ACC0(2)+J3*ACC0(3);
        Hacc=J1*ACC0(1)+J2*ACC0(2)+J3*ACC0(3);
        Hmag = J1*MAG0(1)+J2*MAG0(2)+J3*MAG0(3);

        H=[Hacc
           Hmag];
        G = H*P*H'+R;
        K = P*H'*inv(G);
        X = X+K*(Y-Yhat);
        X(1:4) = X(1:4)/norm(X(1:4));
        P = P - P*K*H;
        xhat(:, end+1) = X;
    end

    Error = [Error norm(K*(Y-Yhat))];
    Covariance_trace = [Covariance_trace sum(diag(P))];
    
    % Update 3D visualization:
    %DCM_k = quat2dcm(X(1:4)');
    DCM_k = quat2dcm(X(1:4)'/norm(X(1:4)))';
    update3D;
end

