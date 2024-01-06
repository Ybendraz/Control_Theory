
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
qa = 0.0005;    %Bruit d'état
ra = 100*ACCVAR; %Bruit de mesure accéléro
rmag = 100*MAGVAR;
rgyr = 100*MAGVAR;
Q = diag([qa qa qa qa qa qa qa]);
R = diag([ra' rmag' rgyr']);
X=[1 0 0 0 0 0 0 ]'; %Etat : quaternion + vitesses de rot
P = eye(7);

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
    X = X;
    q0=X(1);
    q1=X(2);
    q2=X(3);
    q3=X(4);
    p = X(5);
    q = X(6);
    r = X(7);
    OmegaCrossQ = 1/2*[0 -p -q -r
                       p 0 r -q
                       q -r 0 p
                       r q -p 0] ;
    F = eye(7)+dt*[OmegaCrossQ zeros(4,3)
                   zeros(3,4) eye(3)];
    P = F*P*F'+Q;
    % Update
    if ~isnan(t)
        Y =[d(5)    %accX
            d(6)    %accY
            d(7)    %accZ
            d(8)    % Magx
            d(9)    % Magy
            d(10)   % Magz
            d(2)   % Gyrx
            d(3)   % Gyry
            d(4)]; % Gyrz
        M=quat2M(X(1:4))';
        Yhat = [M*ACC0
            M*MAG0
            GYR0]; % Gyro give measurements in the inertial frame!
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
        Hacc=J1*Y(1)+J2*Y(2)+J3*Y(3);
        Hacc_new = [Hacc zeros(3,3)];

        %Hmag = J1*ACC0(1)+J2*ACC0(2)+J3*ACC0(3);
        Hmag=J1*Y(4)+J2*Y(5)+J3*Y(6);
        Hmag_new = [Hmag zeros(3,3)];

        Hgyr=J1*Y(7)+J2*Y(8)+J3*Y(9);
        Hgyr_new = [zeros(3,4) eye(3)];    
        H=[Hacc_new
           Hmag_new
           Hgyr_new];
        G = H*P*H'+R;
        K = P*H'*inv(G);
        X = X+K*(Y-Yhat);
        X(1:4) = X(1:4)/norm(X(1:4));
        P = P - P*K*H;
        xhat(:, end+1) = X;
    end
    
    % Update 3D visualization:
    %DCM_k = quat2dcm(X(1:4)');
    DCM_k = quat2dcm(X(1:4)'/norm(X(1:4)))';
    update3D;
end

