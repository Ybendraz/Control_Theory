function X_dot = DroneDynamicModel(X,U)


%% Définition des constantes (Masse, inerties, distances ...)

% Inspiré d'un doc technique de Parrot
m = 1;
Ix = 1;
Iy = Ix;
Iz = 0.5;

X0 = zeros(12,1);

%% Définition des états et des commandes:

%%% Postion & Vitesses
Px = X(1);
Py = X(2);
Pz = X(3);

Vx = X(4);
Vy = X(5);
Vz = X(6);

%%% Attitudes et vitesses de rotation:

phi = X(7);                      % premier angle d'euler: Roulis
theta = X(8);                    % second angle d'euler: assiette latérale
psi = X(9);                      % troisième angel d'euler: Lacet.

p = X(10);                       % vitesse de roulis
q = X(11);                       % tangage
r = X(12);                       % vitesse du lacet

%%% Commandes
uT = U(1);
uPhi = U(2);
uTheta = U(3);
uPsi = U(4);
%% Mise à jour du modèle d'état: 

X_dot = [Vx
         Vy  
         Vz
         -uT/m*(cos(phi)*sin(theta)*cos(psi)+sin(psi)*sin(theta))
         uT/m*(cos(psi)*sin(phi)-sin(psi)*sin(theta)*cos(phi))
         -uT/m*(cos(phi)*cos(theta))
         p+q*sin(phi)*tan(theta)*r*cos(phi)*tan(theta)
         q*cos(phi)-r*sin(phi)
         q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta)
         (Iy*q*r-Iz*q*r)/Ix + uPhi/Ix
         (Iz*p*r-Ix*p*r)/Iy + uTheta/Iy
         (Ix*p*q-Iz*p*q)/Iz + uPsi/Iz
        ]


end