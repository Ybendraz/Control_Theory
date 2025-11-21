% Initialize constants for the RCAM simulation:
clear
clc
close all

%% Adding paths:
addpath("01_Models\")
addpath("02_Scripts\")
addpath("03_Docs\")
addpath("04_Data\csv\template\")

%% Define Constants

% Xcg = 0.23*cbar;        %x position of CoG in Fm (m)
% Ycg = 0;                %y position of CoG in Fm (m)
% Zcg = 0.10*cbar;        %z position of CoG in Fm (m)
% m = 120000;
% 
% var_params = [m,Xcg,Ycg,Zcg];
g = 9.81;

% Initial state value
X0 = [100;       % approx 165 knots
    0;
    0; 
    0;
    0;
    0;
    0;
    0.1;        % approx 5.73 deg
    0
    ];

lat0 = deg2rad(21.315603);
lon0 = deg2rad(-157.858093);
h0 = 500; %[m]
Xgeodetic0 = [lon0;lat0;h0];
XNED0 = [0;0;-h0]
X0 = [X0;XNED0];


% Initial command value
U0 = [0;
        -0.1;      % approx -5.73 deg
        0;
        0.08;      % recall minimum for throttles are 0.5*pi/180 = 0.0087
        0.08];

TF = 200;

%% trim the model:

