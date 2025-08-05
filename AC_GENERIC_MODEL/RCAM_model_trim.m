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
X0 = [70;       % approx 165 knots
    0;
    0; 
    0;
    0;
    0;
    0;
    0.1;        % approx 5.73 deg
    0
    ];

% Initial command value
U0 = [0;
        -0.1;      % approx -5.73 deg
        0;
        0.1;      % recall minimum for throttles are 0.5*pi/180 = 0.0087
        0.1];

TF = 500;

%% Operating point & linearizations:
id_long = [1 3 5 8];
id_lat = [2 4 6 7];

id_output_long = 1:4;
id_output_lat = 6:9 ;

id_act_lat = [1 3];
id_act_long = [2 4 5];

RCAM_LIN_OP = linmod("RCAM_Model",X0,U0);
RCAM_LIN_SYS = ss(RCAM_LIN_OP.a,RCAM_LIN_OP.b,RCAM_LIN_OP.c,RCAM_LIN_OP.d);
RCAM_LIN_SYS.InputName = {'dA','dE','dR','dTh1','dTh2'};
RCAM_LIN_SYS.OutputName = {'Va','Vz','nz','q','gamma','beta','p','r','phi'};

RCAM_LIN_LONG = modred(RCAM_LIN_SYS(id_output_long,id_act_long),[id_lat 9],'truncate');
RCAM_LIN_LAT = modred(RCAM_LIN_SYS(id_output_lat,id_act_lat),[id_long 9],'truncate');



