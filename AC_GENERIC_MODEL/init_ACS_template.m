%% Initial scripts:
clc; close all; clear;

%% Adding paths:
addpath("01_Models\")
addpath("02_Scripts\")
addpath("03_Docs\")
addpath("04_Data\csv\template\")

%% Initialize the ACSP:
ACSP = importStructFromFolder('04_Data\csv\template\ACSP');

% Open the simulink model:
open("ACS_AP.slx")