%%
% CALC                                     Civil Aircraft Landing Challenge
% -------------------------------------------------------------------------
%
% This folder contains a  set of routines dedicated to a generic civil air-
% craft approach & landing problem. The objective  is to improve the propo-
% sed  baseline controller  so that a safe  landing can be performed in all 
% average and limit risks conditions with maximized crosswind.
%
% Simulink Files:
%
%   ACS        - Open-Loop Nonlinear Aircraft Model 
%   ALS        - Closed-Loop with Baseline Controller 
%
% Script File:
%
%   ALSeval    - AutoLand System Evaluation
%
%
% Functions & sub-routines:
%
%   initACSP   - Initialization of the global variable ACSP
%   ACStrim    - Trim & Linearization of ACS
%   initALS    - Initialization of the AutoLand Controller
%   MCsim      - Monte-Carlo Simulations
%   MCpar      - Generation of Monte Carlo Distributed Parameters
%   MCplots    - Monte-Carlo Analysis & Plots
%
%
% -------------------------------------------------------------------------
% SMAC Toolbox - ONERA The French Aerospace Lab, Toulouse, France
% Multi-Objectives Landing Aircraft Benchmark.
% http://w3.onera.fr/smac
% -------------------------------------------------------------------------
