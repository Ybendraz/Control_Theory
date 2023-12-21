clc
close all
%run trimRCAM.m

%Straight and level condition
Xdoto = zeros(9,1);
Xo = XStar;
Uo = UStar;

% Define the perturbation matrices (ie how much we perturb each function in
% each direction)

dxdot_matrix = 1e-11*ones(9,9);
dx_matrix = 1e-11*ones(9,9);
du_matrix = 1e-11*ones(9,5);

[E,Ap,Bp] = implicitLinmod(@RCAM_model_implicit, Xdoto, Xo, Uo, dxdot_matrix,dx_matrix,du_matrix);

%Calculate the A and B matrices:

A = -inv(E)*Ap;
B = -inv(E)*Bp;

% Tinv = [1 zeros(1,8);
%         0 0 1 zeros(1,6);
%         zeros(1,4) 1 zeros(1,4);
%         zeros(1,7) 1 0;
%         0 1 zeros(1,7);
%         0 0 0 1 zeros(1,5);
%         zeros(1,5) 1 0 0 0;
%         zeros(1,6) 1 0 0;
%         zeros(1,8) 1;
%     ];
% 
% A_round = Tinv*A*inv(Tinv);
% B_round = Tinv*B;

save linear_evolution_model A B;