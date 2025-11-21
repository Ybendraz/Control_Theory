%% Paths to add
addpath("..\")
addpath("..\01_Models\")
addpath("..\02_Scripts\")
addpath("..\03_Docs\")
addpath("..\s04_Data\csv\template\")

%% Run trim scripts:
run RCAM_model_trim.m
clear RCAM_LIN_OP id_act_lat id_output_lat id_lat RCAM_LIN_SYS RCAM_LIN_LAT

%% Analysis of longitudinal open_loop:
Y0_long = RCAM_LIN_LONG.C*X0(id_long)+RCAM_LIN_LONG.D*U0(id_act_long);
[V_long,D_long,W_long] = eig(RCAM_LIN_LONG.A);
w_1 = abs(D_long(1,1));
w_2 = abs(D_long(3,3));
xi_1 = abs(real(D_long(1,1))/w_1);
xi_2 = abs(real(D_long(3,3))/w_2);

display("right eigen vectors:")
V_long

display("eigen values:")
diag(D_long)

display("mode N°1 frequency:")
w_1
display("mode N°1 damping:")
xi_1

display("mode N°2 frequency:")
w_2
display("mode N°2 damping:")
xi_2

figure("Name","Poles/Zeros mapping - open loop")
pzmap(RCAM_LIN_LONG)
%% We can almost immediately notice that the mode number 1 is the short mode
% characterized with a 1.56 rad/s and 0.48 damping coefficient.
% As for the phugoid mode, its characteristic frequency is equal to 0.5
% rad/s with a very low damping coefficient equal to 0.0933.

%----------------------------------------------------------------
% Analysis of the modal composition of the matrices A,B,C and D
% Given that : X = V*Zeta, Zeta_dot = W*D*V*Zeta + W*B*U,
% Y = C*V*Zeta + D*U
%----------------------------------------------------------------
V_abs = abs(V_long);
State_mod_map = zeros(size(RCAM_LIN_LONG.A,1),2);
for i=1:size(RCAM_LIN_LONG.A,1)
    modal_weight = V_abs(i,:);
    [~,id] = sort(modal_weight,2,"descend");
    id(id==2) = 1;
    id(id==3) = 2;
    id(id==4) = 2;
    State_mod_map(i,:) = id([1,4]);
end

display("Mapping of states/modes:")
State_mod_map

%----------------------------------------------------------------
% Note:
% u -> Phugoïd (1) + Short-mode (2)
% v -> Short-mode (1) + Phugoïd (2)
% q -> Short-mode (1) + Phugoïd (2)
% theta -> Short-mode (1) + Phugoïd (2)
%----------------------------------------------------------------

%% Frequency response of the open loop:
RCAM_LIN_LONG_TF = RCAM_LIN_LONG(:,[1 2]);
for i=1:4
    figure()
    bode(RCAM_LIN_LONG_TF(i,:))
    grid on
end

%% Time response of the open-loop:

for i=1:4
    figure("Name","State_mod_map")
    step(deg2rad(1)*RCAM_LIN_LONG_TF(i,:),100);
    grid on
end

%% Longitudinal autopilot design:
%----------------------------------------------------------------
% Requirements and constraints:
% The longitudinal autopilot controller structure is a feedback type
% structure : u = K*Y+H*e
% -------------
% Requirements:
% -------------
% 1) - We would like to control the calibrated speed V and the vertical speed
% Vz while establishing a strong independence between the two, meaning that
% a V_c entry command shall not create a significative Vz transient
% response and a zero steady-state error and vice-versa.
%
% 2) - The phugoid mode shall be damped up to 0.707 if possible and
% fastened by a factor of 1.2.
% 3) - The time response of the V control loop shall be 15 to 20 seconds
% while the Vz loop response time shall be 10 seconds.
%
% 4) - Robustness margins:
%    All the broken loops shall exhibit a 6db gain margin / >+45deg phase
%    margin.
%----------------------------------------------------------------

xi = sqrt(2)/2;
w_short_mode = 1.2*w_1;
tr_V = 20;
tr_Vz = 15;
w_V_loop = 4/(xi*tr_V);
w_Vz_loop = 4/(xi*tr_Vz);

cl_poles = [roots([1 2*xi*w_short_mode w_short_mode^2]) roots([1 2*xi*w_V_loop w_V_loop^2]) roots([1 2*xi*w_Vz_loop w_Vz_loop^2])];

% Open loop model with integral outputs:
ol_integral = linmod('RCAM_OP_LONG_AP_MODAL_INTEGRAL');
ol_integral = ss(ol_integral.a,ol_integral.b,ol_integral.c,ol_integral.d);

% Pole placement:
n = size(ol_integral.a,1);
m = size(ol_integral.b,2);
VW = zeros(n+m,n);

for i = 1:n
    if (i == 3 || i == 4)
        eig_v = null([ol_integral.a - cl_poles(i)*eye(n) ol_integral.b;ol_integral.c(2,:) zeros(1,m)]);
        VW(:,i) = mean(eig_v,2);
    else
        eig_v = null([ol_integral.a - cl_poles(i)*eye(n) ol_integral.b;ol_integral.c(1,:) zeros(1,m)]);
        VW(:,i) = mean(eig_v,2);
    end
end


K_long1_y= real(-VW(n+1:n+m,:)*inv(ol_integral.c*VW(1:n,:)+ol_integral.d*VW(n+1:n+m,:)));

RCAM_Model_AP_CL = feedback(ol_integral,K_long1_y);
damp(RCAM_Model_AP_CL)

%% Feedforward gain computation:
H_long_1 = eye(2);
cl_integral = linmod('RCAM_OP_LONG_AP_MODAL_CL');
cl_integral = ss(cl_integral.a,cl_integral.b,cl_integral.c,cl_integral.d);
damp(cl_integral);
H_long_1 = inv(dcgain(cl_integral));
%%
[A,B,C,D]=linmod('RCAM_Model_AP_LONG_NL');  % linéarisation autour de x0
damp(A)
