%% Pitch control of a quadrotor:

m = 2;
ut0 = 1;
tau_theta = 0.1;
s = tf("s");
G = -1/s^2/(1+tau_theta*s)/m;
bode(G);
grid on;
sisotool(G)

%% Control using linear output feedback:

A = [0 1 0; 0 0 -ut0/m; 0 0 -1/tau_theta];
B = [0;0;1/tau_theta];
C = [1 0 0; 0 1 0];
D = 0;

sys = ss(A,B,C,D);
damp(sys)
poles = [roots([0.5^2 sqrt(2)/2*0.5 1]); -10];
K = place(A,B,poles);
norm(K)
damp(A-B*K)

