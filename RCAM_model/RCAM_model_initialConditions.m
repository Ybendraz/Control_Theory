% Initialize constants for the RCAM simulation:
clear
clc
close all
%% Define Constants
g = 9.81;
X0 = [100;       % approx 165 knots
    0;
    0; 
    0;
    0;
    0;
    0;
    0.1;        % approx 5.73 deg
    0];
XYZ0 = [0;0;0]; % Initial NED postion of the COG
U0 = [0;
        -0.1;      % approx -5.73 deg
        0;
        0.08;      % recall minimum for throttles are 0.5*pi/180 = 0.0087
        0.08];

TF = 60;

%% Run the model:
sim('RCAM_SimulinkModel_MeasurementsIncluded.slx')

%% Plot the results:

t = ans.tout;

u1 = ans.simU.Data(:,1);
u2 = ans.simU.Data(:,2);
u3 = ans.simU.Data(:,3);
u4 = ans.simU.Data(:,4);
u5 = ans.simU.Data(:,5);

x1 = ans.simX.Data(:,1);
x2 = ans.simX.Data(:,2);
x3 = ans.simX.Data(:,3);
x4 = ans.simX.Data(:,4);
x5 = ans.simX.Data(:,5);
x6 = ans.simX.Data(:,6);
x7 = ans.simX.Data(:,7);
x8 = ans.simX.Data(:,8);
x9 = ans.simX.Data(:,9);

figure("Name","States - Speed of the airplane")

subplot(5,2,1)
plot(t,x1)
grid on
xlabel("Time(s)")
legend("x1")

subplot(5,2,2)
plot(t,x2)
grid on
xlabel("Time(s)")
legend("x2")


subplot(5,2,3)
plot(t,x3)
grid on
xlabel("Time(s)")
legend("x3")


subplot(5,2,4)
plot(t,x4)
grid on
xlabel("Time(s)")
legend("x4")


subplot(5,2,5)
plot(t,x5)
grid on
xlabel("Time(s)")
legend("x5")

subplot(5,2,6)
plot(t,x6)
grid on
xlabel("Time(s)")
legend("x6")

subplot(5,2,7)
plot(t,x7)
grid on
xlabel("Time(s)")
legend("x7")

subplot(5,2,8)
plot(t,x8)
grid on
xlabel("Time(s)")
legend("x8")

subplot(5,2,[9 10])
plot(t,x9)
grid on
xlabel("Time(s)")
legend("x9")


figure("Name","Commands")
grid on

subplot(3,2,1)
plot(t,u1*ones(size(t)))
grid on
xlabel("Time(s)")
legend("u1")

subplot(3,2,2)
plot(t,u2*ones(size(t)))
grid on
xlabel("Time(s)")
legend("u2")

subplot(3,2,3)
plot(t,u3*ones(size(t)))
grid on
xlabel("Time(s)")
legend("u3")

subplot(3,2,4)
plot(t,u4*ones(size(t)))
grid on
xlabel("Time(s)")
legend("u4")

subplot(3,2,[5 6])
plot(t,u5*ones(size(t)))
grid on
xlabel("Time(s)")
legend("u5")