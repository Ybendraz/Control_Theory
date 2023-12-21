clear
clc
close all


%Load the trim point
temp = load('trim_values_straight_level');
XStar = temp.XStar;
UStar = temp.UStar;

%run model
sim('RCAMAircraftSim.slx')

%Extract the data

t = simX.Time;
X = simX.Data;

figure;

for k=1:9
    subplot(5,2,k)
    plot(t,X(:,k),'LineWidth',2)
    ylabel(['x_', num2str(k)])
    grid on
end
















