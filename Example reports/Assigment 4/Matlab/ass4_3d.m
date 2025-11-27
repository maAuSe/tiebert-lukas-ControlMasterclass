clear all
close all
clc

R = [10^(-6),0;
    0, 10^(-6)];


ke = KalmanExperiment.createfromQRC45();

figure
hold on
plotmeasurements(ke,1);
plotstates(ke,1);
xlabel('Time')
ylabel('Position x')
title('95% certainty interval state 1')

figure
hold on
plotmeasurements(ke,2);
plotstates(ke,2);
xlabel('Time')
ylabel('Pendulum angle \theta')
title('95% certainty interval state 2')
