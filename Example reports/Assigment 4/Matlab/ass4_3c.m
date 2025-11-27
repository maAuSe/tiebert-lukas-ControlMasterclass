close all
clear all
clc

filePath_normal = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 4\Data\Normal_QR_new.csv';
rawData_normal = dlmread(filePath_normal, ',', 2, 0); 

filePath_bigR = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 4\Data\Big_R_new.csv';
rawData_bigR = dlmread(filePath_bigR, ',', 2, 0); 

filePath_bigQ = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 4\Data\Big_Q_new.csv';
rawData_bigQ = dlmread(filePath_bigQ, ',', 2, 0); 

filePath_smallQ = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 4\Data\Small_Q_new.csv';
rawData_smallQ = dlmread(filePath_smallQ, ',', 2, 0); 



xhat0_normal = rawData_normal(1:800, 3);
xhat0_bigR = rawData_bigR(1:800, 3);
xhat0_bigQ = rawData_bigQ(1:800, 3);
xhat0_smallQ = rawData_smallQ(1:800, 3);

xhat1_normal = rawData_normal(1:800, 4);
xhat1_bigR = rawData_bigR(1:800, 4);
xhat1_bigQ = rawData_bigQ(1:800, 4);
xhat1_smallQ = rawData_smallQ(1:800, 4);

xhat2_normal = rawData_normal(1:800, 5);
xhat2_bigR = rawData_bigR(1:800, 5);
xhat2_bigQ = rawData_bigQ(1:800, 5);
xhat2_smallQ = rawData_smallQ(1:800, 5);

meas0_normal = rawData_normal(1:800, 12);
meas0_bigR = rawData_bigR(1:800, 12);
meas0_bigQ = rawData_bigQ(1:800, 12);
meas0_smallQ = rawData_smallQ(1:800, 12);

meas1_normal = rawData_normal(1:800, 13);
meas1_bigR = rawData_bigR(1:800, 13);
meas1_bigQ = rawData_bigQ(1:800, 13);
meas1_smallQ = rawData_smallQ(1:800, 13);


%plots

figure
hold on
plot(meas0_normal,'Color','k')
plot(xhat0_normal,'LineStyle','-')
plot(xhat0_bigR,'LineStyle','-.')
plot(xhat0_bigQ,'LineStyle',':')
plot(xhat0_smallQ,'LineStyle','--')
legend('Measurement','Normal QR ratio','Big R','Big Q', 'Small Q')
xlabel('Time')
ylabel('Position')
title('Estimation of state 1')

figure
hold on
plot(meas1_normal,'Color','k')
plot(xhat1_normal,'LineStyle','-')
plot(xhat1_bigR,'LineStyle','-.')
plot(xhat1_bigQ,'LineStyle',':')
plot(xhat1_smallQ,'LineStyle','--')
legend('Measurement','Normal QR ratio','Big R','Big Q', 'Small Q')
xlabel('Time')
ylabel('Pendulum angle \theta')
title('Estimation of state 2')

figure
hold on
plot(xhat2_normal,'LineStyle','-')
plot(xhat2_bigR,'LineStyle','-.')
plot(xhat2_bigQ,'LineStyle',':')
plot(xhat2_smallQ,'LineStyle','--')
legend('Normal QR ratio','Big R','Big Q', 'Small Q')
xlabel('Time')
ylabel('Tangential speed')
title('Estimation of state 3')


