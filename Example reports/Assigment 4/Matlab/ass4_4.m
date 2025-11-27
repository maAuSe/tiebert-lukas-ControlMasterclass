clear all
close all
clc

A = [1 0 0
    0 0.9964 0.07398
    0 -0.09798 0.9964];
B = [0.01
    -0.07398
    0.003631];
R = 1;


Q = [0.1 0 0
    0 0.1 0
    0 0 0];
K_0_1 = dlqr(A,B,Q,R)

Q = [0.5 0 0
    0 0.5 0
    0 0 0];
K_0_5 = dlqr(A,B,Q,R)

Q = [1 0 0
    0 1 0
    0 0 0];
K_1 = dlqr(A,B,Q,R)

Q =[ 5 0 0
    0 5 0
    0 0 0];
K_5 = dlqr(A,B,Q,R)



filePath_0_1 = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 4\Data\vraag4_0.1.csv';
rawData_0_1 = dlmread(filePath_0_1, ',', 2, 0); 

filePath_0_5 = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 4\Data\vraag4_0.5.csv';
rawData_0_5 = dlmread(filePath_0_5, ',', 2, 0); 

filePath_1 = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 4\Data\vraag4_1.csv';
rawData_1 = dlmread(filePath_1, ',', 2, 0); 

filePath_5 = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 4\Data\vraag4_5.csv';
rawData_5 = dlmread(filePath_5, ',', 2, 0); 


measurement_0_1 = rawData_0_1(1:1600, 2);
measurement_0_5 = rawData_0_5(1:1600, 2);
measurement_1 = rawData_1(1:1600, 2);
measurement_5 = rawData_5(1:1600, 2);

actuator_0_1 = rawData_0_1(1:1600, 9);
actuator_0_5 = rawData_0_5(1:1600, 9);
actuator_1 = rawData_1(1:1600, 9);
actuator_5 = rawData_5(1:1600, 9);

figure
hold on
plot(measurement_0_1,'Color','k')
plot(measurement_0_5,'LineStyle','--')
plot(measurement_1,'LineStyle','-.')
plot(measurement_5,'LineStyle',':')
legend('\rho = 0.1','\rho = 0.5','\rho = 1','\rho = 5')
xlabel('Time')
ylabel('Pendulum mass position')
title('Response for multiple \rho')

figure
hold on
plot(actuator_0_1,'Color','k')
plot(actuator_0_5,'LineStyle','--')
plot(actuator_1,'LineStyle','-.')
plot(actuator_5,'LineStyle',':')
legend('\rho = 0.1','\rho = 0.5','\rho = 1','\rho = 5')
xlabel('Time')
ylabel('Actuator signal [m/s]')
title('Actuator signal for multiple \rho')





