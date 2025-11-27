close all
clear all
clc

filePath_10 = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 3\Data\K10.csv';
lines_10 = strsplit(fileread(filePath_10), '\n'); 
headers_10 = strsplit(lines_10{2}, ', '); 
rawData_10 = dlmread(filePath_10, ',', 2, 0); 
sensor_10= rawData_10(5:1180, 9);
step_10= rawData_10(5:1180, 13);
voltA_10 = rawData_10(5:1180, 2);
voltB_10 = rawData_10(5:1180, 3);


filePath_50 = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 3\Data\K50.csv';
lines_50 = strsplit(fileread(filePath_50), '\n'); 
headers_50 = strsplit(lines_50{2}, ', '); 
rawData_50 = dlmread(filePath_50, ',', 2, 0); 
sensor_50= rawData_50(5:1180, 9);
step_50= rawData_50(5:1180, 13);
voltA_50 = rawData_50(5:1180, 2);
voltB_50 = rawData_50(5:1180, 3);

filePath_100 = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 3\Data\K100.csv';
lines_100 = strsplit(fileread(filePath_100), '\n'); 
headers_100 = strsplit(lines_100{2}, ', '); 
rawData_100 = dlmread(filePath_100, ',', 2, 0); 
sensor_100= rawData_100(5:1180, 9);
step_100= rawData_100(5:1180, 13);
voltA_100 = rawData_100(5:1180, 2);
voltB_100 = rawData_100(5:1180, 3);

figure
hold on
plot(step_10,'Color','k')
plot(sensor_10,'LineStyle','-')
plot(sensor_50,'LineStyle','--')
plot(sensor_100,'LineStyle',':')
legend('Step input','K = 10','K = 50','K = 100')
xlabel('Time [ms]')
ylabel('Distance sensor value')


figure
hold on
plot(voltA_10,'Color','k')
plot(voltA_50,'LineStyle','-')
plot(voltA_100,'LineStyle','--')
legend('K = 10','K = 50','K = 100')
xlabel('Time [ms]')
ylabel('Control signal [V]')
title('Wheel A')

figure
hold on
plot(voltB_10,'Color','k')
plot(voltB_50,'LineStyle','-')
plot(voltB_100,'LineStyle','--')
legend('K = 10','K = 50','K = 100')
xlabel('Time [ms]')
ylabel('Control signal [V]')
title('Wheel B')

