close all
clear all
clc

filePath_good = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 3\Data\good_est_new.csv';
lines_good = strsplit(fileread(filePath_good), '\n'); 
headers_good = strsplit(lines_good{2}, ', '); 
rawData_good = dlmread(filePath_good, ',', 2, 0); 
sensor_good= rawData_good(10:650, 9);
xhat_good= rawData_good(10:650, 10);
step_good = rawData_good(10:650, 13);

filePath_bad = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 3\Data\bad_est_new.csv';
lines_bad = strsplit(fileread(filePath_bad), '\n'); 
headers_bad = strsplit(lines_bad{2}, ', '); 
rawData_bad = dlmread(filePath_bad, ',', 2, 0); 
sensor_bad = rawData_bad(10:650, 9);
xhat_bad = rawData_bad(10:650, 10);
step_bad = rawData_bad(10:650, 13);



figure
hold on
plot(sensor_good,'Color','k')
plot(xhat_good,'LineStyle','-.')
plot(step_good,'LineStyle','--')
legend('Measured distance','Estimated distance','Step reference')
xlabel('Time [ms]')
ylabel('Distance sensor value')
title('Good starting estimate')


figure
hold on
plot(sensor_bad,'Color','k')
plot(xhat_bad,'LineStyle','-.')
plot(step_bad,'LineStyle','--')
legend('Measured distance','Estimated distance','Step reference')
xlabel('Time [ms]')
ylabel('Distance')
title('Wrong starting estimate')
