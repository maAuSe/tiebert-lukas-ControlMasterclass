clear
close all
clc


filePath = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 1\Data\data_ground_excitation.csv';
lines = strsplit(fileread(filePath), '\n'); % Read and split by lines
headers = strsplit(lines{2}, ', '); % Extract header labels
rawData = dlmread(filePath, ',', 2, 0); % Load numeric data starting from the third line

 
Ts = 0.01;
fs = 1 / Ts;
freqAxis = (0:1599)' * (fs / 1600);
timeAxis = 0:Ts:15.99;


processedData = rawData(2:3201, [3, 4]); % Relevant columns for processing
inputSignal = rawData(2:3201, 5);
totalDataPoints = length(processedData(:, 1));
numCycles = 2;
pointsPerCycle = totalDataPoints / numCycles;
speedA_matrix = reshape(processedData(:, 1), pointsPerCycle, numCycles);
speedB_matrix = reshape(processedData(:, 2), pointsPerCycle, numCycles);
reshapedInputSignal = reshape(inputSignal, pointsPerCycle, numCycles);
v_avg_A = mean(speedA_matrix, 2);
v_avg_B = mean(speedB_matrix, 2);
inputsig_avg = mean(reshapedInputSignal, 2);
TF_A_ZOH = tf([0.6594 -0.3973], [1 -0.8789 0.006421 0], Ts);
TF_B_ZOH = tf([0.6845 -0.4205], [1 -0.8804 0.00909 0], Ts);
figure(1)
hold on
lsim(TF_A_ZOH, inputsig_avg, timeAxis, 'r')
plot(timeAxis, v_avg_A,'LineStyle','--');
title('Discretization Comparison: Wheel A')
legend({'Zero-Order Hold Model', 'Measured Data'}, 'Location', 'best')
xlabel('Time [s]')
ylabel('Speed [rad/s]')
grid on

figure(2)
hold on
lsim(TF_B_ZOH, inputsig_avg, timeAxis, 'r')
plot(timeAxis, v_avg_B,'LineStyle','--');
title('Discretization Comparison: Wheel B')
legend({'Zero-Order Hold Model', 'Measured Data'}, 'Location', 'best')
xlabel('Time [s]')
ylabel('Speed [rad/s]')
grid on
mod_resp_A = lsim(TF_A_ZOH, inputsig_avg, timeAxis);   
error_A = mod_resp_A - v_avg_A;
mod_resp_B = lsim(TF_B_ZOH, inputsig_avg, timeAxis);   
error_B = mod_resp_B - v_avg_B;
figure(3)
hold on
plot(timeAxis, error_A, 'r', 'LineWidth', 0.8) 
plot(timeAxis, error_B, 'b', 'LineWidth', 0.8,'LineStyle','--')
title('Model Error Comparison')
xlabel('Time [s]')
ylabel('Error [rad/s]')
legend({'Error: Wheel A', 'Error: Wheel B'}, 'Location', 'best')
grid on

