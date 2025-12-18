clear
close all
clc

filePath = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 1\Data\data_ground_excitation.csv';
lines = strsplit(fileread(filePath), '\n'); 
headers = strsplit(lines{2}, ', ');
rawData = dlmread(filePath, ',', 2, 0); 

Ts = 0.01;
fs = 1 / Ts;
freqAxis = (0:1599)' * (fs / 1600);
timeAxis = 0:Ts:15.99;
processedData = rawData(2:3201, [3, 4]);
inputSignal = rawData(2:3201, 5);
totalDataPoints = length(processedData(:, 1));
numCycles = 2;
pointsPerCycle = totalDataPoints / numCycles;
vA_matr = reshape(processedData(:, 1), pointsPerCycle, numCycles);
vB_matr = reshape(processedData(:, 2), pointsPerCycle, numCycles);
reshapedInputSignal = reshape(inputSignal, pointsPerCycle, numCycles);

v_avg_A = mean(vA_matr, 2);
v_avg_B = mean(vB_matr, 2);
avgInputSignal = mean(reshapedInputSignal, 2);
TF_A_ZOH = tf([0.6594 -0.3973], [1 -0.8789 0.006421 0], Ts);
TF_B_ZOH = tf([0.6845 -0.4205], [1 -0.8804 0.00909 0], Ts);
denomA = TF_A_ZOH.Denominator{1}; 
numA = TF_A_ZOH.Numerator{1};
[respA, freqA] = freqz(denomA, numA, 2001, fs);
denomB = TF_B_ZOH.Denominator{1}; 
numB = TF_B_ZOH.Numerator{1}; 
[respB, freqB] = freqz(denomB, numB, 2001, fs); 
figure
hold on
plot(freqA, abs(respA), 'LineWidth', 1.2)
plot(freqB, abs(respB), 'LineWidth', 1.2,'LineStyle','--')
title('Magnitude of Weighting Function vs Frequency (ZOH)')
xlabel('Frequency [Hz]')
ylabel('|A(q,\theta)|')
grid on

freq_int = 30;
cut_off_freq = 1.5 * freq_int;
[Before_filt, After_filt] = butter(6, cut_off_freq / (fs / 2));
avgSpeedA_filt = filter(Before_filt, After_filt, v_avg_A);
avgSpeedB_filt = filter(Before_filt, After_filt, v_avg_B);
respA = avgSpeedA_filt(4:end);
regrA = [-avgSpeedA_filt(3:end-1), -avgSpeedA_filt(2:end-2), avgInputSignal(2:end-2), avgInputSignal(1:end-3)];
thetaA = regrA \ respA;
denomA = [1, thetaA(1), thetaA(2), 0];
numA = [0, 0, thetaA(3), thetaA(4)];
wheelA_tf_filt = tf(numA, denomA, Ts)

respB = avgSpeedB_filt(4:end);
regrB = [-avgSpeedB_filt(3:end-1), -avgSpeedB_filt(2:end-2), avgInputSignal(2:end-2), avgInputSignal(1:end-3)];
thetaB = regrB \ respB;
denomB = [1, thetaB(1), thetaB(2), 0];
numB = [0, 0, thetaB(3), thetaB(4)];
wheelB_tf_filt = tf(numB, denomB, Ts)
figure
hold on
lsim(wheelA_tf_filt, avgInputSignal, timeAxis, 'r')
plot(timeAxis, avgSpeedA_filt,'LineStyle','--');
title('Discretization Comparison Filtered: Wheel A')
legend({'Zero-Order Hold Model', 'Measured Data'}, 'Location', 'best')
xlabel('Time [s]')
ylabel('Speed [rad/s]')
grid on

figure
hold on
lsim(wheelB_tf_filt, avgInputSignal, timeAxis, 'r')
plot(timeAxis, avgSpeedB_filt,'LineStyle','--')
title('Discretization Comparison Filtered: Wheel B')
legend({'Zero-Order Hold Model', 'Measured Data'}, 'Location', 'best')
xlabel('Time [s]')
ylabel('Speed [rad/s]')
grid on

mod_respB = lsim(wheelB_tf_filt, avgInputSignal, timeAxis);
errorB = mod_respB - avgSpeedB_filt;
mod_respA = lsim(wheelA_tf_filt, avgInputSignal, timeAxis);   
errorA = mod_respA - avgSpeedA_filt;
figure
hold on
beginzoom = find(timeAxis >= 3.5, 1); 
endzoom = find(timeAxis <= 8.5, 1, 'last'); 
plot(timeAxis(beginzoom:endzoom), errorA(beginzoom:endzoom), 'r', 'LineWidth', 0.8)
plot(timeAxis(beginzoom:endzoom), errorB(beginzoom:endzoom), 'b', 'LineWidth', 0.8,'LineStyle','--') 
title('Model Error Comparison (Zoomed In) - Filtered')
xlabel('Time [s]')
ylabel('Error [rad/s]')
ylim([-6 8])
legend({'Error: Wheel A', 'Error: Wheel B'}, 'Location', 'best')
grid on
