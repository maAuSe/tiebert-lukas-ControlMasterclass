clear
close all
clc

filePath = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 1\Data\data_ground_excitation.csv';
lines = strsplit(fileread(filePath), '\n'); 
headers = strsplit(lines{2}, ', ');
rawData = dlmread(filePath, ',', 2, 0); 
Ts = 0.01;
fs = 1 / Ts;
timeAxis = 0:Ts:15.99;
freqAxis = (0:1599)' * (fs / 1600);
processedData = rawData(2:3201, [3, 4]); 
totalDataPoints = length(processedData(:, 1));
inputSignal = rawData(2:3201, 5);
numCycles = 2;
pointsPerCycle = totalDataPoints / numCycles;
speedA_matrix = reshape(processedData(:, 1), pointsPerCycle, numCycles);
speedB_matrix = reshape(processedData(:, 2), pointsPerCycle, numCycles);
reshapedInputSignal = reshape(inputSignal, pointsPerCycle, numCycles);
avgSpeedA = mean(speedA_matrix, 2);
avgSpeedB = mean(speedB_matrix, 2);
avgInputSignal = mean(reshapedInputSignal, 2);

responseA = avgSpeedA(4:end);
regressorA = [-avgSpeedA(3:end-1), -avgSpeedA(2:end-2), avgInputSignal(2:end-2), avgInputSignal(1:end-3)];
thetaA = regressorA \ responseA;
numA = [0, 0, thetaA(3), thetaA(4)];
denomA = [1, thetaA(1), thetaA(2), 0];
wheelA_tf = tf(numA, denomA, Ts)

responseB = avgSpeedB(4:end);
regressorB = [-avgSpeedB(3:end-1), -avgSpeedB(2:end-2), avgInputSignal(2:end-2), avgInputSignal(1:end-3)];
thetaB = regressorB \ responseB;
numB = [0, 0, thetaB(3), thetaB(4)];
denomB = [1, thetaB(1), thetaB(2), 0];
wheelB_tf = tf(numB, denomB, Ts)


