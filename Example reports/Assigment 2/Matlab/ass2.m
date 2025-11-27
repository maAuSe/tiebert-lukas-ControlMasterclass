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
vA_matrix = reshape(processedData(:, 1), pointsPerCycle, numCycles);
vB_matrix = reshape(processedData(:, 2), pointsPerCycle, numCycles);
reshapedInputSignal = reshape(inputSignal, pointsPerCycle, numCycles);

avg_v_A = mean(vA_matrix, 2);
avg_v_B = mean(vB_matrix, 2);
avgInputSignal = mean(reshapedInputSignal, 2);


TF_A_ZOH = tf([0.6594 -0.3973], [1 -0.8789 0.006421 0], Ts);
TF_B_ZOH = tf([0.6845 -0.4205], [1 -0.8804 0.00909 0], Ts);

interesting_frequency = 30;
cutoff_freq = 1.5 * interesting_frequency;

[Before_filt, After_filt] = butter(6, cutoff_freq / (fs / 2));

avgSpeedA_filt = filter(Before_filt, After_filt, avg_v_A);
avgSpeedB_filt = filter(Before_filt, After_filt, avg_v_B);
respA = avgSpeedA_filt(4:end);
regrA = [-avgSpeedA_filt(3:end-1), -avgSpeedA_filt(2:end-2), avgInputSignal(2:end-2), avgInputSignal(1:end-3)];
thetaA = regrA \ respA;
numA = [0, 0, thetaA(3), thetaA(4)];
denomA = [1, thetaA(1), thetaA(2), 0];
wheelA_tf_filt = tf(numA, denomA, Ts)
respB = avgSpeedB_filt(4:end);
regrB = [-avgSpeedB_filt(3:end-1), -avgSpeedB_filt(2:end-2), avgInputSignal(2:end-2), avgInputSignal(1:end-3)];
thetaB = regrB \ respB;
numB = [0, 0, thetaB(3), thetaB(4)];
denomB = [1, thetaB(1), thetaB(2), 0];
wheelB_tf_filt = tf(numB, denomB, Ts)

figure
margin(wheelA_tf_filt)
figure
margin(wheelB_tf_filt)

f_cross=30.3;
T_i=tand(90-15)/f_cross;
K=0.8395;

controller_cont = tf([K, K/T_i], [1, 0]);
controller_disc = c2d(controller_cont,Ts,'tustin')
compensated_system = controller_disc*wheelA_tf_filt;
figure
margin(compensated_system)

closed_loop = compensated_system/(1+compensated_system);

csvfile = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 2\Data\data_controlled_pm100.csv';
labels = strsplit(fileread(csvfile), '\n'); % Split file in lines
labels = strsplit(labels{:, 2}, ', '); % Split and fetch the labels (they are in line 2 of every recodata = dlmread(csvfile, ',', 2, 0); % Data follows the labels
data_controlled_car = dlmread(csvfile, ',', 2, 0); % Data follows the labels
plot_data = data_controlled_car(10:510,:);
time_interval = 0:0.01:5;

ref = plot_data(:,4);
meas_speed = plot_data(:,9);
sim_resp = lsim(closed_loop,ref,time_interval);

figure
plot_sim = plot(time_interval,sim_resp,'r');
hold on
plot_ref = plot(time_interval,ref,'g','LineStyle','--');
plot_meas = plot(time_interval,meas_speed,'b','LineStyle',':');
legend([plot_sim(1),plot_ref(1),plot_meas(1)],{'Simulated closed-loop response','Step reference','Measured closed-loop response'})
title('Step reference plot wheel A')
xlabel('Time [s]')
ylabel('Rotational speed [rad/s]')
hold off


ref = plot_data(:,4);
meas_speed = plot_data(:,10);
sim_resp = lsim(closed_loop,ref,time_interval);

figure
plot_sim = plot(time_interval,sim_resp,'r');
hold on
plot_ref = plot(time_interval,ref,'g','LineStyle','--');
plot_meas = plot(time_interval,meas_speed,'b','LineStyle',':');
legend([plot_sim(1),plot_ref(1),plot_meas(1)],{'Simulated closed-loop response','Step reference','Measured closed-loop response'})
title('Step reference plot wheel B')
xlabel('Time [s]')
ylabel('Rotational speed [rad/s]')
hold off

tracking_error = plot_data(:,5);
tracking_error_tf = 1/(1+compensated_system);
tracking_error_sim_resp = lsim(tracking_error_tf,ref,time_interval);
figure
plot_error_sim = plot(time_interval,tracking_error_sim_resp,'r');
hold on
plot_error = plot(time_interval,tracking_error,'b','LineStyle','--');
legend([plot_error_sim(1),plot_error(1)],{'Simulated tracking error','Measured tracking error'})
title('Tracking error wheel A')
xlabel('Time [s]')
ylabel('Tracking error [rad/s]')
tracking_error = plot_data(:,6);
tracking_error_tf = 1/(1+compensated_system);
tracking_error_sim_resp = lsim(tracking_error_tf,ref,time_interval);

figure
plot_error_sim = plot(time_interval,tracking_error_sim_resp,'r');
hold on
plot_error = plot(time_interval,tracking_error,'b','LineStyle','--');
legend([plot_error_sim(1),plot_error(1)],{'Simulated tracking error','Measured tracking error'})
title('Tracking error wheel B')
xlabel('Time [s]')
ylabel('Tracking error [rad/s]')

control_volt = plot_data(:,7);
control_volt_tf = controller_disc/(1+compensated_system);
control_volt_sim = lsim(control_volt_tf,ref,time_interval);
figure
plot_control_sim = plot(time_interval,control_volt_sim,'r');
hold on
plot_control_meas = plot(time_interval,control_volt,'b','LineStyle','--');
legend([plot_control_sim(1),plot_control_meas(1)],{'Simulated control signal','Measured control signal'})
title('Control signal wheel A')
xlabel('Time [s]')
ylabel('Voltage [V]')

control_volt = plot_data(:,8);
control_volt_tf = controller_disc/(1+compensated_system);
control_volt_sim = lsim(control_volt_tf,ref,time_interval);

figure
plot_control_sim = plot(time_interval,control_volt_sim,'r');
hold on
plot_control_meas = plot(time_interval,control_volt,'b','LineStyle','--');
legend([plot_control_sim(1),plot_control_meas(1)],{'Simulated control signal','Measured control signal'})
title('Control signal wheel B')
xlabel('Time [s]')
ylabel('Voltage [V]')


csvfile = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 2\Data\data_controlled_incline.csv';
labels = strsplit(fileread(csvfile), '\n'); % Split file in lines
labels = strsplit(labels{:, 2}, ', '); % Split and fetch the labels (they are in line 2 of every recodata = dlmread(csvfile, ',', 2, 0); % Data follows the labels
data_controlled_car = dlmread(csvfile, ',', 2, 0); % Data follows the labels
plot_data = data_controlled_car(10:510,:);
time_interval = 0:0.01:5;


ref = plot_data(:,4);
meas_speed = plot_data(:,9);
sim_resp = lsim(closed_loop,ref,time_interval);

figure
plot_sim = plot(time_interval,sim_resp,'r');
hold on
plot_ref = plot(time_interval,ref,'g','LineStyle','--');
plot_meas = plot(time_interval,meas_speed,'b','LineStyle',':');
legend([plot_sim(1),plot_ref(1),plot_meas(1)],{'Simulated closed-loop response','Step reference','Measured closed-loop response'})
title('Step reference plot wheel A (Incline)')
xlabel('Time [s]')
ylabel('Rotational speed [rad/s]')
hold off


ref = plot_data(:,4);
meas_speed = plot_data(:,10);
sim_resp = lsim(closed_loop,ref,time_interval);
figure
plot_sim = plot(time_interval,sim_resp,'r');
hold on
plot_ref = plot(time_interval,ref,'g','LineStyle','--');
plot_meas = plot(time_interval,meas_speed,'b','LineStyle',':');
legend([plot_sim(1),plot_ref(1),plot_meas(1)],{'Simulated closed-loop response','Step reference','Measured closed-loop response'})
title('Step reference plot wheel B (Incline)')
xlabel('Time [s]')
ylabel('Rotational speed [rad/s]')
hold off






tracking_error = plot_data(:,5);
tracking_error_tf = 1/(1+compensated_system);
tracking_error_sim_resp = lsim(tracking_error_tf,ref,time_interval);

figure
plot_error_sim = plot(time_interval,tracking_error_sim_resp,'r');
hold on
plot_error = plot(time_interval,tracking_error,'b','LineStyle','--');
legend([plot_error_sim(1),plot_error(1)],{'Simulated tracking error','Measured tracking error'})
title('Tracking error wheel A (Incline)')
xlabel('Time [s]')
ylabel('Tracking error [rad/s]')
tracking_error = plot_data(:,6);
tracking_error_tf = 1/(1+compensated_system);
tracking_error_sim_resp = lsim(tracking_error_tf,ref,time_interval);
figure
plot_error_sim = plot(time_interval,tracking_error_sim_resp,'r');
hold on
plot_error = plot(time_interval,tracking_error,'b','LineStyle','--');
legend([plot_error_sim(1),plot_error(1)],{'Simulated tracking error','Measured tracking error'})
title('Tracking error wheel B (Incline)')
xlabel('Time [s]')
ylabel('Tracking error [rad/s]')


control_volt = plot_data(:,7);
control_volt_tf = controller_disc/(1+compensated_system);
control_volt_sim = lsim(control_volt_tf,ref,time_interval);
figure
plot_control_sim = plot(time_interval,control_volt_sim,'r');
hold on
plot_control_meas = plot(time_interval,control_volt,'b','LineStyle','--');
legend([plot_control_sim(1),plot_control_meas(1)],{'Simulated control signal','Measured control signal'})
title('Control signal wheel A (Incline)')
xlabel('Time [s]')
ylabel('Voltage [V]')
control_volt = plot_data(:,8);
control_volt_tf = controller_disc/(1+compensated_system);
control_volt_sim = lsim(control_volt_tf,ref,time_interval);
figure
plot_control_sim = plot(time_interval,control_volt_sim,'r');
hold on
plot_control_meas = plot(time_interval,control_volt,'b','LineStyle','--');
legend([plot_control_sim(1),plot_control_meas(1)],{'Simulated control signal','Measured control signal'})
title('Control signal wheel B (Incline)')
xlabel('Time [s]')
ylabel('Voltage [V]')

f_cross=3.1415; 
T_i=tand(90-15)/f_cross;
K=0.4741;
controller_cont_low_crossoverfreq = tf([K, K/T_i], [1, 0]);
controller_disc_low_crossoverfreq = c2d(controller_cont_low_crossoverfreq,Ts,'tustin')
compensated_system_low_crossoverfreq = controller_disc_low_crossoverfreq*wheelA_tf_filt
figure
margin(compensated_system_low_crossoverfreq)
closed_loop_low_crossoverfreq = compensated_system_low_crossoverfreq/(1+compensated_system_low_crossoverfreq);



