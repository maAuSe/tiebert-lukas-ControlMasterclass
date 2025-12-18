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
inp_avg_signal = mean(reshapedInputSignal, 2);
TF_B_ZOH = tf([0.6845 -0.4205], [1 -0.8804 0.00909 0], Ts);
TF_A_ZOH = tf([0.6594 -0.3973], [1 -0.8789 0.006421 0], Ts);
int_frequency = 30;
cutoff_freq = 1.5 * int_frequency;
[Before_filt, After_filt] = butter(6, cutoff_freq / (fs / 2));
v_avg_A_filt = filter(Before_filt, After_filt, v_avg_A);
v_avg_B_filt = filter(Before_filt, After_filt, v_avg_B);
respA = v_avg_A_filt(4:end);
regrA = [-v_avg_A_filt(3:end-1), -v_avg_A_filt(2:end-2), inp_avg_signal(2:end-2), inp_avg_signal(1:end-3)];
thetaA = regrA \ respA;
numA = [0, 0, thetaA(3), thetaA(4)];
denomA = [1, thetaA(1), thetaA(2), 0];
wheelA_tf_filt = tf(numA, denomA, Ts)
respB = v_avg_B_filt(4:end);
regrB = [-v_avg_B_filt(3:end-1), -v_avg_B_filt(2:end-2), inp_avg_signal(2:end-2), inp_avg_signal(1:end-3)];
thetaB = regrB \ respB;
numB = [0, 0, thetaB(3), thetaB(4)];
denomB = [1, thetaB(1), thetaB(2), 0];
wheelB_tf_filt = tf(numB, denomB, Ts)
csvfile = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 2\Data\data_controlled_incline.csv';
labels = strsplit(fileread(csvfile), '\n'); 
labels = strsplit(labels{:, 2}, ', '); 
data_control_car = dlmread(csvfile, ',', 2, 0); 
plot_data = data_control_car(10:510,:);
time_int = 0:0.01:5;
csvfile_LF = 'C:\Users\maxim\OneDrive - KU Leuven\our Assigment\Assigment 2\Data\data_controlled_incline_lowfreq.csv';
labels_LF = strsplit(fileread(csvfile_LF), '\n'); 
labels_LF = strsplit(labels_LF{:, 2}, ', '); 
data_controlled_car_LF = dlmread(csvfile_LF, ',', 2, 0); 
plot_data_LF = data_controlled_car_LF(10:510,:);
time_int = 0:0.01:5;


f_crossover=30.3;
T_i=tand(90-15)/f_crossover;
K=0.8395;
controller_cont = tf([K, K/T_i], [1, 0]);
controller_disc = c2d(controller_cont,Ts,'tustin')
compensated_system = controller_disc*wheelA_tf_filt;
closed_loop = compensated_system/(1+compensated_system);



f_crossover_LF=3.1415; 
T_i_LF=tand(90-15)/f_crossover_LF;
K_LF=0.4741;
controller_cont_LF = tf([K_LF, K_LF/T_i_LF], [1, 0]);
controller_disc_LF = c2d(controller_cont_LF,Ts,'tustin')
compensated_system_LF = controller_disc_LF*wheelA_tf_filt;
closed_loop_LF = compensated_system_LF/(1+compensated_system_LF);
meas_speed = plot_data(:,9);
ref = plot_data(:,4);
simulation_response = lsim(closed_loop,ref,time_int);

figure(2)
plot_sim = plot(time_int,simulation_response,'r');
hold on
plot_ref = plot(time_int,ref,'g','LineStyle','--');
plot_measured = plot(time_int,meas_speed,'b','LineStyle',':');
legend([plot_sim(1),plot_ref(1),plot_measured(1)],{'Simulated closed-loop response','Step reference','Measured closed-loop response'})
title('Step reference plot wheel A (Incline)')
xlabel('Time [s]')
ylabel('Rotational speed [rad/s]')
measured_speed_LF = plot_data_LF(:,9);
reference_LF = plot_data_LF(:,4);
simulation_response_LF = lsim(closed_loop_LF,reference_LF,time_int);

plot_sim_LF = plot(time_int,simulation_response_LF,'r');
plot_ref_LF = plot(time_int,reference_LF,'g','LineStyle','--');
plot_meas_LF = plot(time_int,measured_speed_LF,'b','LineStyle',':');
legend([plot_sim_LF(1),plot_ref_LF(1),plot_meas_LF(1)],{'Simulated closed-loop response','Step reference','Measured closed-loop response'})
title('Step reference plot wheel A (Incline with 0.5Hz)')
xlabel('Time [s]')
ylabel('Rotational speed [rad/s]')
hold off
ref = plot_data(:,4);
meas_speed = plot_data(:,10);
simulation_response = lsim(closed_loop,ref,time_int);

figure
plot_sim = plot(time_int,simulation_response,'r');
hold on
plot_ref = plot(time_int,ref,'g','LineStyle','--');
plot_measured = plot(time_int,meas_speed,'b','LineStyle',':');
legend([plot_sim(1),plot_ref(1),plot_measured(1)],{'Simulated closed-loop response','Step reference','Measured closed-loop response'})
title('Step reference plot wheel B (Incline)')
xlabel('Time [s]')
ylabel('Rotational speed [rad/s]')

reference_LF = plot_data_LF(:,4);
measured_speed_LF = plot_data_LF(:,10);
simulation_response_LF = lsim(closed_loop_LF,reference_LF,time_int);
plot_sim_LF = plot(time_int,simulation_response_LF,'r');
plot_ref_LF = plot(time_int,reference_LF,'g','LineStyle','--');
plot_meas_LF = plot(time_int,measured_speed_LF,'b','LineStyle',':');
legend([plot_sim_LF(1),plot_ref_LF(1),plot_meas_LF(1)],{'Simulated closed-loop response','Step reference','Measured closed-loop response'})
title('Step reference plot wheel B (Incline with 0.5Hz)')
xlabel('Time [s]')
ylabel('Rotational speed [rad/s]')
hold off

tracking_error = plot_data(:,5);
tracking_error_tf = 1/(1+compensated_system);
tracking_error_sim_response = lsim(tracking_error_tf,ref,time_int);
figure
plot_error_sim = plot(time_int,tracking_error_sim_response,'r');
hold on
plot_error = plot(time_int,tracking_error,'b','LineStyle','--');
legend([plot_error_sim(1),plot_error(1)],{'Simulated tracking error','Measured tracking error'})
title('Tracking error wheel A (Incline)')
xlabel('Time [s]')
ylabel('Tracking error [rad/s]')

tracking_error_LF = plot_data_LF(:,5);
tracking_error_tf_LF = 1/(1+compensated_system_LF);
tracking_error_sim_response_LF = lsim(tracking_error_tf_LF,reference_LF,time_int);
plot_error_sim_LF = plot(time_int,tracking_error_sim_response_LF,'r');
plot_error_LF = plot(time_int,tracking_error_LF,'b','LineStyle','--');
legend([plot_error_sim_LF(1),plot_error_LF(1)],{'Simulated tracking error','Measured tracking error'})
title('Tracking error wheel A (Incline with 0.5Hz)')
xlabel('Time [s]')
ylabel('Tracking error [rad/s]')


tracking_error = plot_data(:,6);
tracking_error_tf = 1/(1+compensated_system);
tracking_error_sim_response = lsim(tracking_error_tf,ref,time_int);
figure
plot_error_sim = plot(time_int,tracking_error_sim_response,'r');
hold on
plot_error = plot(time_int,tracking_error,'b','LineStyle','--');
legend([plot_error_sim(1),plot_error(1)],{'Simulated tracking error','Measured tracking error'})
title('Tracking error wheel B (Incline)')
xlabel('Time [s]')
ylabel('Tracking error [rad/s]')

tracking_error_LF = plot_data_LF(:,6);
tracking_error_tf_LF = 1/(1+compensated_system_LF);
tracking_error_sim_response_LF = lsim(tracking_error_tf_LF,reference_LF,time_int);
plot_error_sim_LF = plot(time_int,tracking_error_sim_response_LF,'r');
plot_error_LF = plot(time_int,tracking_error_LF,'b','LineStyle','--');
legend([plot_error_sim_LF(1),plot_error_LF(1)],{'Simulated tracking error','Measured tracking error'})
title('Tracking error wheel B (Incline with 0.5Hz)')
xlabel('Time [s]')
ylabel('Tracking error [rad/s]')


control_volt = plot_data(:,7);
control_volt_tf = controller_disc/(1+compensated_system);
control_volt_sim = lsim(control_volt_tf,ref,time_int);
figure
plot_control_sim = plot(time_int,control_volt_sim,'r');
hold on
plot_control_meas = plot(time_int,control_volt,'b','LineStyle','--');
legend([plot_control_sim(1),plot_control_meas(1)],{'Simulated control signal','Measured control signal'})
title('Control signal wheel A (Incline)')
xlabel('Time [s]')
ylabel('Voltage [V]')

control_volt_LF = plot_data_LF(:,7);
control_volt_tf_LF = controller_disc_LF/(1+compensated_system_LF);
control_volt_sim_LF = lsim(control_volt_tf_LF,reference_LF,time_int);
plot_control_sim_LF = plot(time_int,control_volt_sim_LF,'r');
plot_control_meas_LF = plot(time_int,control_volt_LF,'b','LineStyle','--');
legend([plot_control_sim_LF(1),plot_control_meas_LF(1)],{'Simulated control signal','Measured control signal'})
title('Control signal wheel A (Incline with 0.5Hz)')
xlabel('Time [s]')
ylabel('Voltage [V]')


control_volt = plot_data(:,8);
control_volt_tf = controller_disc/(1+compensated_system);
control_volt_sim = lsim(control_volt_tf,ref,time_int);
figure
plot_control_sim = plot(time_int,control_volt_sim,'r');
hold on
plot_control_meas = plot(time_int,control_volt,'b','LineStyle','--');
legend([plot_control_sim(1),plot_control_meas(1)],{'Simulated control signal','Measured control signal'})
title('Control signal wheel B (Incline)')
xlabel('Time [s]')
ylabel('Voltage [V]')

control_volt_LF = plot_data_LF(:,8);
control_volt_tf_LF = controller_disc_LF/(1+compensated_system_LF);
control_volt_sim_LF = lsim(control_volt_tf_LF,reference_LF,time_int);
plot_control_sim_LF = plot(time_int,control_volt_sim_LF,'r');
plot_control_meas_LF = plot(time_int,control_volt_LF,'b','LineStyle','--');
legend([plot_control_sim_LF(1),plot_control_meas_LF(1)],{'Simulated control signal','Measured control signal'})
title('Control signal wheel B (Incline with 0.5Hz)')
xlabel('Time [s]')
ylabel('Voltage [V]')
