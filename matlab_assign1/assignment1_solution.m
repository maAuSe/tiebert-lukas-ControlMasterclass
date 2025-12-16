clear all
close all
clear global
clc


%% 2.1.2 Loading, plotting and noise detection
% -------------------------------------------

% plot time domain data
%load assignment1_data.m
N = length(timeVector);
fs = 100; % sampling frequency [Hz]
Ts = 1/fs; % sampling period [s]
timePeriod = 14; % time period of cycle [s]

points_per_period = timePeriod/Ts;
num_periods = N/points_per_period;
timeVectorInSec= [0:0.01:27.99]' ;
timeVectorToPlot = timeVectorInSec;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 1: input voltage plotted over time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1),hold on
sgtitle('Excitation voltage to motors')
plot(timeVectorInSec, voltage, 'LineWidth', 1)
grid on
axis tight
xlabel('t [s]')
ylabel('voltage [V]')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 2: outputs omegaA,omegaB plotted over time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2),hold on
sgtitle('Measured output data in time domain')

subplot(2,1,1),plot(timeVector, omegaA, 'LineWidth', 1)
grid on
axis tight
xlabel('t [s]')
ylabel('omegaA [rad/s]')

subplot(2,1,2),plot(timeVector, omegaB, 'LineWidth', 1)
grid on
axis tight
xlabel('t [s]')
ylabel('omegaB [rad/s]')




%% overlap the plot for different periods to appreciate the noise
% separate the different periods (one per column)
omegaA_matrix = reshape(omegaA,points_per_period,num_periods); 
omegaB_matrix = reshape(omegaB,points_per_period,num_periods);
voltage_matrix = reshape(voltage,points_per_period,num_periods);

% lets compute the mean of the signals across the periods to have a point of comparison to assess the noise
omegaA_mean = mean(omegaA_matrix,2);
omegaB_mean = mean(omegaB_matrix,2);
voltage_mean = mean(voltage_matrix,2);

domegaA_matrix = omegaA_matrix - repmat(omegaA_mean,1,num_periods);
domegaB_matrix = omegaB_matrix - repmat(omegaB_mean,1,num_periods);
dvoltage_matrix = voltage_matrix - repmat(voltage_mean,1,num_periods);

% plotting some interesting comparisons

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 3: omegaA,omegaB,voltage noise plotted over 1 period 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3),hold on
sgtitle('Comparison across periods to assess noise')

subplot(2,3,1),plot(timeVector(1:points_per_period), omegaA_matrix, 'LineWidth', 1) 
grid on
axis tight
xlabel('t  [s]')
ylabel('omegaA [rad/s]')

subplot(2,3,2),plot(timeVector(1:points_per_period), omegaB_matrix, 'LineWidth', 1) 
grid on
axis tight
xlabel('t  [s]')
ylabel('omegaB [rad/s]')

subplot(2,3,3),plot(timeVector(1:points_per_period), voltage_matrix, 'LineWidth', 1)
grid on
axis tight
xlabel('t  [s]')
ylabel('voltage [V]')

subplot(2,3,4),plot(timeVector(1:points_per_period), domegaA_matrix, 'LineWidth', 1)
grid on
axis tight
xlabel('t  [s]')
ylabel('\Delta omegaA [rad/s]')

subplot(2,3,5),plot(timeVector(1:points_per_period), domegaB_matrix, 'LineWidth', 1)
grid on
axis tight
xlabel('t  [s]')
ylabel('\Delta omegaB [rad/s]')

subplot(2,3,6),plot(timeVector(1:points_per_period), dvoltage_matrix, 'LineWidth', 1)
grid on
axis tight
xlabel('t  [s]')
ylabel('\Delta voltage [V]')

%% 2.1.3 LLS without data filtering
% -------------------------------------------
% assume the model to have the same shape of the discrete model we obtained

% H(z) = (b1 z + b2)/(z^3 + a1 z^2 + a2 z)

% collect the signals appearing in the difference equation
b_A = omegaA(3:end);
A_A = [-omegaA(2:end-1), -omegaA(1:end-2), voltage(2:end-1), voltage(1:end-2)];

b_B = omegaB(3:end);
A_B = [-omegaB(2:end-1), -omegaB(1:end-2), voltage(2:end-1), voltage(1:end-2)];

% perform the fit to get the desired parameters
theta_A = A_A\b_A

theta_B = A_B\b_B

% theta = [a1 a2 b1 b2]'

% build the identified model
%Num1 = [0, theta(3), theta(4)];
%Den1 = [1, theta(1) theta(2)];
%sys_d1 = tf(Num1, Den1, Ts)

% % alternative way to construct transfer function
z = tf('z', Ts);

num_unfilt_A = [theta_A(3), theta_A(4)];
den_unfilt_A = [1, theta_A(1), theta_A(2)];
sys_d1_A = tf(num_unfilt_A, den_unfilt_A, Ts)

num_unfilt_B = [theta_B(3), theta_B(4)];
den_unfilt_B = [1, theta_B(1), theta_B(2)];
sys_d1_B = tf(num_unfilt_B, den_unfilt_B, Ts)



% plot the results

omegaA_model = lsim(sys_d1_A, voltage, timeVectorToPlot);

omegaB_model = lsim(sys_d1_B, voltage, timeVectorToPlot);

omegaA_rms_error = sqrt(mean((omegaA - omegaA_model).^2));
omegaB_rms_error = sqrt(mean((omegaB - omegaB_model).^2));

fprintf('RMS error (omegaA vs. model): %.4f rad/s\n', omegaA_rms_error);
fprintf('RMS error (omegaB vs. model): %.4f rad/s\n', omegaB_rms_error);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 4: omegaA experiments vs. model %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(4), hold on
sgtitle('LLS without low-pass filtering (motor A)')

subplot(2,1,1)
plot(timeVectorToPlot, omegaA, 'k-', ...        % solid black
     timeVectorToPlot, omegaA_model, 'k--');    % dashed black
legend('empirical','estimated','Location','SouthWest')
xlabel('time [s]')
ylabel('omegaA [rad/s]')
axis tight

subplot(2,1,2),plot(timeVectorToPlot, omegaA-omegaA_model)
legend('error')
xlabel('time [s]')
ylabel('omegaA-error [rad/s]')
axis tight

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 5: omegaB experiments vs. model %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(5), hold on
sgtitle('LLS without low-pass filtering (motor B)')

subplot(2,1,1)
plot(timeVectorToPlot, omegaB, 'k-', ...        % solid black
     timeVectorToPlot, omegaB_model, 'k--');    % dashed black
legend('empirical','estimated','Location','SouthWest')
xlabel('time [s]')
ylabel('omegaB [rad/s]')
axis tight

subplot(2,1,2),plot(timeVectorToPlot, omegaB-omegaB_model)
legend('error')
xlabel('time [s]')
ylabel('omegaB-error [rad/s]')
axis tight


% analyse step response of model in detail: calculate from model and
% measure from step response: peak value, peak time, steady state value
% to do that, model is first transformed back to CT and step response of CT model
% is simulated. Also zeta, wn, wd and DC-gain of CT model are calculated and used
% ALSO compare these values with values derived from the provided
% measurement

% transform model back to CT
% Use 'tustin' method because sys_d1 has a pole at z=0

sys_c1_A = d2c(sys_d1_A, 'tustin');
pc1_A = pole(sys_c1_A); % poles of the CT system 
wd1_A = abs(imag(pc1_A(1))); 

[wn1_A,zeta1_A] = damp(sys_c1_A);



sys_c1_B = d2c(sys_d1_B, 'tustin');
pc1_B = pole(sys_c1_B); % poles of the CT system 
wd1_B = abs(imag(pc1_B(1))); 

[wn1_B,zeta1_B] = damp(sys_c1_B);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 6: step response of the CT system (motor A) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(6)
step(sys_d1_A), grid  
xlabel('time')
ylabel('step response [-]')
title('Step response of the CT system (motor A)')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 7: step response of the CT system motor B %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(7)
step(sys_d1_B), grid  
xlabel('time')
ylabel('step response [-]')
title('Step response of the CT system (motor B)')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 8: omegaA averaged over the periods %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(8)
sgtitle('Velocity of wheel A averaged over the periods')
plot(timeVectorToPlot(1:points_per_period),omegaA_mean) 
xlabel('time [s]')
ylabel('omegaA-mean [rad/s]')
grid


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 9: omegaB averaged over the periods %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(9)
sgtitle('Velocity of wheel B averaged over the periods')
plot(timeVectorToPlot(1:points_per_period),omegaB_mean) 
xlabel('time [s]')
ylabel('omegaB-mean [rad/s]')
grid


%%% relative peak value: value of response at peak/steady state value

%y_peak_A = ...;     % obtained from figure(5)
%Mp1_A = y_peak_A - 1
Mp1_model_A = exp(-pi*zeta1_A(1)/sqrt(1-zeta1_A(1)^2)); % formula slide 10, C6
%Mp1_measurement_A = ...  % obtained from figure(6)

%y_peak_B = ...;     % obtained from figure(5)
%Mp1_B = y_peak_B - 1
Mp1_model_B = exp(-pi*zeta1_B(1)/sqrt(1-zeta1_B(1)^2)); % formula slide 10, C6
%Mp1_measurement_B = ...  % obtained from figure(6)




%%% peak time

%tp1_A = ...    % obtained from figure(5)
tp1_model_A = pi/wd1_A;      % formula slide 9, C6
%tp1_measurement_A = ...   % obtained from figure(6)

%tp1_B = ...    % obtained from figure(5)
tp1_model_B = pi/wd1_B;      % formula slide 9, C6
%tp1_measurement_B = ...   % obtained from figure(6)



%%% steady state value

[Numc1_A, Denc1_A] = tfdata(sys_c1_A);
Numc1_A = Numc1_A{1};
Denc1_A = Denc1_A{1};
y_ss1_model_A = Numc1_A(end)/Denc1_A(end); % DC gain of CT model motor A (evaluating transfer function at s=0) 
%y_ss1_A = ... %obtained from figure(5)
%y_ss1_measurement_A = ... %obtained from figure(6)

[Numc1_B, Denc1_B] = tfdata(sys_c1_B);
Numc1_B = Numc1_B{1};
Denc1_B = Denc1_B{1};
y_ss1_model_B = Numc1_B(end)/Denc1_B(end); % DC gain of CT model motor B (evaluating transfer function at s=0) 
%y_ss1_B = ... %obtained from figure(5)
%y_ss1_measurement_B = ... %obtained from figure(6)






%% 2.1.4 LLS with low-pass filter applied to the input and output data
% -------------------------------------------------------------------
% identify two filtered models: cutoff at 20 Hz and 35 Hz
cutoff_freq_20 = 20;  % Hz
cutoff_freq_35 = 35;  % Hz

[B_filt20, A_filt20] = butter(8, cutoff_freq_20/(fs/2));
[B_filt35, A_filt35] = butter(8, cutoff_freq_35/(fs/2));

% apply the filters to both input and output for motor A
omegaA_filt20 = filtfilt(B_filt20, A_filt20, omegaA); 
voltage_filt_A20 = filtfilt(B_filt20, A_filt20, voltage);

omegaA_filt35 = filtfilt(B_filt35, A_filt35, omegaA); 
voltage_filt_A35 = filtfilt(B_filt35, A_filt35, voltage);

% apply the filters to both input and output for motor B
omegaB_filt20 = filtfilt(B_filt20, A_filt20, omegaB); 
voltage_filt_B20 = filtfilt(B_filt20, A_filt20, voltage);

omegaB_filt35 = filtfilt(B_filt35, A_filt35, omegaB); 
voltage_filt_B35 = filtfilt(B_filt35, A_filt35, voltage);


% repeat the identification for both cutoffs
b_A20 = omegaA_filt20(3:end);
A_A20 = [-omegaA_filt20(2:end-1), -omegaA_filt20(1:end-2), voltage_filt_A20(2:end-1), voltage_filt_A20(1:end-2)];

b_A35 = omegaA_filt35(3:end);
A_A35 = [-omegaA_filt35(2:end-1), -omegaA_filt35(1:end-2), voltage_filt_A35(2:end-1), voltage_filt_A35(1:end-2)];

b_B20 = omegaB_filt20(3:end);
A_B20 = [-omegaB_filt20(2:end-1), -omegaB_filt20(1:end-2), voltage_filt_B20(2:end-1), voltage_filt_B20(1:end-2)];

b_B35 = omegaB_filt35(3:end);
A_B35 = [-omegaB_filt35(2:end-1), -omegaB_filt35(1:end-2), voltage_filt_B35(2:end-1), voltage_filt_B35(1:end-2)];

% H(z) = (b1 z + b2)/(z^3 + a1 z^2 + a2 z)

theta_filter_A20 = A_A20\b_A20
theta_filter_A35 = A_A35\b_A35

theta_filter_B20 = A_B20\b_B20
theta_filter_B35 = A_B35\b_B35

num_filter_A20 = [theta_filter_A20(3), theta_filter_A20(4)];
den_filter_A20 = [1, theta_filter_A20(1), theta_filter_A20(2)];
sys_d2_A20 = tf(num_filter_A20, den_filter_A20, Ts)

num_filter_A35 = [theta_filter_A35(3), theta_filter_A35(4)];
den_filter_A35 = [1, theta_filter_A35(1), theta_filter_A35(2)];
sys_d2_A35 = tf(num_filter_A35, den_filter_A35, Ts)

num_filter_B20 = [theta_filter_B20(3), theta_filter_B20(4)];
den_filter_B20 = [1, theta_filter_B20(1), theta_filter_B20(2)];
sys_d2_B20 = tf(num_filter_B20, den_filter_B20, Ts)

num_filter_B35 = [theta_filter_B35(3), theta_filter_B35(4)];
den_filter_B35 = [1, theta_filter_B35(1), theta_filter_B35(2)];
sys_d2_B35 = tf(num_filter_B35, den_filter_B35, Ts)


% simulate models with measured input (for comparison) and with filtered
% input (for validation)
omegaA_model20_measured = lsim(sys_d2_A20, voltage, timeVectorToPlot);
omegaA_model35_measured = lsim(sys_d2_A35, voltage, timeVectorToPlot);
omegaB_model20_measured = lsim(sys_d2_B20, voltage, timeVectorToPlot);
omegaB_model35_measured = lsim(sys_d2_B35, voltage, timeVectorToPlot);

omegaA_model20_filt = lsim(sys_d2_A20, voltage_filt_A20, timeVectorToPlot);
omegaA_model35_filt = lsim(sys_d2_A35, voltage_filt_A35, timeVectorToPlot);
omegaB_model20_filt = lsim(sys_d2_B20, voltage_filt_B20, timeVectorToPlot);
omegaB_model35_filt = lsim(sys_d2_B35, voltage_filt_B35, timeVectorToPlot);

% RMS errors against measured data
omegaA_rms_error_model20 = sqrt(mean((omegaA - omegaA_model20_measured).^2));
omegaA_rms_error_model35 = sqrt(mean((omegaA - omegaA_model35_measured).^2));
omegaB_rms_error_model20 = sqrt(mean((omegaB - omegaB_model20_measured).^2));
omegaB_rms_error_model35 = sqrt(mean((omegaB - omegaB_model35_measured).^2));

fprintf('RMS error (omegaA vs. model 2(c) with 20 Hz cutoff on measured input): %.4f rad/s\n', omegaA_rms_error_model20);
fprintf('RMS error (omegaA vs. model 2(c) with 35 Hz cutoff on measured input): %.4f rad/s\n', omegaA_rms_error_model35);
fprintf('RMS error (omegaB vs. model 2(c) with 20 Hz cutoff on measured input): %.4f rad/s\n', omegaB_rms_error_model20);
fprintf('RMS error (omegaB vs. model 2(c) with 35 Hz cutoff on measured input): %.4f rad/s\n', omegaB_rms_error_model35);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 10: measured response vs. models (motor A) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
omegaA_err_unfilt = omegaA - omegaA_model;
omegaA_err_20     = omegaA - omegaA_model20_measured;
omegaA_err_35     = omegaA - omegaA_model35_measured;

figure(10), hold on
sgtitle('Measured vs simulated (motor A)')

subplot(2,1,1)
plot(timeVectorToPlot, omegaA, 'k-', ...
     timeVectorToPlot, omegaA_model, 'k--', ...
     timeVectorToPlot, omegaA_model20_measured, 'k-.', ...
     timeVectorToPlot, omegaA_model35_measured, 'k:');
legend('measured','model 2(b)','model 2(c) 20 Hz','model 2(c) 35 Hz','Location','SouthWest')
xlabel('time [s]')
axis tight
ylabel('omegaA [rad/s]')

subplot(2,1,2)
plot(timeVectorToPlot, omegaA_err_unfilt, 'k--', ...
     timeVectorToPlot, omegaA_err_20, 'k-.', ...
     timeVectorToPlot, omegaA_err_35, 'k:');
legend('error 2(b)','error 2(c) 20 Hz','error 2(c) 35 Hz','Location','SouthWest')
xlabel('time [s]')
ylabel('omegaA error [rad/s]')
axis tight


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 11: measured response vs. models (motor B) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
omegaB_err_unfilt = omegaB - omegaB_model;
omegaB_err_20     = omegaB - omegaB_model20_measured;
omegaB_err_35     = omegaB - omegaB_model35_measured;

figure(11), hold on
sgtitle('Measured vs simulated (motor B)')

subplot(2,1,1)
plot(timeVectorToPlot, omegaB, 'k-', ...
     timeVectorToPlot, omegaB_model, 'k--', ...
     timeVectorToPlot, omegaB_model20_measured, 'k-.', ...
     timeVectorToPlot, omegaB_model35_measured, 'k:');
legend('measured','model 2(b)','model 2(c) 20 Hz','model 2(c) 35 Hz','Location','SouthWest')
xlabel('time [s]')
axis tight
ylabel('omegaB [rad/s]')

subplot(2,1,2)
plot(timeVectorToPlot, omegaB_err_unfilt, 'k--', ...
     timeVectorToPlot, omegaB_err_20, 'k-.', ...
     timeVectorToPlot, omegaB_err_35, 'k:');
legend('error 2(b)','error 2(c) 20 Hz','error 2(c) 35 Hz','Location','SouthWest')
xlabel('time [s]')
ylabel('omegaB error [rad/s]')
axis tight


% analyse step response of model in detail: calculate from model and
% measure from step response: peak value, peak time, steady state value
% before that, model is transformed back to CT and step response of CT model
% is simulated. Also zeta, wn, wd and DC-gain of CT model are calculated and used
% ALSO compare these values with values derived from the provided
% measurement

%transform 35 Hz filtered model back to CT

sys_c2_A = d2c(sys_d2_A35, 'tustin')
pc2_A = pole(sys_c2_A);
wd2_A = abs(imag(pc2_A(2)));

[wn2_A,zeta2_A] = damp(sys_c2_A);


sys_c2_B = d2c(sys_d2_B35, 'tustin')
pc2_B = pole(sys_c2_B);
wd2_B = abs(imag(pc2_B(2)));

[wn2_B,zeta2_B] = damp(sys_c2_B);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 16: step response of the CT system (filtered model, motor A) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(16)
step(sys_d2_A35);grid
xlabel('time')
ylabel('step response [-]')
title('Step response of the CT system (filtered model, motor A)')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 17: step response of the CT system (filtered model, motor B) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(17)
step(sys_d2_B35);grid
xlabel('time')
ylabel('step response [-]')
title('Step response of the CT system (filtered model, motor B)')





%%% relative peak value: value of response at peak/steady state value

%y_peak_A = ...; %obtained from figure(8)
%Mp_A = y_peak_A - 1
Mp_model_A = exp(-pi*zeta2_A(1)/sqrt(1-zeta2_A(1)^2)); %formula slide 10, C6
%Mp_measurement_A = ...  %obtained from figure(6)

%y_peak_B = ...; %obtained from figure(8)
%Mp_B = y_peak_B - 1
Mp_model_B = exp(-pi*zeta2_B(1)/sqrt(1-zeta2_B(1)^2)); %formula slide 10, C6
%Mp_measurement_B = ...  %obtained from figure(6)




%%% peak time

%tp_A = ... %obtained from figure(8)
tp_model_A = pi/wd2_A; %formula slide 9, C6
%tp_measurement_A = ... %obtained from figure(6)

%tp_B = ... %obtained from figure(8)
tp_model_B = pi/wd2_B; %formula slide 9, C6
%tp_measurement_B = ... %obtained from figure(6)



%%% steady state value

[Numc2_A, Denc2_A] = tfdata(sys_c2_A);
Numc2_A = Numc2_A{1};
Denc2_A = Denc2_A{1};
y_ss_model_A = Numc2_A(end)/Denc2_A(end); %DC gain of CT model motor A (evaluating transfer function at s=0) 
%y_ss_A = ... %obtained from figure(8)
%y_ss_measurement_A = ... %obtained from figure(6)

[Numc2_B, Denc2_B] = tfdata(sys_c2_B);
Numc2_B = Numc2_B{1};
Denc2_B = Denc2_B{1};
y_ss_model_B = Numc2_B(end)/Denc2_B(end); %DC gain of CT model motor B(evaluating transfer function at s=0) 
%y_ss_B = ... %obtained from figure(8)
%y_ss_measurement_B = ... %obtained from figure(6)
