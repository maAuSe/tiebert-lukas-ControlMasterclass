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
timeVectorInSec = (0:0.01:27.99)';

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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 2: outputs omegaA,omegaB plotted over time %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 3: omegaA,omegaB,voltage noise plotted over 1 period %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3),hold on
sgtitle('Comparison across periods to assess noise')

subplot(2,3,1),plot(timeVector(1:points_per_period), omegaA_matrix, 'LineWidth', 1) 
grid on
axis tight
xlabel('t  [s]')
ylabel('omegaA  [rad/s]')

subplot(2,3,2),plot(timeVector(1:points_per_period), omegaB_matrix, 'LineWidth', 1) 
grid on
axis tight
xlabel('t  [s]')
ylabel('omegaB  [rad/s]')

subplot(2,3,3),plot(timeVector(1:points_per_period), voltage_matrix, 'LineWidth', 1)
grid on
axis tight
xlabel('t  [s]')
ylabel('voltage  [V]')

subplot(2,3,4),plot(timeVector(1:points_per_period), domegaA_matrix, 'LineWidth', 1)
grid on
axis tight
xlabel('t  [s]')
ylabel('\Delta omegaA  [rad/s]')

subplot(2,3,5),plot(timeVector(1:points_per_period), domegaB_matrix, 'LineWidth', 1)
grid on
axis tight
xlabel('t  [s]')
ylabel('\Delta omegaB  [rad/s]')

subplot(2,3,6),plot(timeVector(1:points_per_period), dvoltage_matrix, 'LineWidth', 1)
grid on
axis tight
xlabel('t  [s]')
ylabel('\Delta voltage  [V]')

%% 2.1.3 LLS without data filtering
% -------------------------------------------
% assume the model to have the same shape of the discrete model we obtained

% H(z) = b1 /(z^2 + a1 z)

% collect the signals appearing in the difference equation
b_A = omegaA(3:end);
A_A = [-omegaA(2:end-1), voltage(1:end-2)];

b_B = omegaB(3:end);
A_B = [-omegaB(2:end-1), voltage(1:end-2)];

% perform the fit to get the desired parameters for both motors
theta_A = A_A\b_A;
theta_B = A_B\b_B;
% theta = [a1 b1]'

% build the identified models
z = tf('z', Ts);
sys_d1_A = (theta_A(2))/(z^2 + theta_A(1)*z)
sys_d1_B = (theta_B(2))/(z^2 + theta_B(1)*z)

% plot the results

timeVectorToPlot = 0.01:0.01:28; % IN SECONDS (total time period = 28 s --- sampling period = 0.01 s)

omegaA_model = lsim(sys_d1_A,voltage,timeVectorToPlot);
omegaB_model = lsim(sys_d1_B,voltage,timeVectorToPlot);

omegaA_rms_error = sqrt(mean((omegaA - omegaA_model).^2));
omegaB_rms_error = sqrt(mean((omegaB - omegaB_model).^2));

fprintf('RMS error (omegaA vs. simplified model): %.4f rad/s\n', omegaA_rms_error);
fprintf('RMS error (omegaB vs. simplified model): %.4f rad/s\n', omegaB_rms_error);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 4: omegaA experiments vs. model %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(4), hold on
sgtitle('LLS without low-pass filtering (motor A, simplified)')

subplot(2,1,1)
plot(timeVectorToPlot, omegaA, 'k-', ...        % solid black (measured)
     timeVectorToPlot, omegaA_model, 'k--');    % dashed black (estimated model)
legend('empirical','estimated','Location','SouthWest')
xlabel('time [s]')
ylabel('omegaA [rad/s]')
axis tight

subplot(2,1,2),plot(timeVectorToPlot,omegaA-omegaA_model)
legend('error')
xlabel('time [s]')
ylabel('omegaA-error [rad/s]')
axis tight

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 5: omegaB experiments vs. model %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(5), hold on
sgtitle('LLS without low-pass filtering (motor B, simplified)')

subplot(2,1,1)
plot(timeVectorToPlot, omegaB, 'k-', ...        % solid black (measured)
     timeVectorToPlot, omegaB_model, 'k--');    % dashed black (estimated model)
legend('empirical','estimated','Location','SouthWest')
xlabel('time [s]')
ylabel('omegaB [rad/s]')
axis tight

subplot(2,1,2),plot(timeVectorToPlot,omegaB-omegaB_model)
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

% transform models back to CT (use 'tustin' method because sys_d1_* have a pole at z=0)
sys_c1_A = d2c(sys_d1_A, 'tustin')
pc1_A = pole(sys_c1_A); % poles of the CT system
if abs(imag(pc1_A(1))) > abs(imag(pc1_A(2)))
    wd1_A = abs(imag(pc1_A(1)));
else
    wd1_A = abs(imag(pc1_A(2)));
end

[wn1_A,zeta1_A] = damp(sys_c1_A);

sys_c1_B = d2c(sys_d1_B, 'tustin')
pc1_B = pole(sys_c1_B); % poles of the CT system
if abs(imag(pc1_B(1))) > abs(imag(pc1_B(2)))
    wd1_B = abs(imag(pc1_B(1)));
else
    wd1_B = abs(imag(pc1_B(2)));
end

[wn1_B,zeta1_B] = damp(sys_c1_B);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 6: step response of the CT system (motor A) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(6)
step(sys_d1_A), grid  
xlabel('time')
ylabel('step response [-]')
title('Step response of the CT system (motor A, simplified)')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 7: step response of the CT system (motor B) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(7)
step(sys_d1_B), grid  
xlabel('time')
ylabel('step response [-]')
title('Step response of the CT system (motor B, simplified)')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 6: omegaA averaged over the periods %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(8)
sgtitle('Velocity of wheel A averaged over the periods')
plot(timeVectorToPlot(1:points_per_period),omegaA_mean) 
xlabel('time [s]')
ylabel('omegaA-mean [rad/s]')
grid

figure(9)
sgtitle('Velocity of wheel B averaged over the periods')
plot(timeVectorToPlot(1:points_per_period),omegaB_mean) 
xlabel('time [s]')
ylabel('omegaB-mean [rad/s]')
grid

% relative peak value: value of response at peak/steady state value
%y_peak = 0.00439/0.00334;     % obtained from figure(5)
%Mp1 = y_peak-1
Mp1_model_A = exp(-pi*zeta1_A(1)/sqrt(1-zeta1_A(1)^2)); % formula slide 10, C6
Mp1_model_B = exp(-pi*zeta1_B(1)/sqrt(1-zeta1_B(1)^2)); % formula slide 10, C6
%Mp1_measurement = 0.005081/0.003351 - 1  % obtained from figure(8)

% peak time
%tp1 = 0.312    % obtained from figure(6)
tp1_model_A = pi/wd1_A;      % formula slide 9, C6
tp1_model_B = pi/wd1_B;      % formula slide 9, C6
%tp1_measurement = 8.125 - 7.813   % obtained from figure(8)

% steady state value
[Numc1_A, Denc1_A] = tfdata(sys_c1_A);
Numc1_A = Numc1_A{1};
Denc1_A = Denc1_A{1};
y_ss1_model_A = Numc1_A(end)/Denc1_A(end); % DC gain of CT model (evaluating transfer function at s=0) 

[Numc1_B, Denc1_B] = tfdata(sys_c1_B);
Numc1_B = Numc1_B{1};
Denc1_B = Denc1_B{1};
y_ss1_model_B = Numc1_B(end)/Denc1_B(end); % DC gain of CT model (evaluating transfer function at s=0) 


%% 2.1.4 LLS with low-pass filter applied to the input and output data
% -------------------------------------------------------------------
% match the filtering approach of the full third-order model (8th-order
% Butterworth at 35 Hz applied to both input and output)
cutoff_freq = 35; % Hz
[B_filt, A_filt] = butter(8, cutoff_freq/(fs/2));

% apply the same zero-phase filter to both motors
omegaA_filt = filtfilt(B_filt, A_filt, omegaA);
voltage_filt_A = filtfilt(B_filt, A_filt, voltage);

omegaB_filt = filtfilt(B_filt, A_filt, omegaB);
voltage_filt_B = filtfilt(B_filt, A_filt, voltage);

% repeat the identification for both motors
b_A_filt = omegaA_filt(3:end);
A_A_filt = [-omegaA_filt(2:end-1), voltage_filt_A(1:end-2)];

b_B_filt = omegaB_filt(3:end);
A_B_filt = [-omegaB_filt(2:end-1), voltage_filt_B(1:end-2)];

% H(z) = b1 /(z^2 + a1 z)

theta_filter_A = A_A_filt\b_A_filt;
theta_filter_B = A_B_filt\b_B_filt;
% theta_filter = [a1 b1]'

sys_d2_A = (theta_filter_A(2))/(z^2 + theta_filter_A(1)*z)
sys_d2_B = (theta_filter_B(2))/(z^2 + theta_filter_B(1)*z)

% plot results

omegaA_model2_filt = lsim(sys_d2_A,voltage_filt_A,timeVectorToPlot);
omegaB_model2_filt = lsim(sys_d2_B,voltage_filt_B,timeVectorToPlot);

omegaA_model2_raw = lsim(sys_d2_A,voltage,timeVectorToPlot);
omegaB_model2_raw = lsim(sys_d2_B,voltage,timeVectorToPlot);

omegaA_rms_error_filt = sqrt(mean((omegaA_filt - omegaA_model2_filt).^2));
omegaB_rms_error_filt = sqrt(mean((omegaB_filt - omegaB_model2_filt).^2));

fprintf('RMS error (omegaA_filt vs. simplified filtered model): %.4f rad/s\n', omegaA_rms_error_filt);
fprintf('RMS error (omegaB_filt vs. simplified filtered model): %.4f rad/s\n', omegaB_rms_error_filt);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 10: omegaA experiments vs. filtered + unfiltered models %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
omegaA_err_unfilt = omegaA - omegaA_model;
omegaA_err_filt_raw   = omegaA - omegaA_model2_raw;

figure(10), hold on
sgtitle('LLS comparison with/without low-pass filtering (motor A, simplified)')

subplot(2,1,1)
plot(timeVectorToPlot, omegaA, 'k-', ...
     timeVectorToPlot, omegaA_model, 'k--', ...
     timeVectorToPlot, omegaA_model2_raw, 'k:');
legend('empirical','model 2(b)','model 2(c)','Location','SouthWest')
xlabel('time [s]')
axis tight
ylabel('omegaA [rad/s]')

subplot(2,1,2)
plot(timeVectorToPlot, omegaA_err_unfilt, 'k--', ...
     timeVectorToPlot, omegaA_err_filt_raw, 'k:');
legend('error 2(b)','error 2(c)','Location','SouthWest')
xlabel('time [s]')
ylabel('omegaA error [rad/s]')
axis tight


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 11: omegaB experiments vs. filtered + unfiltered models %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
omegaB_err_unfilt = omegaB - omegaB_model;
omegaB_err_filt_raw   = omegaB - omegaB_model2_raw;

figure(11), hold on
sgtitle('LLS comparison with/without low-pass filtering (motor B, simplified)')

subplot(2,1,1)
plot(timeVectorToPlot, omegaB, 'k-', ...
     timeVectorToPlot, omegaB_model, 'k--', ...
     timeVectorToPlot, omegaB_model2_raw, 'k:');
legend('empirical','model 2(b)','model 2(c)','Location','SouthWest')
xlabel('time [s]')
axis tight
ylabel('omegaB [rad/s]')

subplot(2,1,2)
plot(timeVectorToPlot, omegaB_err_unfilt, 'k--', ...
     timeVectorToPlot, omegaB_err_filt_raw, 'k:');
legend('error 2(b)','error 2(c)','Location','SouthWest')
xlabel('time [s]')
ylabel('omegaB error [rad/s]')
axis tight


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 12: validation on filtered data (motor A) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(12), hold on
sgtitle('LLS with low-pass filtering data set (motor A, simplified)')

subplot(2,1,1)
plot(timeVectorToPlot, omegaA_filt, 'k-', ...
     timeVectorToPlot, omegaA_model2_filt, 'k--');
legend('filtered empirical','filtered-model','Location','SouthWest')
xlabel('time [s]')
axis tight
ylabel('omegaA_filt [rad/s]')

subplot(2,1,2)
plot(timeVectorToPlot, omegaA_filt - omegaA_model2_filt, 'k-')
legend('filtered error','Location','SouthWest')
xlabel('time [s]')
ylabel('omegaA_filt-error [rad/s]')
axis tight


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 13: validation on filtered data (motor B) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(13), hold on
sgtitle('LLS with low-pass filtering data set (motor B, simplified)')

subplot(2,1,1)
plot(timeVectorToPlot, omegaB_filt, 'k-', ...
     timeVectorToPlot, omegaB_model2_filt, 'k--');
legend('filtered empirical','filtered-model','Location','SouthWest')
xlabel('time [s]')
axis tight
ylabel('omegaB_filt [rad/s]')

subplot(2,1,2)
plot(timeVectorToPlot, omegaB_filt - omegaB_model2_filt, 'k-')
legend('filtered error','Location','SouthWest')
xlabel('time [s]')
ylabel('omegaB_filt-error [rad/s]')
axis tight


% analyse step response of model in detail: calculate from model and
% measure from step response: peak value, peak time, steady state value
% before that, models are transformed back to CT and step responses of the CT models
% are simulated. Also zeta, wn, wd and DC-gain of CT models are calculated and used.

sys_c2_A = d2c(sys_d2_A, 'tustin')
pc2_A = pole(sys_c2_A);
if abs(imag(pc2_A(1))) > abs(imag(pc2_A(2)))
    wd2_A = abs(imag(pc2_A(1)));
else
    wd2_A = abs(imag(pc2_A(2)));
end

[wn2_A,zeta2_A] = damp(sys_c2_A);

sys_c2_B = d2c(sys_d2_B, 'tustin')
pc2_B = pole(sys_c2_B);
if abs(imag(pc2_B(1))) > abs(imag(pc2_B(2)))
    wd2_B = abs(imag(pc2_B(1)));
else
    wd2_B = abs(imag(pc2_B(2)));
end

[wn2_B,zeta2_B] = damp(sys_c2_B);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 14: step response of the CT system (filtered model, motor A) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(14)
step(sys_d2_A);grid
xlabel('time')
ylabel('step response [-]')
title('Step response of the CT system (filtered model, motor A, simplified)')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 15: step response of the CT system (filtered model, motor B) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(15)
step(sys_d2_B);grid
xlabel('time')
ylabel('step response [-]')
title('Step response of the CT system (filtered model, motor B, simplified)')


% relative peak value: value of response at peak/steady state value
Mp_model_A = exp(-pi*zeta2_A(1)/sqrt(1-zeta2_A(1)^2)); %formula slide 10, C6
Mp_model_B = exp(-pi*zeta2_B(1)/sqrt(1-zeta2_B(1)^2)); %formula slide 10, C6

% peak time
tp_model_A = pi/wd2_A;      % formula slide 9, C6
tp_model_B = pi/wd2_B;      % formula slide 9, C6

% steady state value
[Numc2_A, Denc2_A] = tfdata(sys_c2_A);
Numc2_A = Numc2_A{1};
Denc2_A = Denc2_A{1};
y_ss_model_A = Numc2_A(end)/Denc2_A(end); %DC gain of CT model (evaluating transfer function at s=0)

[Numc2_B, Denc2_B] = tfdata(sys_c2_B);
Numc2_B = Numc2_B{1};
Denc2_B = Denc2_B{1};
y_ss_model_B = Numc2_B(end)/Denc2_B(end); %DC gain of CT model (evaluating transfer function at s=0)
