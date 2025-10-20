clear all
close all
clear global
clc

m = 3;  % [kg]
c = 12; % [Ns/m] 
k = 300; % [N/m]
sys_c = tf(1,[m c k]);

% % alternative way to construct transfer function
% s = tf('s');
% sys_c = 1/(m*s^2 + c*s + k);

p = pole(sys_c)
fs = ceil(20*imag(p(1)/(2*pi)));
Ts = 1/fs;
sys_d = c2d(sys_c,Ts,'zoh');

%% 2.1.2 Loading, plotting and noise detection
% -------------------------------------------

% plot time domain data
load time_data_2024 % F_t, x_t, t and num_periods
N = length(x_t);

points_per_period = N/num_periods;

figure(1),hold on
sgtitle('Measured data in time domain')
subplot(2,1,1),plot(t, x_t, 'LineWidth', 1)
grid on
axis tight
xlabel('t [s]')
ylabel('x [m]')
subplot(2,1,2), plot(t, F_t, 'LineWidth', 1)
grid on
axis tight
xlabel('t [s]')
ylabel('F [N]')


%% overlap the plot for different periods to appreciate the noise
% separate the different periods (one per column)
x_matrix = reshape(x_t,points_per_period,num_periods); 
F_matrix = reshape(F_t,points_per_period,num_periods);

% lets compute the mean of the signals across the periods to have a point of comparison to assess the noise
x_mean = mean(x_matrix,2);
F_mean = mean(F_matrix,2);
dx_matrix = x_matrix - repmat(x_mean,1,num_periods);
dF_matrix = F_matrix - repmat(F_mean,1,num_periods);

% plotting some interesting comparisons
figure(2),hold on
sgtitle('Comparison across periods to assess noise')
subplot(2,2,1),plot(t(1:points_per_period), x_matrix, 'LineWidth', 1) 
grid on
axis tight
xlabel('t  [s]')
ylabel('x  [m]')

subplot(2,2,3),plot(t(1:points_per_period), F_matrix, 'LineWidth', 1)
grid on
axis tight
xlabel('t  [s]')
ylabel('F  [N]')
subplot(2,2,2),plot(t(1:points_per_period), dx_matrix, 'LineWidth', 1)
grid on
axis tight
xlabel('t  [s]')
ylabel('\Delta x  [m]')
subplot(2,2,4)
plot(t(1:points_per_period), dF_matrix, 'LineWidth', 1)
grid on
axis tight
xlabel('t  [s]')
ylabel('\Delta F  [N]')

%% 2.1.3 LLS without data filtering
% -------------------------------------------
% assume the model to have the same shape of the discrete model we obtained
% on paper: H(z) = (b1 z + b0)/(z^2 + a1 z + a0)

% collect the signals appearing in the difference equation
b = x_t(3:end);
A = [-x_t(2:end-1), -x_t(1:end-2), F_t(2:end-1), F_t(1:end-2)];
% perform the fit to get the desired parameters
theta = A\b;

% build the identified model
Num1 = [0, theta(3), theta(4)];
Den1 = [1, theta(1) theta(2)];
sys_d1 = tf(Num1, Den1, Ts);

% % alternative way to construct transfer function
% z = tf('z', Ts);
% sys_d1 = (theta(3)*z + theta(4))/(z^2 + theta(1)*z + theta(2));

% plot the results
x1 = lsim(sys_d1,F_t,t);

figure(3), hold on
sgtitle('LLS without low-pass filtering')
subplot(2,1,1)
plot(t,[x_t x1]);
legend('empirical','estimated','Location','SouthWest')
xlabel('time [s]')
ylabel('displacement [m]')
axis tight
subplot(2,1,2),plot(t,x_t-x1)
legend('error')
xlabel('time [s]')
ylabel('displacement [m]')
axis tight

% analyse step response of model in detail: calculate from model and
% measure from step response: peak value, peak time, steady state value
% to do that, model is first transformed back to CT and step response of CT model
% is simulated. Also zeta, wn, wd and DC-gain of CT model are calculated and used
% ALSO compare these values with values derived from the provided
% measurement

% transform model back to CT
sys_c1 = d2c(sys_d1);
pc1 = pole(sys_c1);
wd1 = abs(imag(pc1(1)));

[wn1,zeta1] = damp(sys_c1);
figure(103)
step(sys_d1), grid

figure(1003)
plot(t(1:1070),x_mean)
xlabel('time [s]')
ylabel('displacement [m]')
grid

% relative peak value: value of response at peak/steady state value
y_peak = 0.00439/0.00334; %obtained from figure(103)
Mp1 = y_peak-1
Mp1_model = exp(-pi*zeta1(1)/sqrt(1-zeta1(1)^2)) %formula slide 10, C6
Mp1_measurement = 0.005081/0.003351 - 1  %obtained from figure(1003)

% peak time
tp1 = 0.312 %obtained from figure(103)
tp1_model = pi/wd1 %formula slide 9, C6
tp1_measurement = 8.125 - 7.813
 %obtained from figure(1003)

% steady state value
[Numc1, Denc1] = tfdata(sys_c1);
Numc1 = Numc1{1};
Denc1 = Denc1{1};
y_ss1_model = Numc1(end)/Denc1(end) %DC gain of CT model (evaluating transfer function at s=0) 
y_ss1 = 0.00334 %obtained from figure(103)
y_ss1_measurement = 0.003351 %obtained from figure(1003)


%% 2.1.4 LLS with low-pass filter applied to the input and output data
% -------------------------------------------------------------------
% define a low(band)-pass filter
pd1 = pole(sys_d1);
pc1 = log(pd1)/Ts;
interesting_frequency = (imag(pc1(1))/(2*pi));
cutoff = 1.5*interesting_frequency;
[B_filt,A_filt] = butter(6, cutoff/(fs/2));

% apply the filter to both input and output
x_filt = filtfilt(B_filt, A_filt, x_t); 
F_filt = filtfilt(B_filt, A_filt, F_t);

% repeat the identification
b = x_filt(3:end);
A = [-x_filt(2:end-1), -x_filt(1:end-2), F_filt(2:end-1), F_filt(1:end-2)];
theta = A\b;
Num2 = [theta(3), theta(4)];
Den2 = [1, theta(1) theta(2)];
sys_d2 = tf(Num2, Den2, Ts);

% plot results
x2 = lsim(sys_d2,F_t,t);
figure(4), hold on
sgtitle('LLS with low-pass filtering')
subplot(2,1,1)
plot(t,[x_t x2]);
legend('empirical','estimated','Location','SouthWest')
xlabel('time [s]')
axis tight
ylabel('displacement [m]')
subplot(2,1,2),plot(t,x_t-x2)
legend('error')
xlabel('time [s]')
ylabel('displacement [m]')
axis tight

% analyse step response of model in detail: calculate from model and
% measure from step response: peak value, peak time, steady state value
% befor that, model is transformed back to CT and step response of CT model
% is simulated. Also zeta, wn, wd and DC-gain of CT model are calculated and used
% ALSO compare these values with values derived from the provided
% measurement

%transform model back to CT
sys_c2 = d2c(sys_d2);
pc2 = pole(sys_c2);
wd2 = abs(imag(pc2(2)));

[wn2,zeta2]=damp(sys_c2);
figure(104)
step(sys_d2);grid

% relative peak value: value of response at peak/steady state value
y_peak = 0.00508/0.00333; %obtained from figure(104)
Mp = y_peak-1
Mp_model = exp(-pi*zeta2(1)/sqrt(1-zeta2(1)^2)) %formula slide 10, C6
Mp_measurement = 0.005081/0.003351 - 1  %obtained from figure(1003)

% peak time
tp = 0.313 %obtained from figure(104)
tp_model = pi/wd2 %formula slide 9, C6
tp_measurement = 8.125 - 7.813 %obtained from figure(1003)

% steady state value
[Numc2, Denc2] = tfdata(sys_c2);
Numc2 = Numc2{1};
Denc2 = Denc2{1};
y_ss_model = Numc2(end)/Denc2(end) %DC gain of CT model (evaluating transfer function at s=0) 
y_ss = 0.00333 %obtained from figure(104)
y_ss_measurement = 0.003351 %obtained from figure(1003)

%% 2.1.4 LLS applied to the averaged output data
% -------------------------------------------------------------------
% collect the signals appearing in the difference equation
b = x_mean(3:end);
A = [-x_mean(2:end-1), -x_mean(1:end-2), F_t(2:1070-1), F_t(1:1070-2)];
% perform the fit to get the desired parameters
theta = A\b;

% build the identified model
Num3 = [0, theta(3), theta(4)];
Den3 = [1, theta(1) theta(2)];
sys_d3 = tf(Num3, Den3, Ts);

x3 = lsim(sys_d3,F_t,t);

% plot results
figure(5), hold on
sgtitle('LLS with averaging over periods')
subplot(2,1,1)
plot(t,[x_t x3]);
legend('empirical','estimated','Location','SouthWest')
xlabel('time [s]')
axis tight
ylabel('displacement [m]')
subplot(2,1,2),plot(t,x_t-x3)
legend('error')
xlabel('time [s]')
ylabel('displacement [m]')
axis tight

% compare all results 
figure(6),hold on
title('Compare all models')
subplot(2,1,1)
plot(t(1:1280),[x_t(1:1280) x1(1:1280) x2(1:1280), x3(1:1280)]);
legend('empirical','model 1', 'model 2', 'model 3','Location','SouthWest')
xlabel('time [s]')
ylabel('displacement [m]')
axis tight
subplot(2,1,2),plot(t(1:1280),[x_t(1:1280)-x1(1:1280),x_t(1:1280)-x2(1:1280), x_t(1:1280)-x3(1:1280)])
legend('error model 1', 'error model 2', 'error model 3')
xlabel('time [s]')
ylabel('displacement [m]')
axis tight

% in which aspects is are the new models better than the old?

%% 2.1.5 Extra: Iterative Identification based on Sanathanan Koerner method 
% -------------------------------------------------------------------------

% Iteration 1
% we start with model1.
% Filter data using denominator of model1
Den4 = Den1;
Num4 = [0 0 1];
x_filt = filtfilt(Num4, Den4, x_t); 
F_filt = filtfilt(Num4, Den4, F_t);

% Redo the parameter estimation with the filtered data
b = x_filt(3:end);
A = [-x_filt(2:end-1), -x_filt(1:end-2), F_filt(2:end-1), F_filt(1:end-2)];
theta = A\b;
Num4 = [theta(3), theta(4)];
Den4 = [1, theta(1) theta(2)];

% Iteration 2
% Use model from the previous step to improve result.
% Filter data using denominator of model3
Num4 = [0 0 1];
x_filt = filtfilt(Num4, Den4, x_t); 
F_filt = filtfilt(Num4, Den4, F_t);

% Redo the parameter estimation with the filtered data
b = x_filt(3:end);
A = [-x_filt(2:end-1), -x_filt(1:end-2), F_filt(2:end-1), F_filt(1:end-2)];
theta = A\b;
Num4 = [theta(3), theta(4)];
Den4 = [1, theta(1) theta(2)];


% Iteration 3 (last one)
% Use model from the previous step to improve result.
% Filter data using denominator of model3
Num4 = [0 0 1];
x_filt = filtfilt(Num4, Den4, x_t); 
F_filt = filtfilt(Num4, Den4, F_t);

% Redo the parameter estimation with the filtered data
b = x_filt(3:end);
A = [-x_filt(2:end-1), -x_filt(1:end-2), F_filt(2:end-1), F_filt(1:end-2)];
theta = A\b;
Num4 = [theta(3), theta(4)];
Den4 = [1, theta(1) theta(2)];

sys_d4 = tf(Num4, Den4, Ts);

% plot results
x4 = lsim(sys_d4,F_t,t);
figure(7), hold on
sgtitle('LLS after 3 iterations of the Sanathanan Koerner method')
subplot(2,1,1)
plot(t,[x_t x4]);
legend('empirical','estimated','Location','SouthWest')
xlabel('time [s]')
axis tight
ylabel('displacement [m]')
subplot(2,1,2),plot(t,x_t-x4)
legend('error')
xlabel('time [s]')
ylabel('displacement [m]')
axis tight

% analyse step response of model in detail: calculate rise time, settling time,
% steady state value of the measured data (data avalaible for the
% identification) ... and check these values for the obtained model.  


% compare last 2 results 
figure(8),hold on
title('Compare last two models')
subplot(2,1,1)
plot(t(1:1280),[x_t(1:1280) x2(1:1280) x4(1:1280)]);
legend('empirical','model 2 (low-pass filter)', 'model 4 (Sanathan Koerner)','Location','SouthWest')
xlabel('time [s]')
ylabel('displacement [m]')
axis tight
subplot(2,1,2),plot(t(1:1280),[x_t(1:1280)-x2(1:1280),x_t(1:1280)-x4(1:1280)])
legend('error model 2 (low-pass filter)', 'error model 4 (Sanathan Koerner)')
xlabel('time [s]')
ylabel('displacement [m]')
axis tight

% in which aspects is the new model better than the low-pass filtered one or is the improvement negligible?




