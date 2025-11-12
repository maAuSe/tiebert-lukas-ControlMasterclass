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
b = omegaA(3:end);
A = [-omegaA(2:end-1), voltage(1:end-2)];

% perform the fit to get the desired parameters
theta = A\b
% theta = [a1 b1]'

% build the identified model
%Num1 = [0, theta(2)];
%Den1 = [1, theta(1)];
%sys_d1 = tf(Num1, Den1, Ts)

% % alternative way to construct transfer function
z = tf('z', Ts);
sys_d1 = (theta(2))/(z^2 + theta(1)*z)

% plot the results

timeVectorToPlot = 0.01:0.01:28; % IN SECONDS (total time period = 28 s --- sampling period = 0.01 s)

omegaA_model = lsim(sys_d1,voltage,timeVectorToPlot);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 4: omegaA experiments vs. model %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(4), hold on
sgtitle('LLS without low-pass filtering (simplified)')

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

% analyse step response of model in detail: calculate from model and
% measure from step response: peak value, peak time, steady state value
% to do that, model is first transformed back to CT and step response of CT model
% is simulated. Also zeta, wn, wd and DC-gain of CT model are calculated and used
% ALSO compare these values with values derived from the provided
% measurement

% transform model back to CT
% Use 'tustin' method because sys_d1 has a pole at z=0
sys_c1 = d2c(sys_d1, 'tustin')
pc1 = pole(sys_c1) % poles of the CT system 
wd1 = abs(imag(pc1(1))); 

[wn1,zeta1] = damp(sys_c1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 5: step response of the CT system %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(5)
step(sys_d1), grid  
xlabel('time')
ylabel('step response [-]')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 6: omegaA averaged over the periods %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(6)
sgtitle('Velocity of wheel A averaged over the periods')
plot(timeVectorToPlot(1:points_per_period),omegaA_mean) 
xlabel('time [s]')
ylabel('omegaA-mean [rad/s]')
grid

% relative peak value: value of response at peak/steady state value
%y_peak = 0.00439/0.00334;     % obtained from figure(5)
%Mp1 = y_peak-1
Mp1_model = exp(-pi*zeta1(1)/sqrt(1-zeta1(1)^2)); % formula slide 10, C6
%Mp1_measurement = 0.005081/0.003351 - 1  % obtained from figure(6)

% peak time
%tp1 = 0.312    % obtained from figure(5)
tp1_model = pi/wd1;      % formula slide 9, C6
%tp1_measurement = 8.125 - 7.813   % obtained from figure(6)

% steady state value
[Numc1, Denc1] = tfdata(sys_c1);
Numc1 = Numc1{1};
Denc1 = Denc1{1};
y_ss1_model = Numc1(end)/Denc1(end); % DC gain of CT model (evaluating transfer function at s=0) 
%y_ss1 = 0.00334 %obtained from figure(5)
%y_ss1_measurement = 0.003351 %obtained from figure(6)


%% 2.1.4 LLS with low-pass filter applied to the input and output data
% -------------------------------------------------------------------
% define a low(band)-pass filter
pd1 = pole(sys_d1);
pc1 = log(pd1)/Ts;
% For 2nd order system, use the non-zero pole (index 1 or 2)
% Find the pole with non-zero imaginary part
if abs(imag(pc1(1))) > abs(imag(pc1(2)))
    interesting_frequency = abs(imag(pc1(1))/(2*pi));
else
    interesting_frequency = abs(imag(pc1(2))/(2*pi));
end
cutoff = 0.1*interesting_frequency;  %was 1.5*interesting_frequency
[B_filt,A_filt] = butter(6, cutoff/(fs/2));

% apply the filter to both input and output
omegaA_filt = filtfilt(B_filt, A_filt, omegaA); 
voltage_filt = filtfilt(B_filt, A_filt, voltage);

% repeat the identification
b = omegaA_filt(3:end);
A = [-omegaA_filt(2:end-1), voltage_filt(1:end-2)];

% H(z) = b1 /(z^2 + a1 z)

theta_filter = A\b
% theta_filter = [a1 b1]'

Num2 = [theta_filter(2)];
Den2 = [1, theta_filter(1)];
sys_d2 = tf(Num2, Den2, Ts);

% plot results

omegaA_model2 = lsim(sys_d2,voltage,timeVectorToPlot);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 7: omegaA experiments vs. filtered model %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(7), hold on
sgtitle('LLS with low-pass filtering')

subplot(2,1,1)
plot(timeVectorToPlot,[omegaA omegaA_model2]);
legend('empirical','estimated','Location','SouthWest')
xlabel('time [s]')
axis tight
ylabel('omegaA [rad/s]')

subplot(2,1,2),plot(timeVectorToPlot,omegaA-omegaA_model2)
legend('error')
xlabel('time [s]')
ylabel('omegaA-filtered-error [rad/s]')
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
% For 2nd order system, get the imaginary part from the complex pole
if abs(imag(pc2(1))) > abs(imag(pc2(2)))
    wd2 = abs(imag(pc2(1)));
else
    wd2 = abs(imag(pc2(2)));
end

[wn2,zeta2] = damp(sys_c2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FIGURE 8: step response of the CT system (filtered model) %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(8)
step(sys_d2);grid




% relative peak value: value of response at peak/steady state value
%y_peak = 0.00508/0.00333; %obtained from figure(8)
%Mp = y_peak-1
Mp_model = exp(-pi*zeta2(1)/sqrt(1-zeta2(1)^2)); %formula slide 10, C6
%Mp_measurement = 0.005081/0.003351 - 1  %obtained from figure(6)

% peak time
%tp = 0.313 %obtained from figure(8)
tp_model = pi/wd2; %formula slide 9, C6
%tp_measurement = 8.125 - 7.813 %obtained from figure(6)

% steady state value
[Numc2, Denc2] = tfdata(sys_c2);
Numc2 = Numc2{1};
Denc2 = Denc2{1};
y_ss_model = Numc2(end)/Denc2(end); %DC gain of CT model (evaluating transfer function at s=0) 
%y_ss = 0.00333 %obtained from figure(8)
%y_ss_measurement = 0.003351 %obtained from figure(6)


