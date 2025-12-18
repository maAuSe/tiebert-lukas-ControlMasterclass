
clear all
close all 
clear global
clc

%% 2.4 Recursion formula

% Define both transfer functions:
Gs = tf(100,[1 2*0.7*10 100]);
Gc = tf([1 4],[1 0]); 

% Convert them to discrete time at a 100 Hz sampling rate:
Ts = 0.01;  % sampling time [s], Ts = 1/(100 Hz)
Gsd = c2d(Gs,Ts,'zoh');
Gcd = c2d(Gc,Ts,'zoh'); 

% The recursion formula directly follows from the coefficients of the
% discrete-time transfer functions (see also part 2.7 of the assignment of 
% the tutorial session on the mecotrons):
[nums,dens] = tfdata(Gsd); nums = nums{1}; dens = dens{1}; 
[numc,denc] = tfdata(Gcd); numc = numc{1}; denc = denc{1};

% The recursion formulas become:
% output[k] = num(1)/den(1) * input[k] + num(2)/den(1) * input[k-1] + ... + num(length(num))/den(1) * input([k-length(num)+1])
%              - den(2)/den(1) * output[k-1] - den(3)/den(1) * output[k-1] - ... - den(length(den))/den(1) * output[k-length(den)+1]
%
% ! Important: This formula assumes that 'num' and 'den' have the same
% length, which is only coincidentally true in this case. If this is not
% the case, you should pad the shorter one with zeros from the left. 

% Using the recursion formulas, we can now simulate the closed-loop
% step response:

time = 0:Ts:5;
r = ones(length(time),1);       % the reference signal = 1 for a step
e = zeros(length(time),1);      % the error signal, initialize at 0
u = zeros(length(time),1);      % the control signal, initialize at 0
y = zeros(length(time),1);      % the output signal, initialize at 0

for k=3:length(time)
    
    % simulate the system dynamics
    y(k) = nums(1)/dens(1)*u(k) + nums(2)/dens(1)*u(k-1) + nums(3)/dens(1)*u(k-2) - dens(2)/dens(1)*y(k-1) - dens(3)/dens(1)*y(k-2); 
    
    % calculate the tracking error
    e(k) = r(k) - y(k); 
    
    % simulate the controller dynamics
    u(k) = numc(1)/denc(1)*e(k) + numc(2)/denc(1)*e(k-1) - denc(2)/denc(1)*u(k-1); 
    
end

% Plot the results. 
figure; plot(time,r,'k'); hold on; plot(time,y); plot(time,u); legend('reference','output','control signal'); title('Step response');

% We can now verify the MATLAB simulation of the step response of the
% original continuous-time system, in feedback configuration, almost gives
% the same result, but not exactly. This is due to the discretization 
% error: the dynamics are discretized, so there is always an approximation 
% error (discretization causes some phase lag at higher frequencies).
CL = feedback(Gs*Gc,1);
figure; step(CL); 
hold on; plot(time,y,'r'); 
title('Output response to a step reference'); 
legend('MATLAB simulation: continuous time','recursion expression'); 
