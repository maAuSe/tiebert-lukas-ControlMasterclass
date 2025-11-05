%***************************************************************************
%
% Feedback exercise 1.
% 
% ***************************************************************************

clear all;
close all;

Ra = 1;
Km = 1;
J  = 2;
c  = 0.5;
Kb = 1;
Ka = 54;
Kt = 1;

% Mechanical system
tfmech = tf(1, [J c]);

% Electrical system
tfel = 1/Ra;

% Transformations
tfeltomech = Km;
tfmechtoel = Kb;

% 1.1
omegaTd = -feedback(tfmech, tfmechtoel*tfel*tfeltomech)
figure(1)
bode(omegaTd)

% 1.2
figure(2)
step(omegaTd);
title('Step response to a disturbance torque (without tacho/velocity feedback)');

% redefine the plant
Forward_chain = feedback(tfmech, tfmechtoel*tfel*tfeltomech)

% bode diagram
omegaTdtacho1 = -feedback(Forward_chain, Ka*Kt)
figure(3)
bode(omegaTdtacho1); 

% Double the feedback constant Ka -> 2*kA
omegaTdtacho2 = -feedback(Forward_chain, 2*Ka*Kt)

% bode diagram
figure(3); hold on;
bode(omegaTdtacho2);

% step response
figure(4)
step(omegaTdtacho1, omegaTdtacho2);
title('Response to a step disturbance torque (with tacho feedback)');
legend('K_a = 54', 'K_a = 2*54');



%% compute the transfer function using the PI
kappa = 2;
PIfeedback = Ka*Kt*tf([1,kappa],[1,0]);
omegaTdPI1 = -feedback(Forward_chain, PIfeedback)

% bode diagram
figure(5)
bode(omegaTdPI1)

% change kappa value
kappa = 20;
PIfeedback = Ka*Kt*tf([1,kappa],[1,0]);
omegaTdPI2 = -feedback(Forward_chain, PIfeedback)

% bode diagram
figure(5); hold on;
bode(omegaTdPI2)

% step response
figure(6)
step(omegaTdPI1,omegaTdPI2);
title('Response to a step disturbance torque (with PI)');
legend('kappa = 2', 'kappa = 20');


% ramp response
figure(7)
hold on
t = linspace(0,3.5,100); %time vector
omega1=lsim(omegaTdPI1,t,t);
omega2=lsim(omegaTdPI2,t,t);
plot(t,omega1)
plot(t,omega2)
title('Response to a ramp disturbance torque (with PI)');
legend('kappa = 2', 'kappa = 20');


% error-reference transfer function
kappa = 2;
trackingerror1 = 1 - feedback(Ka*tf([1,kappa],[1,0])*Forward_chain, Kt)

% bode diagram
figure(8)
bode(trackingerror1)

% change kappa value
kappa = 20;
trackingerror2 = 1-feedback(Ka*tf([1,kappa],[1,0])*Forward_chain, Kt)

% bode diagram
figure(8); hold on; 
bode(trackingerror2)

figure(9)
step(trackingerror1,trackingerror2);
title('Tracking error evolution for a step reference (with PI)');
legend('kappa = 2', 'kappa = 20');

figure(10)
pzmap(trackingerror1, trackingerror2)

figure(11)
hold on
e1=lsim(trackingerror1,t,t);
e2=lsim(trackingerror2,t,t);
plot(t,e1)
plot(t,e2)
title('Tracking error evolution for a ramp reference (with PI)');
legend('kappa = 2', 'kappa = 20');
