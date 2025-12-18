close all
clear all
clc

T_s = 0.01;
r = 0.033;
A = 1;
B = T_s*r;

%%
Num = 1;

K = 0;
Den = [1, -A+K*B];
transfer_function = tf(Num,Den,T_s);
figure
pzmap(transfer_function)
hold on

K = 1/(2*T_s*r);
Den = [1, -A+K*B];
transfer_function = tf(Num,Den,T_s);
pzmap(transfer_function)
hold on

K = 1/(T_s*r);
Den = [1, -A+K*B];
transfer_function = tf(Num,Den,T_s);
pzmap(transfer_function)
hold on

K = 3/(2*T_s*r);
Den = [1, -A+K*B];
transfer_function = tf(Num,Den,T_s);
pzmap(transfer_function)
hold on

K = 2/(T_s*r);
Den = [1, -A+K*B];
transfer_function = tf(Num,Den,T_s);
pzmap(transfer_function)
hold on

legend('K=0','K=1515','K=3030','K=4545','K=6060')


%%
T_s = 0.01;
A = 1;
B = T_s*r;
C=-1;
D=0;

for K = 1515:1515:4545
H_num = [C*B];
H_denom = [1,-A+K*B];
H = tf(H_num,H_denom,T_s);
figure(5)
step(H)
hold on
end
legend('K=1515','K=3030','K=4545')

% % Define the range of K values
% K_values = [1515,3030,4545,6060];
% 
% % Time vector for simulation
% time_steps = 50;              % Number of steps
% t = (0:time_steps-1)' * T_s;   % Time array
% 
% % Initialize figure
% figure;
% hold on;
% 
% 
% for i = 1:6
%     k = K(i,1);
% 
%     transferf= tf(1,[1,-k*A*B])
% 
% 
%     Trans = feedback(K*transferf,1)
%     figure
%     hold on 
%     grid on 
%     step(Trans)
%     xlabel('Time (s)');
%     ylabel('Amplitude');
%     title('Closed-Loop Step Response for Varying K');
%     legend(arrayfun(@(K) sprintf('K = %.1f', K), K_values, 'UniformOutput', false));
%     hold off;
% 
% end