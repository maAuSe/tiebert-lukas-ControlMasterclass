time = 0:0.01:32; 
signal_excit = 8 * (heaviside(mod(time, 16) - 4) - heaviside(mod(time, 16) - 8)) - 8 * (heaviside(mod(time, 16) - 12) - heaviside(mod(time, 16) - 16)); 
figure
plot(time, signal_excit);
xlabel('Time [s]');
ylabel('Voltage [V]');
title('Excitation signal');
grid on;
xlim([0 32])
ylim([-12 12]);
