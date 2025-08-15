clear

% this data is after one filtration ...
data = readmatrix("acc_data.csv");
Td = 0.01;
time = 0:Td:(length(data)-1)*Td;
angle = data(:,2);

figure(1);
plot(time, angle),grid on, hold on;


%% moving average

windowSize = 13;
b = (1/windowSize)*ones(1,windowSize);
a = 1;
filtered_signal = filter(b, a, angle);

plot(time, filtered_signal, 'r', 'LineWidth', 2);



%% low pass filter (best so far)
% iir butter filter with matlab functions 

fs = 100; % Sampling frequency
fc = 10;   % Cut-off frequency
[b, a] = butter(2, fc/(fs/2)); % 2nd-order filter
low_passed = filter(b, a, angle);
plot(time, low_passed, 'g-', 'LineWidth', 1);
legend('Noisy', 'Moving Avg', 'low pass filter');

