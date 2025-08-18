clear

% this data is after one filtration ...

data = readmatrix("acc_velocity_stationary_1.csv");             % DFT here
data1 = readmatrix("acc_velocity_normal_work_no_motors.csv");   % test for
data2 = readmatrix("acc_velocity_normal_work_motors_on.csv");   % filters

Td = 0.01;
time = 0:Td:(length(data)-1)*Td;
angle = data(:,1);
velocity = data(:,2)

figure(1);
plot(time, data),grid on, hold on;

figure(2);
time1 = 0:Td:(length(data1)-1)*Td;
plot(data1),grid on, hold on;
figure(3);
time2 = 0:Td:(length(data2)-1)*Td;
plot(data2),grid on, hold on;
%% moving average

windowSize = 13;
b = (1/windowSize)*ones(1,windowSize);
a = 1;
filtered_signal = filter(b, a, angle);

figure(10)
plot(time, angle, 'b-'), grid on, hold on;
plot(time, filtered_signal, 'r', 'LineWidth', 2);



%% low pass filter (best so far)
% iir butter filter with matlab functions 

fs = 100; % Sampling frequency
fc = 10;   % Cut-off frequency
[b, a] = butter(2, fc/(fs/2)); % 2nd-order filter
low_passed = filter(b, a, angle);
plot(time, low_passed, 'g-', 'LineWidth', 1);
legend('Noisy', 'Moving Avg', 'low pass filter');

