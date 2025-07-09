cd clear all;
close all;

%% Test signal
t = 0:0.01:10;
true_signal = sin(4*t)+2;
noise = 0.2 * randn(size(t)) + 0.05*sin(400*t) ;

noisy_signal = true_signal + noise;

plot(t, noisy_signal);
hold on;
plot(t, true_signal, 'k', 'LineWidth', 2), grid on, hold on;
legend('Noisy', 'True');

%% average filter  
average = sum(noisy_signal)/ length(t);
plot(t, average * ones(1, length(t)), 'r');

%% moving average

windowSize = 10;
b = (1/windowSize)*ones(1,windowSize);
a = 1;
filtered_signal = filter(b, a, noisy_signal);

plot(t, filtered_signal, 'y', 'LineWidth', 2);
legend('Noisy', 'True', 'Moving Avg');


%% low pass filter (best so far)
% iir butter filter with matlab functions 

fs = 100; % Sampling frequency
fc = 6;   % Cut-off frequency
[b, a] = butter(2, fc/(fs/2)); % 2nd-order filter
low_passed = filter(b, a, noisy_signal);
plot(t, low_passed, 'g--', 'LineWidth', 2);


%% exponential filter 

% Exponentially decaying IIR filter demo

% Parameters
alpha = 0.9;            % Decay factor (0 < alpha < 1)
N = length(t);                % Number of samples
x = noisy_signal;        % Input signal
x(10) = 1;              % Impulse at n=10

% Pre-allocate output
y = zeros(1, N);

% IIR filter: y[n] = (1 - alpha)*x[n] + alpha*y[n-1]
for n = 2:N
    y(n) = (1 - alpha) * x(n) + alpha * y(n-1);
end

% Plot
figure(2);
plot(t, y);
title('Exponentially Decaying IIR Filter Response');
xlabel('t');
ylabel('y[t]');
grid on;
hold on;
%plot(t, true_signal);

%% Kalman Filter


