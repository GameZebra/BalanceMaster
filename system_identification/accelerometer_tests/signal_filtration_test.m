%% Test signal
t = 0:0.01:10;
true_signal = sin(t)+2;
noise = 0.2 * randn(size(t));

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

plot(t, filtered_signal, 'y');
legend('Noisy', 'True', 'Moving Avg');

%% low pass filter

fs = 100; % Sampling frequency
fc = 3;   % Cut-off frequency
[b, a] = butter(2, fc/(fs/2)); % 2nd-order filter
low_passed = filter(b, a, noisy_signal);
plot(t, low_passed, 'g');
