clear

% I want to have controll response time 0.010 s
% this will require controll bandwith 0.35/0.01 = 35 Hz
% and the filter cutoff must be 3-10 times this frequency
% fc = 175 Hz


data = readmatrix("bare_encoder_stat_1.csv");
Td = 0.002;
time = 0:Td:(length(data)-1)*Td;
encoderRSpeed = data(:,2);
speed = data(:,3);

figure(1);
plot(time, speed, 'k'), grid on, hold on;
plot(time, encoderRSpeed, 'b-');
ylim([-500 1200]), xlim([0 140]);

%% moving average (10th order)
movingAverage10Speed = zeros(1, length(data));
filterOrder = 10;
sum = 0;
encoderValues = zeros(1,filterOrder);
for i = 1:length(data)
    sum = 0;
    for j = filterOrder:-1:2
        encoderValues(j) = encoderValues(j-1);
        sum = sum + encoderValues(j);
    end
    encoderValues(1) = encoderRSpeed(i);
    sum = sum + encoderValues(1);
    movingAverage10Speed(i) = sum / filterOrder;
end

plot(time, movingAverage10Speed, 'g-');

%% moving average (100th order)
movingAverage100Speed = zeros(1, length(data));
filterOrder = 100;
sum = 0;
encoderValues = zeros(1,filterOrder);
for i = 1:length(data)
    sum = 0;
    for j = filterOrder:-1:2
        encoderValues(j) = encoderValues(j-1);
        sum = sum + encoderValues(j);
    end
    encoderValues(1) = encoderRSpeed(i);
    sum = sum + encoderValues(1);
    movingAverage100Speed(i) = sum / filterOrder;
end

plot(time, movingAverage100Speed, 'g--');


%% FIR Low-Pass Filter Design (cutoff 10Hz)
Fs = 1/0.002;          % Sampling frequency (Hz)
Fc = 10;           % Cutoff frequency (Hz)
N  = 10;            % Filter order (number of taps - 1)

Wn = Fc / (Fs/2);   % Normalized cutoff (0–1), 1 = Nyquist frequency
b  = fir1(N, Wn, 'low');  % Low-pass filter coefficients

% Plot frequency response
%fvtool(b, 1)

figure(1)
firSpeed1 = filter(b, 1, encoderRSpeed); % Filter signal x
plot(time, firSpeed1, 'r--')

%% FIR Low-Pass Filter Design (cutoff 2Hz)
Fs = 1/0.002;          % Sampling frequency (Hz)
Fc = 120;           % Cutoff frequency (Hz)
N  = 10;            % Filter order (number of taps - 1)

Wn = Fc / (Fs/2);   % Normalized cutoff (0–1), 1 = Nyquist frequency
b  = fir1(N, Wn, 'low');  % Low-pass filter coefficients

% Plot frequency response
%fvtool(b, 1)

figure(1)
firSpeed2 = filter(b, 1, encoderRSpeed); % Filter signal x
plot(time, firSpeed2, 'r-', 'LineWidth', 2)

%% FIR Low-Pass Filter Design (cutoff 175Hz, order 4)
Fs = 1/0.002;          % Sampling frequency (Hz)
Fc = 175;           % Cutoff frequency (Hz)
N  = 4;            % Filter order (number of taps - 1)

Wn = Fc / (Fs/2);   % Normalized cutoff (0–1), 1 = Nyquist frequency
b  = fir1(N, Wn, 'low');  % Low-pass filter coefficients

% Plot frequency response
%fvtool(b, 1)

figure(1)
firSpeed3 = filter(b, 1, encoderRSpeed); % Filter signal x
plot(time, firSpeed3, 'r--', 'LineWidth', 1)

%becomes worse

%% IIR Butterworth Low-Pass Filter
Fs = 1/0.002;;          % Sampling frequency (Hz)
Fc = 175;           % Cutoff frequency (Hz)
N  = 3;             % Filter order

Wn = Fc / (Fs/2);   % Normalized cutoff frequency
[b, a] = butter(N, Wn, 'low');  % Low-pass design

% View frequency response
%fvtool(b, a)

% Apply filter to signal x
butterworthSpeed1 = filter(b, a, encoderRSpeed);
plot(time, butterworthSpeed1, 'c--', 'LineWidth', 1)

%% IIR Butterworth Low-Pass Filter
Fs = 1/0.002;;          % Sampling frequency (Hz)
Fc = 133;           % Cutoff frequency (Hz)
N  = 3;             % Filter order

Wn = Fc / (Fs/2);   % Normalized cutoff frequency
[b, a] = butter(N, Wn, 'low');  % Low-pass design

% View frequency response
%fvtool(b, a)

% Apply filter to signal x
figure(1)
butterworthSpeed2 = filter(b, a, encoderRSpeed);
plot(time, butterworthSpeed2, 'c-', 'LineWidth', 2)

figure(3)
zplane(b, a);                           % plot zeros and poles
grid on;
title('Poles and Zeros of the Filter');

%[sos,g] = tf2sos(b,a)

%% moving average (5th order) on top of fir filter
movingAvgFir = zeros(1, length(data));
filterOrder = 6;
sum = 0;
encoderValues = zeros(1,filterOrder);
for i = 1:length(data)
    sum = 0;
    for j = filterOrder:-1:2
        encoderValues(j) = encoderValues(j-1);
        sum = sum + encoderValues(j);
    end
    encoderValues(1) = butterworthSpeed2(i);
    sum = sum + encoderValues(1);
    movingAvgFir(i) = sum / filterOrder;
end
figure(1)
plot(time, movingAvgFir, 'g-', 'LineWidth', 3);

%% IIR Cheby1 Low-Pass Filter
Fs = 1/0.002;       % Sampling frequency (Hz)
Fc = 175;            % Cutoff frequency (Hz)
N  = 3;             % Filter order
Rp = 0.001;             % Passband ripple in dB
Wn = Fc / (Fs/2);   % Normalized cutoff frequency
[b, a] = cheby1(N, Rp, Wn, 'low');  % Low-pass design

% View frequency response
%fvtool(b, a)

% Apply filter to signal x
figure(1)
chebby11 = filter(b, a, encoderRSpeed);
plot(time, chebby11, 'm--', 'LineWidth', 1)

figure(3)
zplane(b, a);                           % plot zeros and poles
grid on;
title('Poles and Zeros of the Filter');

[sos,g] = tf2sos(b,a)

% Frequency response
freqz(b, a, 1024);
title('4th-order Chebyshev Type I Low-pass Filter');
