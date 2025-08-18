%% Kalman Filter for Balancing Robot (3-state: angle, angular velocity, bias)
clear
data0 = readmatrix("acc_velocity_stationary_1.csv");
a0 = data0(:,1);
g0 = data0(:,2);


data = readmatrix("acc_velocity_normal_work_motors_on.csv");
%data = readmatrix("acc_velocity_normal_work_no_motors.csv");

dt = 0.01;

t = 0:dt:(length(data)-1)*dt;
accel_angle = data(:,1);
gyro = data(:,2);



% States: x = [theta; omega; bias]
x = [0; 0; 0];         % initial guess
P = eye(3);            % initial covariance

% State transition matrix
A = [1 dt 0;
     0 1  0;
     0 0  1];

% Process noise covariance (tune these)
q_theta = 0.15;
q_omega = 0.23;
q_bias  = 1e-6;
Q = diag([q_theta q_omega q_bias]);

% Measurement matrices
H_acc = [1 0 0];   % accelerometer measures angle
H_gyro = [0 1 1];  % gyro measures omega + bias

% Measurement noise (tune these)
R_acc  = 5;     % accelerometer angle noise variance
R_gyro = 1.44;     % gyro noise variance

%R_acc  = var(a0 - mean(a0));     % accelerometer angle noise variance
%R_gyro = var(g0 - mean(g0));     % gyro noise variance

% Storage for results
x_est = zeros(3, length(t));

%% Main loop
for k = 1:length(t)
    % 1. Prediction
    x = A * x;             % no control input
    P = A * P * A' + Q;
    
    % 2a. Update with gyro measurement
    z_gyro = gyro(k);
    y = z_gyro - H_gyro * x;
    S = H_gyro * P * H_gyro' + R_gyro;
    K = P * H_gyro' / S;
    x = x + K * y;
    P = (eye(3) - K * H_gyro) * P;
    
    % 2b. Update with accelerometer angle
    z_acc = accel_angle(k);
    y = z_acc - H_acc * x;
    S = H_acc * P * H_acc' + R_acc;
    K = P * H_acc' / S;
    x = x + K * y;
    P = (eye(3) - K * H_acc) * P;
    
    % Save estimate
    x_est(:,k) = x;
end

%% Plot results
figure;
subplot(3,1,1);
plot(t, accel_angle, 'g', t, x_est(1,:), 'r');
legend('Accel angle', 'KF angle');
ylabel('\theta (rad)');

subplot(3,1,2);
plot(t, gyro, 'b', t, x_est(2,:), 'r');
legend('Raw gyro', 'KF angular velocity');
ylabel('\omega (rad/s)');

subplot(3,1,3);
plot(t, x_est(3,:), 'k');
ylabel('Bias (rad/s)');
xlabel('Time (s)');
legend('Estimated bias');
