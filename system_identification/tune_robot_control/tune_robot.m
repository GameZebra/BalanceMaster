clear
% Open the binary file
fid = fopen('raw3.bin', 'rb');

% Read all floats in the file
rawData = fread(fid, 'float32');

% Close file
fclose(fid);

num_data_sent = 10;
td = 0.01;

% Separate into two variables

targetPosition          = rawData(1:num_data_sent:end); 
currentPosition         = rawData(2:num_data_sent:end); 
targetAngle             = rawData(3:num_data_sent:end); 
KalmanAngle             = rawData(4:num_data_sent:end); 
KalmanAngularVelolocity = rawData(5:num_data_sent:end);  
AccAngle                = rawData(6:num_data_sent:end); 
gyroAngularVelolocity   = rawData(7:num_data_sent:end);
targetSpeed             = rawData(8:num_data_sent:end); 
measuredSpeed           = rawData(9:num_data_sent:end); 
motorControl            = rawData(10:num_data_sent:end);  
time = 0:td:(length(motorControl)-1)*td;

% Example check

figure(1)
plot(time, targetPosition(1:length(time))), hold on
plot(time, currentPosition(1:length(time))), grid on
legend('Target position', 'Measured position')

figure(2), subplot(211)
plot(time, targetAngle(1:length(time))), hold on
plot(time, KalmanAngle(1:length(time))), grid on
plot(time, AccAngle(1:length(time)))
legend('Target angle', 'Kalman angle', 'Acc angle')
subplot(212)
plot(time, KalmanAngularVelolocity(1:length(time))), hold on, grid on
plot(time, -gyroAngularVelolocity(1:length(time)))
legend('Kalman Angular Velocity', 'gyro angular velocity')

figure(3), subplot(211),
plot(time, targetSpeed(1:length(time))), hold on
plot(time, measuredSpeed(1:length(time))), grid on
legend('Target speed', 'Measured speed')
subplot(212)
plot(time, motorControl(1:length(time))), grid on

