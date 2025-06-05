%% important
% you first need to save some experiment data with RealTerm in bin format
% using the func pcTransmitBin
clear
% Open the binary file
% Change the name appropriately 
fileID = fopen('PCCommunicationTest_05_derivative.bin', 'r');

% Determine the length of one sequence in bytes
sequenceLength = 4 + 4*4 + 4*2 + 4 +2*1 + 4*4 %+ 2*4; 

% Read the entire file into a buffer
buffer = fread(fileID, Inf, 'uint8');

% Close the file
fclose(fileID);

% Initialize arrays to store the data
numSequences = floor(length(buffer) / sequenceLength);
tick = zeros(numSequences, 1);
gyroAngle = zeros(numSequences, 1);
accAngle = zeros(numSequences, 1);
angle = zeros(numSequences, 1);
setpoint = zeros(numSequences, 1);
encoderL = zeros(numSequences, 1);
encoderR = zeros(numSequences, 1);
encoderLSpeed = zeros(numSequences, 1);
encoderRSpeed = zeros(numSequences, 1);

error = zeros(numSequences, 1);
zero = zeros(numSequences, 1);
speed = zeros(numSequences, 1);
Kp = zeros(numSequences, 1);
Ki = zeros(numSequences, 1);
Kd = zeros(numSequences, 1);
derivative = zeros(numSequences, 1);

accX = zeros(numSequences, 1);
accZ = zeros(numSequences, 1);


for i = 1:numSequences
    startIdx = (i-1) * sequenceLength + 1;
    endIdx = startIdx + sequenceLength - 1;
    
    % Extract the current sequence
    sequence = buffer(startIdx:endIdx);
    
    tick(i) = typecast(uint8((sequence(1:4))), 'uint32');
    gyroAngle(i) = typecast(uint8((sequence(5:8))), 'single');
    accAngle(i) = typecast(uint8((sequence(9:12))), 'single');
    angle(i) = typecast(uint8((sequence(13:16))), 'single');
    setpoint(i) = typecast(uint8((sequence(17:20))), 'single');
    encoderL(i) = typecast(uint8((sequence(21:22))), 'int16');
    encoderR(i) = typecast(uint8((sequence(23:24))), 'int16');
    encoderLSpeed(i) = typecast(uint8((sequence(25:26))), 'int16');
    encoderRSpeed(i) = typecast(uint8((sequence(27:28))), 'int16');

    error(i) = typecast(uint8((sequence(29:32))), 'single');
    zero(i) = typecast(uint8((sequence(33))), 'uint8');
    speed(i) = typecast(uint8((sequence(34))), 'uint8');
    Kp(i) = typecast(uint8((sequence(35:38))), 'single');
    Ki(i) = typecast(uint8((sequence(39:42))), 'single');
    Kd(i) = typecast(uint8((sequence(43:46))), 'single');
    derivative(i) = typecast(uint8((sequence(47:50))), 'single');

   % accX(i) = typecast(uint8((sequence(51:54))), 'single');
   % accZ(i) = typecast(uint8((sequence(55:58))), 'single');
end

%% plot some data
Td = 0.005;
Time=0:Td:(numSequences-1)*Td;

figure(1);
plot(Time, angle'), hold on, grid on;
title('Angle over time');
plot(Time, accAngle');

figure(2);
plot(Time, setpoint'), grid on;
title('Setpoint');

figure(3);
plot(Time, encoderL'), grid on, hold on;
plot(Time, encoderR');
title('distance traveled by the wheels');


figure(4);
plot(Time, derivative'), grid on;

%% data filtration


