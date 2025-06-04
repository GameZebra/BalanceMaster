%% important
% you first need to save some experiment data with RealTerm in bin format
% using the func pcTransmitBin
clear
% Open the binary file
% Change the name appropriately 
fileID = fopen('PCCommunicationTest_04.bin', 'r');

% Determine the length of one sequence in bytes
sequenceLength = 4 + 4*4 + 4*2; % 4 bytes for uint32, 8 bytes for 2 floats, 3 bytes for 3 uint8s

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


for i = 1:numSequences
    startIdx = (i-1) * sequenceLength + 1;
    endIdx = startIdx + sequenceLength - 1;
    
    % Extract the current sequence
    sequence = buffer(startIdx:endIdx);
    
    tick(i) = typecast(uint8(sequence(1:4)), 'uint32');
    gyroAngle(i) = typecast(uint8(sequence(5:8)), 'single');
    accAngle(i) = typecast(uint8(sequence(9:12)), 'single');
    angle(i) = typecast(uint8(sequence(13:16)), 'single');
    setpoint(i) = typecast(uint8(sequence(17:20)), 'single');
    encoderL(i) = typecast(uint8(sequence(21:22)), 'int16');
    encoderR(i) = typecast(uint8(sequence(23:24)), 'int16');
    encoderLSpeed(i) = typecast(uint8(sequence(25:26)), 'int16');
    encoderRSpeed(i) = typecast(uint8(sequence(27:28)), 'int16');


end



