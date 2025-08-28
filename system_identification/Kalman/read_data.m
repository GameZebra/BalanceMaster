clear
% Open the binary file
fid = fopen('raw4.bin', 'rb');

% Read all floats in the file
rawData = fread(fid, 'float32');

% Close file
fclose(fid);

% Separate into two variables
accAngle    = rawData(1:2:end); % odd indices
kalmanAngle = rawData(2:2:end); % even indices

% Example check
disp([accAngle(1:5), kalmanAngle(1:5)]);

plot(accAngle), hold on
plot(kalmanAngle)