clear

data = readmatrix("no_load time_response.csv");
Td = 0.002;
time = 0:Td:(length(data)-1)*Td;
Rspeed = data(:,2);
Lspeed = data(:,3);
Rcontrol = data(:,4); % failed to send the data
Lcontrol = data(:,5);

% the data was collected with faster interupt than acctually achievable 
equlizeConstant = 0.0001/0.001816;
Rspeed = Rspeed * equlizeConstant;
Lspeed = Lspeed * equlizeConstant;
%Rcontrol = Rcontrol * equlizeConstant;
%Lcontrol = Lcontrol * equlizeConstant;

figure(1), subplot(211);
plot(time, Rspeed), grid on, hold on;
plot(time, -Lspeed);
legend('right wheel speed', 'left wheel speed');
title('static experiment data');
subplot(212);
plot(time, Lcontrol), grid on;
legend('input');





%% faster PID
data = readmatrix("no_load time_response_1.csv");
Td = 0.002;
time = 0:Td:(length(data)-1)*Td;
Rspeed = data(:,2);
Lspeed = data(:,3);
Rcontrol = data(:,4); % failed to send the data
Lcontrol = data(:,5);




figure(2), subplot(211);
plot(time, Rspeed), grid on, hold on;
plot(time, -Lspeed);
legend('right wheel speed', 'left wheel speed');
title('static experiment data');
subplot(212);
plot(time, Lcontrol), grid on;
legend('input');




