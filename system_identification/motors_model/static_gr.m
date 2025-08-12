clear

data = readmatrix("static_graph_1.csv");
Td = 0.002;
time = 0:Td:(length(data)-1)*Td;
Rspeed = data(:,2);
Lspeed = data(:,3);
Rcontrol = data(:,4); % failed to send the data
Lcontrol = data(:,5);


figure(1), subplot(211);
plot(time, Rspeed), grid on, hold on;
plot(time, -Lspeed);
legend('right wheel speed', 'left wheel speed');
title('static experiment data');
subplot(212);
plot(time, Lcontrol), grid on;
legend('input');

figure(2)
% calculate the points to plot
speeds = zeros(2,8);
control = zeros(1,8);
lsum = 0;
rsum = 0;
for i = 510:1500:length(data)-1
    for j = -10:0
        rsum = rsum + Rspeed(i-j+1);
        lsum = lsum + Lspeed(i-j+1);
    end
    rsum = rsum /10;
    speeds(1, int16(i/1500+1)) = rsum;
    rsum = 0;
    lsum = lsum /10;
    speeds(2, int16(i/1500+1)) = -lsum;
    lsum = 0;
    control(int16(i/1500+1)) = Lcontrol(i);
end
plot(control, speeds), grid on;
legend('right wheel speed', 'left wheel speed');
title('static haracteristic');



