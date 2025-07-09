t = 0:0.1:5;
for i = 1:length(t)
    %x = y(i, 3);
    x = 0:0.1:5;
    %theta = y(i, 1);
    theta = 0;
    %plotTwoWheeledRobot(x, theta, params.l);
    l = 12;
    plotTwoWheeledRobot(x(i), theta, l);
    %pause(0.01);
end
