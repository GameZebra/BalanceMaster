% Plant params 
params = struct();

% Physical constants
params.g = 9.81;       % gravity (m/s^2)

% Robot body parameters
params.m = 3.50;        % mass of the robot body (kg)
params.l = 13;        % length to center of mass from wheel axle (m)

% Wheel parameters (can be considered part of base)
params.M = 0.5;        % mass of wheels and axle (kg)
params.r = 3;       % radius of the wheel (m)

% Control input (can be overridden by a controller)
params.F = 0;          % horizontal force applied by wheels (N)

% Simulation visualization
%params.bodyWidth = 0.05;   % width of the robot body (for drawing)
%params.wheelWidth = 0.3;   % width of the wheel base (for drawing)
%params.wheelHeight = 0.05; % height of the wheels (for drawing)


% Initial conditions: [theta, dtheta, x, dx]
y0 = [5*pi/180; 0; 0; 0];  % 5 degrees tilt
tspan = [0 10];


% Anonymous function to pass parameters
dynamics = @(t, y) twoWheeledRobot(t, y, params);

[t, y] = ode45(dynamics, tspan, y0);

% Plot
figure;
subplot(2,1,1);
plot(t, y(:,1) * 180/pi);  % theta in degrees
ylabel('Tilt Angle (deg)');
grid on;

subplot(2,1,2);
plot(t, y(:,3));  % x position
xlabel('Time (s)');
ylabel('Position (m)');
grid on;

%t = 0:0.1:5;
for i = 1:length(t)
    x = y(i, 3);
    %x = 0:0.1:5;
    theta = y(i, 1);
    %theta = 0;
    plotTwoWheeledRobot(x, theta, params.l);
    %l = 12;
    %plotTwoWheeledRobot(x(i), theta, l);
    %pause(0.01);
end
