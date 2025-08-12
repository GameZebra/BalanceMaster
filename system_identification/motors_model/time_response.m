clear

rMotor = readmatrix("time_response.csv");
Td = 0.002;
time = 0:Td:(length(rMotor)-1)*Td;
Rspeed = rMotor(:,2);
Lspeed = rMotor(:,3);
Rcontrol = rMotor(:,4); % failed to send the data
Lcontrol = rMotor(:,5);

T = 2930;
T_half = T/2;

% step up
figure(1), subplot(211);

speeds=zeros(2, T_half);
for i = 732:T:9*T
    for j = 1:T_half
        speeds(1, j) = speeds(1,j) + Rspeed(i+j);
        speeds(2, j) = speeds(2,j) - Lspeed(i+j);
    end
end
speeds = speeds ./ 9;
time = 0:Td:(length(speeds(1,:))-1)*Td;
plot(time, speeds), grid on; 
legend('right wheel', 'left wheel');
title('time response');
subplot(212);
plot(time, Lcontrol(732:length(time)-1+732)), grid on; % Ton 1.466 s
title('input');

%% right motor identification
% --- measured data ---
t   = time'; % [s]
u   = Lcontrol(732:length(time)-1+732); % [0-127]
y   = speeds(1,:)'; % [mm/s]

% Create iddata object (for System Identification Toolbox)
Ts = Td; % sample time
rMotor = iddata(y, u, Ts);

% --- First-order model estimation ---
% Continuous-time transfer function:  G(s) = K / (tau*s + 1)
% In MATLAB, that's order [nb nf nk] = [1 1 0] for tfest
model_order = [2 1 0]; % [nb nf nk]: num coeffs, den coeffs, delay
motor_model = tfest(rMotor, model_order(1), model_order(2));

% Display model
disp('Estimated motor model (continuous-time):');
motor_model

% --- Plot comparison ---
figure;
compare(rMotor, motor_model);
grid on;
title('Motor Step Response: Measured vs Model');

%% left motor identification
% --- measured data ---
t   = time'; % [s]
u   = Lcontrol(732:length(time)-1+732); % [0-127]
y   = speeds(2,:)'; % [mm/s]

% Create iddata object (for System Identification Toolbox)
Ts = Td; % sample time
lMotor = iddata(y, u, Ts);

% --- First-order model estimation ---
% Continuous-time transfer function:  G(s) = K / (tau*s + 1)
% In MATLAB, that's order [nb nf nk] = [1 1 0] for tfest
model_order = [2 1 0]; % [nb nf nk]: num coeffs, den coeffs, delay
motor_model = tfest(lMotor, model_order(1), model_order(2));

% Display model
disp('Estimated motor model (continuous-time):');
motor_model

% --- Plot comparison ---
figure;
compare(lMotor, motor_model);
grid on;
title('Motor Step Response: Measured vs Model');

%% PID tune
%sys_c = d2c(motor_model, 'zoh');
[C_pi,info] = pidtune(motor_model,'PID', 80.0)
T_pi = feedback(C_pi*motor_model, 1);
%step(T_pi, 80)

t = 0:0.005:0.5;           % time vector
u = 800 * ones(size(t)); % step input of magnitude 80
[y, t_out] = lsim(T_pi, u, t);

plot(t_out, y, 'LineWidth', 1.5)
grid on
xlabel('Time (s)')
ylabel('Output')
title('Step to 80')
