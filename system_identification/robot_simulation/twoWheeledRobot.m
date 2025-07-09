function dydt = twoWheeledRobot(t, y, params)
    % Unpack parameters
    g = params.g;
    M = params.M;
    m = params.m;
    l = params.l;
    F = params.F;  % Can be updated with control law

    % State variables
    theta = y(1);
    dtheta = y(2);
    x = y(3);
    dx = y(4);

    % Intermediate calculations
    sin_theta = sin(theta);
    cos_theta = cos(theta);
    denom = l * (4/3 - (m * cos_theta^2) / (M + m));
    
    % Dynamics
    ddtheta = (g * sin_theta + cos_theta * ((-F - m * l * dtheta^2 * sin_theta) / (M + m))) / denom;
    ddx = (F + m * l * (dtheta^2 * sin_theta - ddtheta * cos_theta)) / (M + m);

    % Derivatives
    dydt = [dtheta; ddtheta; dx; ddx];
end
