function plotTwoWheeledRobot(x, theta, l)
    % Inputs:
    %   x     - horizontal position of the wheel base (m)
    %   theta - tilt angle from vertical (rad), positive CCW
    %   l     - length from wheel base to center of mass (m)
    %   alpha  - wheel angle

    clf; hold on; axis equal;
    grid on;

    % Set axis limits (adjust as needed)
    xlim([-10, 10] + x);
    ylim([-0.2, 20]);

    % Draw ground
    plot([-50 50], [0 0], 'k', 'LineWidth', 2);

    % Robot wheels 
    wheel_width = 3;
    wheel_y = 3;
    rectangle('Position', [x - wheel_width/2, 0, wheel_width, wheel_y], ...
              'EdgeColor', 'k', 'LineWidth', 2, 'Curvature', [1 1], ...
              'Tag', 'wheel');
    
    % wheel stripes
    alpha = -x/(wheel_width/2)
    x_stripes(1) = x - wheel_y/2 * sin(alpha);
    x_stripes(2) = x + wheel_y/2 * sin(alpha);
    y_spike(1) = wheel_y/2 + wheel_y/2 * cos(alpha);
    y_spike(2) = wheel_y/2 - wheel_y/2 * cos(alpha);
    plot(x_stripes, y_spike, 'k-', 'LineWidth', 2);
    
    x_stripes(1) = x - wheel_y/2 * sin(alpha+pi/2);
    x_stripes(2) = x + wheel_y/2 * sin(alpha+pi/2);
    y_spike(1) = wheel_y/2 + wheel_y/2 * cos(alpha+pi/2);
    y_spike(2) = wheel_y/2 - wheel_y/2 * cos(alpha+pi/2);
    plot(x_stripes, y_spike, 'k-', 'LineWidth', 2);

    % Robot body (rod or rectangle)
    body_length = l;   % distance to center of mass or height of body
    body_width = 0.05;

    % Body top position based on angle
    x_top = x - body_length * sin(theta);
    y_top = wheel_y + body_length * cos(theta);

    % Draw body as a line
    plot([x, x_top], [wheel_y/2, y_top], 'r-', 'LineWidth', 4, 'Tag','body');

    % set body to back
    %robot_body = findobj('Tag', 'body');
    %uistack(robot_body, 'bottom'); 

    % Optional: draw center of mass
    plot(x_top, y_top, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % Labels
    title(sprintf('x = %.2f m, \\theta = %.1f^\\circ', x, theta * 180/pi));
    xlabel('Horizontal Position (m)');
    ylabel('Height (m)');
    
    drawnow;
end
