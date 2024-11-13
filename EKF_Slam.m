clear; clc;
x = [0; 0; 0]; P = eye(3); % State [x, y, theta] and covariance
landmark = [5; 5];         % Known landmark position
Q = 0.01 * eye(3); R = 0.1 * eye(2); % Process and measurement noise
dt = 0.1; u = [1; 0.1];    % Control inputs (linear and angular velocity)
grid_limit = 10;           % Grid limit [-10, 10]

tic; % Start timer
figure; % Create a new figure for plott`ing
hold on;
axis([-11 11 -11 11]); % Set axis limits
xlabel('X Position');
ylabel('Y Position');
title('Robot Navigation with EKF-SLAM');
grid on;

while toc < 30  % Loop for 30 seconds
    % Predict step with boundary checks
    x_new = x + [u(1)*cos(x(3))*dt; u(1)*sin(x(3))*dt; u(2)*dt];
    x_new(1:2) = max(min(x_new(1:2), grid_limit), -grid_limit); % Keep inside grid
    x = x_new; 
    P = P + Q; % Update covariance (process noise)

    % Simulate measurement (range and bearing to the landmark)
    dx = landmark(1) - x(1); 
    dy = landmark(2) - x(2);
    z = [sqrt(dx^2 + dy^2); atan2(dy, dx)] + R * randn(2, 1);

    % Update step
    H = [-dx/sqrt(dx^2+dy^2), -dy/sqrt(dx^2+dy^2), 0; 
          dy/(dx^2+dy^2), -dx/(dx^2+dy^2), -1];
    K = P * H' / (H * P * H' + R); % Kalman gain
    x = x + K * (z - [sqrt(dx^2+dy^2); atan2(dy, dx)]); 
    P = (eye(3) - K * H) * P; 

    % Plot robot and landmark
    plot(x(1), x(2), 'bo', 'MarkerFaceColor', 'b'); % Robot
    plot(landmark(1), landmark(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Landmark
    pause(0.1); 
end