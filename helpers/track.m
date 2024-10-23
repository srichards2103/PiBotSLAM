clc;
clear;
close all;

% Define the key points for the path
x_points = [0.0, 2.0, 2.5, 3.0, 3.5, 3.0, 2.0, 1.5, 1.0, 0.5, 0.0];
y_points = [0.0, 0.0, 0.5, 1.0, 1.5, 2.0, 2.0, 1.5, 1.0, 0.5, 0.0];

% straight line from (0,0), to (2,0)
% then a quarter circle from (2,0) to (2.5, 0.5) (counter-clockwise)
% then quarter circle the other way from (2.5, 0.5) to (3, 1), (clockwise)
% then a quarter circle from (3, 1) to (3.5, 1.5), (counter-clockwise)
% then a quarter circle from (3.5, 1.5) to (3, 2), (counter-clockwise)
% then a straight linefrom (3, 2) to (2, 2),
% then a quarter circle from (2, 2) to (1.5, 1.5), (clockwise)
% then a quarter circle the other way from (1.5, 1.5) to (1, 1), (clockwise)
% then a quarter circle from (1, 1) to (0.5, 0.5), (counter-clockwise)

% Initialize arrays to store the complete path
x_path = [];
y_path = [];

% Function to generate quarter circle
function [x, y] = quarter_circle(x_center, y_center, radius, start_angle, end_angle)
    theta = linspace(start_angle, end_angle, 50);
    x = x_center + radius * cos(theta);
    y = y_center + radius * sin(theta);
end

% Generate the path
% Straight line from (0,0) to (2,0)
x_path = [x_path, linspace(0, 2, 100)];
y_path = [y_path, zeros(1, 100)];

% Quarter circle from (2,0) to (2.5, 0.5) (counter-clockwise)
[x, y] = quarter_circle(2, 0.5, 0.5, -pi/2, 0);
x_path = [x_path, x];
y_path = [y_path, y];   

% Quarter circle from (2.5, 0.5) to (3, 1) (clockwise)
[x, y] = quarter_circle(3, 0.5, 0.5, -pi, -3*pi/2);
x_path = [x_path, x];
y_path = [y_path, y];

% Quarter circle from (3, 1) to (3.5, 1.5) (counter-clockwise)
[x, y] = quarter_circle(3, 1.5, 0.5, -pi/2, 0);
x_path = [x_path, x];
y_path = [y_path, y];

% Quarter circle from (3.5, 1.5) to (3, 2) (counter-clockwise)
[x, y] = quarter_circle(3, 1.5, 0.5, 0, pi/2);
x_path = [x_path, x];
y_path = [y_path, y];

% Straight line from (3, 2) to (2, 2)
x_path = [x_path, linspace(3, 2, 100)];
y_path = [y_path, ones(1, 100) * 2];

% Quarter circle from (2, 2) to (1.5, 1.5) (clockwise)
[x, y] = quarter_circle(2, 1.5, 0.5, pi/2, pi);
x_path = [x_path, x];
y_path = [y_path, y];

% Quarter circle from (1.5, 1.5) to (1, 1) (clockwise)
[x, y] = quarter_circle(1, 1.5, 0.5, 0, -pi/2);
x_path = [x_path, x];
y_path = [y_path, y];

% Quarter circle from (1, 1) to (0.5, 0.5) (counter-clockwise)
[x, y] = quarter_circle(1, 0.5, 0.5, pi/2, pi);
x_path = [x_path, x];
y_path = [y_path, y];


% Plot the path
figure;
plot(x_path, y_path, 'b-', 'LineWidth', 2);
hold on;
plot(x_points, y_points, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
axis equal;
title('Generated Track');
xlabel('X');
ylabel('Y');
grid on;

% Save the path data
save('track_data.mat', 'x_path', 'y_path');