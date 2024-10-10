% Add the simulator to the MATLAB path
addpath("../simulator/");
addpath('arucoDetector');
addpath('arucoDetector/include');
addpath('arucoDetector/dictionary');

% Import data for landmark detection
load('calibrationSession.mat');
load('arucoDetector/dictionary/arucoDict.mat');
marker_length = 0.073;
cameraParams = calibrationSession.CameraParameters;


% Initialize the PiBot Simulator with the floor image
% pb = piBotSim("floor_spiral.jpg");

% % Place the robot at the starting position with a specific orientation
% initialPosition = [2.5; 2.5];
% initialOrientation = 0.64; % Radians
% pb.place(initialPosition, initialOrientation);

% Uncomment the following line if using an actual PiBot
pb = PiBot('192.168.50.1');
%
% Create a figure window with two subplots for camera view and robot path
figure('Name', 'Robot Camera and State');
camAxes = subplot(1,2,1);
title(camAxes, 'Robot Camera');

stateAxes = subplot(1,2,2);
title(stateAxes, 'Robot Path');
xlabel(stateAxes, 'X Position');
ylabel(stateAxes, 'Y Position');
hold(stateAxes, 'on');

% Initialize variables
end_of_line = false;
state = [0; 0; 0]; % [X; Y; Theta]
past_time = datetime('now');

% Initialize state history array for plotting
state_history = state';
real_u = 0; % Real linear velocity
real_q = 0; % Real angular velocity

% Initialize list to store detected Aruco marker IDs
detectedArucoIDs = [];

% Initialize set to keep track of plotted markers to avoid duplicates
plottedMarkers = containers.Map('KeyType', 'double', 'ValueType', 'logical');

% This map will store the global positions of each detected marker
markerPositions = containers.Map('KeyType', 'double', 'ValueType', 'any');


% Stopping criteria parameters
blackPixelThreshold = 90;        % Minimum number of black pixels to consider the line as present
consecutiveThreshold = 5;        % Number of consecutive frames below the threshold to confirm end of line
belowThresholdCount = 0;         % Counter for consecutive frames below the threshold


% Main loop to follow the line
while ~end_of_line
    tic;
    % Get the current camera frame
    img = pb.getImage();

    % Detect landmarks using Aruco markers
    [marker_nums, landmark_centres, marker_corners] = detectArucoPoses(img, marker_length, cameraParams, arucoDict);

    if ~isempty(marker_nums)
        % Append detected marker numbers to the list
        detectedArucoIDs = [detectedArucoIDs, marker_nums'];
        fprintf("Marker %d detected\n", marker_nums)

        % Iterate through each detected marker
        for i = 1:length(marker_nums)
            marker_id = marker_nums(i);
            if marker_id > 30
                continue;
            end

            % Check if the marker has already been plotted
            if ~isKey(plottedMarkers, marker_id)
                % Get the landmark center in robot frame
                landmark_robot = landmark_centres(i, :)'; % [X; Y; Z] in robot frame

                % Transform landmark position to global frame based on robot's state
                theta = state(3);
                rotation_matrix = [cos(theta) -sin(theta) 0;
                    sin(theta)  cos(theta) 0;
                    0           0          1];
                landmark_global = state(1:2) + rotation_matrix(1:2,1:2) * landmark_robot(1:2);

                % Plot the marker on the stateAxes
                plot(stateAxes, landmark_global(1), landmark_global(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
                text(stateAxes, landmark_global(1)+0.05, landmark_global(2)+0.05, num2str(marker_id), 'Color', 'r', 'FontSize', 12);

                % Mark the marker as plotted
                plottedMarkers(marker_id) = true;

                % =======================
                % 2. New: Store Marker Global Position
                % =======================
                markerPositions(marker_id) = landmark_global; % Store the position for final plotting
                % end
            end
        end
    end
    % Binarize the image with a threshold
    gray_img = rgb2gray(img);
    bin_img = ~imbinarize(gray_img, 0.4);

    % Crop the binary image to the bottom third for line detection
    [height, width] = size(bin_img);
    bottom_third_bin_img = bin_img(round(2*height/3):end, :);

    % Display the cropped binary image in the camera subplot
    imshow(bottom_third_bin_img, 'Parent', camAxes);

    % Count the number of black pixels in the cropped image
    blackPixelCount = sum(bottom_third_bin_img(:));

    % Update the consecutive frame counter based on blackPixelCount
    if blackPixelCount < blackPixelThreshold
        belowThresholdCount = belowThresholdCount + 1;
    else
        belowThresholdCount = 0; % Reset if the count is above threshold
    end

    % Check if the robot has reached the end of the line
    if belowThresholdCount >= consecutiveThreshold
        end_of_line = true;
        break;
    end

    % Find the center of mass of the line pixels in the cropped image
    [rows, cols] = find(bottom_third_bin_img == 1);

    % Ensure that some pixels are detected to avoid NaN values
    if isempty(cols) || isempty(rows)
        centerOfMass = [0, 0];
    else
        centerOfMass = [mean(cols), mean(rows)];
    end

    % Normalize the center of mass relative to the image dimensions
    centerOfMass = (centerOfMass - [width/2, height/6]) ./ [width/2, height/6];

    % Proportional control for angular velocity based on the center of mass

    % angular_velocity = sign(centerOfMass(1)) * -1.2 * (centerOfMass(1))^2;
    angular_velocity = -0.8  * centerOfMass(1);

    % Determine linear velocity based on the position of the line
    if abs(centerOfMass(1)) > 0.2
        linear_velocity = 0.15;
    else
        linear_velocity = 0.25;
    end

    % Calculate wheel velocities using inverse kinematics
    [wl, wr] = inverse_kinematics(linear_velocity, angular_velocity);

    % Helper function to round wheel velocities to the nearest lower multiple of 5
    round_to_lower_5 = @(value) 5 * floor(value / 5);
    wl = round_to_lower_5(wl);
    wr = round_to_lower_5(wr);

    % Calculate the actual velocities after rounding
    [real_u, real_q] = forward_kinematics(wl, wr);

    % Update the robot's state based on kinematics
    current_time = datetime('now');
    dt = seconds(current_time - past_time);
    state = integrate_kinematics(state, dt, real_u, real_q);
    past_time = current_time;

    % Append the current state to the history for plotting
    state_history = [state_history; state'];

    % Set the robot's wheel velocities
    pb.setVelocity(wl, wr);

    % Forward kinematics to confirm real velocities
    [real_u, real_q] = forward_kinematics(wl, wr);

    % Plot the robot's path in the state subplot
    plot(stateAxes, state_history(:,1), state_history(:,2), 'b-');
    drawnow;

    % disp(['Theta: ', num2str(state(3))]);
    toc
end


% Stop the robot by setting wheel velocities to zero
pb.stop();
disp('Robot has stopped at the end of the line.');

% Display all detected unique Aruco marker IDs if detection was enabled
% reject any detectedArucoIDs >= 50

detectedArucoIDs = detectedArucoIDs(detectedArucoIDs < 30);
disp(unique(detectedArucoIDs))

% Final plot of the entire robot path
figure('Name', 'Final Robot Path with Aruco Markers');
plot(state_history(:,1), state_history(:,2), 'b-', 'LineWidth', 2);
hold on;

% Plot the detected Aruco markers
marker_keys = keys(markerPositions);
for i = 1:length(marker_keys)
    marker_id = marker_keys{i};
    marker_pos = markerPositions(marker_id);
    plot(marker_pos(1), marker_pos(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    text(marker_pos(1)+0.05, marker_pos(2)+0.05, num2str(marker_id), 'Color', 'r', 'FontSize', 12);
end

xlabel('X Position');
ylabel('Y Position');
title('Final Robot Path with Aruco Markers');
legend('Robot Path', 'Aruco Markers');
grid on;
axis equal;
hold off;