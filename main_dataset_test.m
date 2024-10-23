
% Always begin by using addpath
addpath("simulator")

% Add the ARUCO detector
% Check the example in the folder
addpath("arucoDetector")
addpath("arucoDetector/include")
addpath("arucoDetector/dictionary")

% Load parameters
load('arucoDetector/dictionary/arucoDict.mat');
load("calibrationSession.mat")
marker_length = 0.075;

cameraParams = calibrationSession.CameraParameters;

% Initialize EKF class
EKF = ekf_slam();

% Initialize thresholds and counters for line following (if still needed)
blackPixelThreshold = 90;        % Minimum number of black pixels to consider the line as present
consecutiveThreshold = 2;        % Number of consecutive frames below the threshold to confirm end of line
belowThresholdCount = 0;         % Counter for consecutive frames below the threshold

% Initialize plotting
fig = figure;

% Robot plot (Error Ellipse)
robotPlot = plot(0, 0, 'g-', 'LineWidth', 2);
hold on;

% Initialize cell arrays for landmarks' uncertainty ellipses and text labels
landmarkPlots = cell(1, 30);      % Assuming maximum of 30 landmarks
landmarkTexts = cell(1, 30);      % Cell array to store text handles for marker IDs
lm_cmap = colormap(hsv(30));       % Color map for landmarks

% Load and plot track_data
load('track_data.mat');
trackPlot = plot(x_path, y_path, 'k-', 'LineWidth', 2);

% Initialize robot path plot
robotPathPlot = plot(0, 0, 'g-', 'LineWidth', 1.5);

% Set plot properties
axis equal;
xlim([min(x_path)-1, max(x_path)+1]);
ylim([min(y_path)-1, max(y_path)+1]);
grid on;
title('Robot Path, Landmark Positions, and Track');
xlabel('X (m)');
ylabel('Y (m)');

% Initialize visualization data struct
vis_data = struct('time', {}, 'robot_pos', {}, 'robot_cov', {}, 'landmark_pos', {}, 'landmark_cov', {}, 'landmark_nums', {});

% Initialize cell array for text labels
landmarkTexts = cell(1, 30);  % Assuming a maximum of 30 landmarks

% Load the test dataset
load('datasets/test_dataset_2310_slow.mat');  % Assumes test_dataset is a struct with fields 'dt' and 'image'

% INITIAL STATE
u = 0;  % Initialize control inputs (modify as needed)
q = 0;

% Initialize robot path (if applicable)
current_robot_path = [];

% Initialize timer for visualization data
start_time = tic; % Start timer for the entire run

% Iterate through each entry in the test dataset
num_entries = length(test_dataset);
for idx = 1:num_entries  % 
    % Get the current data entry
    dt = test_dataset(idx).dt;
    img = test_dataset(idx).image;
    
    % EKF PREDICT
    EKF.predict(dt, u, q);
    
    % Measure landmarks and update EKF
    [marker_nums, landmark_centres, ~] = detectArucoPoses(img, marker_length, cameraParams, arucoDict);
    
    if ~isempty(marker_nums)
        % Filter out markers with IDs >= 30 and those more than 2m away
        valid_markers = (marker_nums < 30) & (vecnorm(landmark_centres(:,1:2), 2, 2) <= 2);
        marker_nums = marker_nums(valid_markers);
        
        landmark_centres = landmark_centres(valid_markers, 1:2)';
        
        % Call EKF input_measurements
        EKF.update(landmark_centres, marker_nums);
    end
    
    % Get estimates from EKF
    [robot_pos, robot_cov] = EKF.output_robot();
    [landmarks_pos, landmarks_cov] = EKF.output_landmarks();
    
    % Use the existing figure for visualization
    figure(fig);
    
    % Robot position plot (Error Ellipse)
    [robot_ellipse_x, robot_ellipse_y] = error_ellipse(robot_pos, robot_cov);
    set(robotPlot, 'XData', robot_ellipse_x, 'YData', robot_ellipse_y);
    
    % Update robot path plot
    current_robot_path = [current_robot_path; robot_pos(1:2)'];  % Assuming robot_pos has at least 2 elements
    set(robotPathPlot, 'XData', current_robot_path(:,1), 'YData', current_robot_path(:,2));
    
    % Update landmarks plot with ellipses and marker IDs
    for j = 1:size(landmarks_pos, 2)  % Renamed loop variable to 'j'
        if isempty(landmarkPlots{j}) || ~isvalid(landmarkPlots{j})
            landmarkPlots{j} = plot(NaN, NaN, 'Color', lm_cmap(j,:), 'LineWidth', 2);
        end
            
        % Extract landmark covariance
        landmark_cov_i = landmarks_cov(2*j-1:2*j, 2*j-1:2*j);
        
        % Calculate ellipse parameters
        [ellipse_x, ellipse_y] = error_ellipse(landmarks_pos(:,j), landmark_cov_i);
        
        % Update ellipse plot
        set(landmarkPlots{j}, 'XData', ellipse_x, 'YData', ellipse_y);
        
        % Get the marker ID from EKF's idx2num mapping
        marker_id = EKF.idx2num(j);
        
        % Initialize and update text labels for marker IDs
        if isempty(landmarkTexts{j}) || ~isvalid(landmarkTexts{j})
            % Create a new text object with a slight offset for readability
            offset = 0.1;  % Adjust as needed based on scale
            landmarkTexts{j} = text(landmarks_pos(1,j) + offset, landmarks_pos(2,j) + offset, ...
                                     num2str(marker_id), ...
                                     'Color', lm_cmap(j,:), 'FontSize', 12, 'FontWeight', 'bold', ...
                                     'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
        else
            % Update existing text object
            set(landmarkTexts{j}, 'Position', landmarks_pos(1:2,j)' + [0.1, 0.1], ...
                                     'String', num2str(marker_id));
        end
    end

    drawnow;
    
    % Get the idx2num array from EKF
    idx2num = EKF.idx2num;
    
    % Store data for later visualization and evaluation
    current_time = toc(start_time);
    vis_data(end+1).time = current_time;
    vis_data(end).robot_pos = robot_pos(1:3);
    vis_data(end).robot_cov = robot_cov(1:2, 1:2);
    vis_data(end).landmark_pos = landmarks_pos;
    vis_data(end).landmark_cov = landmarks_cov;
    vis_data(end).landmark_nums = idx2num;  % Store all landmark numbers using idx2num
    
    % Binarize the image with a threshold
    gray_img = rgb2gray(img);
    bin_img = ~imbinarize(gray_img, 0.4);
    
    % Crop the binary image to the bottom third for line detection
    [height, width] = size(bin_img);
    bottom_third_bin_img = bin_img(round(2*height/3):end, :);
    
    % Count the number of black pixels in the cropped image
    blackPixelCount = sum(bottom_third_bin_img(:));
    
    % Update the consecutive frame counter based on blackPixelCount
    if blackPixelCount < blackPixelThreshold
        belowThresholdCount = belowThresholdCount + 1;
    else
        belowThresholdCount = 0; % Reset if the count is above threshold
    end
    
    if belowThresholdCount <= consecutiveThreshold
        % Follow line iteration (You might need to adjust this function)
        [u, q, wl, wr] = followLineIteration(height, width, bottom_third_bin_img);
    else
        u = 0;
        q = 1;

        % Turn 180 degrees (if applicable)
        turn_time = pi/q;
        
        % Calculate wheel velocities using inverse kinematics
        [wl, wr] = inverse_kinematics(u, q);

        % Helper function to round wheel velocities to the nearest lower multiple of 5
        round_to_lower_5 = @(value) 5 * floor(value / 5);
        wl = round_to_lower_5(wl);
        wr = round_to_lower_5(wr);

        % Calculate the actual velocities after rounding
        [u, q] = forward_kinematics(wl, wr);

    end
end

% Save visualization data to a file
save('visualization_data.mat', 'vis_data');

% Evaluate landmark estimates
[rms_error, R, t] = evaluate_landmarks(vis_data, false);  % Adjust the parameters as needed
disp("Landmark position RMS error: ");
disp(rms_error);

% Plot the trajectory
plot_trajectory(vis_data, R, t);
