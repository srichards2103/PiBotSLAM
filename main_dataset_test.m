% Adjusted MATLAB Code to Process test_dataset.mat

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

% Simulation flag is no longer needed since we're processing a dataset
% simulation = false;
end_time = 120;  % Adjust as needed or remove if not applicable

% Initialize the PiBot connection is no longer needed
% if simulation
%     % Simulation-specific code...
% else
%     pb = PiBot('192.168.50.1');
% end

% Initialize EKF class
EKF = ekf_slam();

% Initialize thresholds and counters for line following (if still needed)
blackPixelThreshold = 90;        % Minimum number of black pixels to consider the line as present
consecutiveThreshold = 2;        % Number of consecutive frames below the threshold to confirm end of line
belowThresholdCount = 0;         % Counter for consecutive frames below the threshold

% Initialize plotting
fig = figure;

robotPlot = plot(0, 0, 'g-', 'LineWidth', 2);
hold on;
landmarkPlots = cell(1, 30);  % Assuming maximum of 30 landmarks
lm_cmap = colormap(hsv(30));

% Load and plot track_data
load('track_data.mat');
trackPlot = plot(x_path, y_path, 'k-', 'LineWidth', 2);

% Initialize robot path plot
robotPathPlot = plot(0, 0, 'g-', 'LineWidth', 1.5);

axis equal;
xlim([min(x_path)-2, max(x_path)+2]);
ylim([min(y_path)-2, max(y_path)+2]);
grid on;
title('Robot Path, Landmark Positions, and Track');
xlabel('X (m)');
ylabel('Y (m)');

% Initialize visualization data struct
vis_data = struct('time', {}, 'robot_pos', {}, 'robot_cov', {}, 'landmark_pos', {}, 'landmark_cov', {}, 'landmark_nums', {});

% Load the test dataset
load('test_dataset_1.mat');  % Assumes test_dataset is a struct with fields 'dt' and 'image'

% INITIAL STATE
u = 0;  % Initialize control inputs (modify as needed)
q = 0;

% Initialize robot path (if applicable)
current_robot_path = [];

% Initialize timer for visualization data
start_time = tic; % Start timer for the entire run

% Iterate through each entry in the test dataset
num_entries = length(test_dataset);
for i = 1:num_entries
    % Get the current data entry
    dt = test_dataset(i).dt;
    img = test_dataset(i).image;
    
    % EKF PREDICT
    EKF.predict(dt, u, q);
    
    % Store the current dt and img if needed (optional)
    % test_dataset(end+1).dt = dt;  % Not needed since we're loading the dataset
    % test_dataset(end).image = img;
    
    % Measure landmarks and update EKF
    
    [marker_nums, landmark_centres, ~] = detectArucoPoses(img, marker_length, cameraParams, arucoDict);
    
    if ~isempty(marker_nums)
        % Filter out markers with IDs >= 30 and those more than 2m away
        disp(marker_nums);
        valid_markers = (marker_nums < 30) & (vecnorm(landmark_centres(1:2)) <= 2);
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
    
    % Update landmarks plot
    for j = 1:size(landmarks_pos, 2)
        if isempty(landmarkPlots{j}) || ~isvalid(landmarkPlots{j})
            landmarkPlots{j} = plot(NaN, NaN, 'Color', lm_cmap(j,:), 'LineWidth', 2);
        end
            
        % Extract landmark covariance
        landmark_cov_i = landmarks_cov(2*j-1:2*j, 2*j-1:2*j);
        
        % Calculate ellipse parameters
        [ellipse_x, ellipse_y] = error_ellipse(landmarks_pos(:,j), landmark_cov_i);
        
        % Update ellipse plot
        set(landmarkPlots{j}, 'XData', ellipse_x, 'YData', ellipse_y);
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
    
    % Control Logic (Modify as needed based on dataset or application)
    if current_time > end_time
        break;
    elseif belowThresholdCount <= consecutiveThreshold
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

% END LINE FOLLOW
% Since there's no robot, no need to stop it
% pb.stop();

% Save visualization data to a file
save('visualization_data.mat', 'vis_data');

% Save the processed dataset if needed
% save('test_dataset_processed.mat', 'test_dataset');  % Optional

% Evaluate landmark estimates
rms_error = evaluate_landmarks(vis_data, false);  % Adjust the parameters as needed
disp("Landmark position RMS error: ");
disp(rms_error);

% Plot the trajectory
plot_trajectory(vis_data);

