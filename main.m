% Always begin by using addpath
% You can always test your algorithm in simulator
addpath("simulator")

% Add the ARUCO detector
% Check the example in the folder
addpath("arucoDetector")
addpath("arucoDetector/include")
addpath("arucoDetector/dictionary")

% Load parameters
load('arucoDetector/dictionary/arucoDict.mat');
load("calibrationSession.mat")
marker_length = 0.070;

cameraParams = calibrationSession.CameraParameters;

simulation = true;

% Initialize the pibot connection

if simulation
    pb = piBotSim("floor_course.jpg");
    % Start by placing your robot at the start of the line
    x0 = 1;
    y0 = 1;
    theta0 = 0;
    pb.place([x0;y0], theta0);
    pb.showLandmarks(true);
else
    pb = PiBot('192.168.50.1');
end

% Initialise your EKF class
EKF = ekf_slam();

blackPixelThreshold = 90;        % Minimum number of black pixels to consider the line as present
consecutiveThreshold = 5;        % Number of consecutive frames below the threshold to confirm end of line
belowThresholdCount = 0;         % Counter for consecutive frames below the threshold
tic;

figure;
robotPlot = plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
hold on;
landmarkPlot = plot(0, 0, 'b*', 'MarkerSize', 8);
xlim([-1, 5]);
ylim([-1, 5]);
grid on;
title('Robot and Landmark Positions');
xlabel('X (m)');
ylabel('Y (m)');

% Initialize visualization data struct
vis_data = struct('time', {}, 'robot_pos', {}, 'robot_cov', {}, 'landmark_pos', {}, 'landmark_cov', {});

% INITIAL STATE
u = 0;
q = 0;

start_time = tic; % Start timer for the entire run

while(true)
    dt = toc;
    tic;

    % EKF PREDICT
    EKF.predict(dt, u, q);

    % Get the current camera frame
    img = pb.getImage();

    % measure landmarks and update EKF
    if simulation
        [landmark_centres, marker_nums] = pb.measureLandmarks();
        
    else
        [marker_nums, landmark_centres, ~] = detectArucoPoses(img, marker_length, cameraParams, arucoDict);
    end

    % Filter out markers with IDs >= 30 and those more than 2m away
    valid_markers = (marker_nums < 30) & (vecnorm(landmark_centres) <= 2);
    marker_nums = marker_nums(valid_markers);
    landmark_centres = landmark_centres(:, valid_markers);

    if ~isempty(marker_nums)
        % Call EKF input_measurements
        EKF.update(landmark_centres, marker_nums);

        % Get estimates from EKF
        [robot_est, robot_cov] = EKF.output_robot();
        [landmarks_est, landmarks_cov] = EKF.output_landmarks();

        % Store data for visualization
        current_time = toc(start_time);
        vis_data(end+1).time = current_time;
        vis_data(end).robot_pos = robot_est(1:2);
        vis_data(end).robot_cov = robot_cov(1:2, 1:2);
        vis_data(end).landmark_pos = landmarks_est;
        vis_data(end).landmark_cov = landmarks_cov;

        % Update the plot
        set(robotPlot, 'XData', robot_est(1), 'YData', robot_est(2));
        set(landmarkPlot, 'XData', landmarks_est(1,:), 'YData', landmarks_est(2,:));
        drawnow;
    end

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

    % Check if the robot has reached the end of the line
    if belowThresholdCount >= consecutiveThreshold
        break;
    end
    
    % line follow module
    [u, q, wl, wr] = followLineIteration(height, width, bottom_third_bin_img);

    pb.setVelocity(wl, wr);
    
end

% END LINE FOLLOW
pb.stop();

% Save visualization data to a file
save('visualization_data.mat', 'vis_data');

% Animate the trajectory
animate_trajectory(vis_data);
