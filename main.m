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
marker_length = 0.075;

cameraParams = calibrationSession.CameraParameters;

simulation = false;
end_time = 240;

save_data = false;
demo = true;
% Initialize the pibot connection

if simulation
    % Load true landmarks from a JSON file
    fid = fopen('groundtruths/groundtruth_1016.json');
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    json_data = jsondecode(str);
    
    % Extract landmark numbers and positions
    field_names = fieldnames(json_data);
    landmark_nums = cellfun(@(x) str2double(x(2:end)), field_names);
    landmark_positions = cell2mat(struct2cell(json_data)');
    
    % Sort landmarks by their numbers
    [true_landmark_nums, sort_idx] = sort(landmark_nums);
    true_landmarks = landmark_positions(:, sort_idx);

    pb = piBotSim("floor_course.jpg", true_landmarks+[1;1]);
    % Start by placing your robot at the start of the line
    pb.place([1;1], 0);
    pb.showLandmarks(true);
    
else
    pb = PiBot('192.168.50.1');

    if demo
        % Load true landmarks from a JSON file
        fid = fopen('groundtruth_demo.json');
        raw = fread(fid, inf);
        str = char(raw');
        fclose(fid);
        json_data = jsondecode(str);
    
        field_names = fieldnames(json_data);
        ground_truth_landmark_nums = cellfun(@(x) str2double(x(2:end)), field_names);
        % disp(ground_truth_landmark_nums)
    end
end

% Initialise your EKF class
EKF = ekf_slam();

blackPixelThreshold = 90;        % Minimum number of black pixels to consider the line as present
consecutiveThreshold = 2;        % Number of consecutive frames below the threshold to confirm end of line
belowThresholdCount = 0;         % Counter for consecutive frames below the threshold
tic;

fig = figure;
% robotPlot = plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
% hold on;
% landmarkPlot = plot(0, 0, 'b*', 'MarkerSize', 8);

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
xlim([min(x_path)-1, max(x_path)+1]);
ylim([min(y_path)-1, max(y_path)+1]);
grid on;
title('Robot Path, Landmark Positions, and Track');
xlabel('X (m)');
ylabel('Y (m)');
% legend([robotPlot, trackPlot, robotPathPlot], ...
%        {'Robot', 'Track', 'Robot Path'}, 'Location', 'best');

% Initialize visualization data struct
vis_data = struct('time', {}, 'robot_pos', {}, 'robot_cov', {}, 'landmark_pos', {}, 'landmark_cov', {}, 'landmark_nums', {}, 'landmarks_seen', {});

if save_data 
    test_dataset = struct('image', {}, 'dt', {});
end

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
    try
        img = pb.getImage();
    catch e
        disp("Error in image capture:");
        disp(e.message);
        disp(e.stack(1));
        imshow(img)
        break;  % Exit the loop if image capture consistently fails
    end
    if save_data            
        test_dataset(end+1).dt = dt;
        test_dataset(end).image = img;
    end

    % measure landmarks and update EKF
    if simulation
        [landmark_centres, marker_nums] = pb.measureLandmarks();
    else
        [marker_nums, landmark_centres, ~] = detectArucoPoses(img, marker_length, cameraParams, arucoDict);
    end

    if ~isempty(marker_nums)
        % Filter out markers with IDs >= 30 and those more than 2m away
        % valid_markers = (marker_nums < 30) & (vecnorm(landmark_centres(1:2)) <= 2);

        % filter out markers with IDs not in ground truth
        valid_markers = ismember(marker_nums, ground_truth_landmark_nums);
        disp(valid_markers);
        
        marker_nums = marker_nums(valid_markers);
        
        if simulation
            landmark_centres = landmark_centres(:, valid_markers);
        else
            landmark_centres = landmark_centres(valid_markers, 1:2)';
        end

        % Call EKF input_measurements
        EKF.update(landmark_centres, marker_nums);

    end

    % Get estimates from EKF
    [robot_pos, robot_cov] = EKF.output_robot();
    [landmarks_pos, landmarks_cov] = EKF.output_landmarks();
    
    % Use the existing figure
    figure(fig);
    
    % Robot position plot
    [robot_ellipse_x,robot_ellipse_y] = error_ellipse(robot_pos,robot_cov);
    set(robotPlot, 'XData', robot_ellipse_x, 'YData', robot_ellipse_y);
    
    % Update robot path plot
    robot_path_x = cellfun(@(x) x(1), {vis_data.robot_pos});
    robot_path_y = cellfun(@(x) x(2), {vis_data.robot_pos});

    set(robotPathPlot, 'XData', robot_path_x, 'YData', robot_path_y);

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
    vis_data(end).landmarks_seen = marker_nums;

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


    if current_time > end_time
        break;
    elseif belowThresholdCount <= consecutiveThreshold
        [u, q, wl, wr] = followLineIteration(height, width, bottom_third_bin_img);
    else
        u = 0;
        q = 1;

        % turn 180 degrees
        turn_time = pi/q;
        
        % Calculate wheel velocities using inverse kinematics
        [wl, wr] = inverse_kinematics(u, q);

        % Helper function to round wheel velocities to the nearest lower multiple of 5
        round_to_lower_5 = @(value) 5 * floor(value / 5);
        wl = round_to_lower_5(wl);
        wr = round_to_lower_5(wr);

        % Calculate the actual velocities after rounding
        [u, q] = forward_kinematics(wl, wr);
        pb.setVelocity(wl, wr);
        pause(turn_time);
        wl = 0;
        wr = 0;
    end

    pb.setVelocity(wl, wr);
    
end

% END LINE FOLLOW
pb.stop();

% Save visualization data to a file
save('visualization_data.mat', 'vis_data');

if save_data
    save('test_dataset.mat', 'test_dataset');
end

% evaluate landmark estimates
if simulation
    [rms_error, R, t] = evaluate_landmarks(vis_data, simulation, pb);
else
    [rms_error, R, t] = evaluate_landmarks(vis_data, simulation);
end

disp("Landmark position RMS error: ");
disp(rms_error);

% plot the trajectory
plot_trajectory(vis_data, R, t);
