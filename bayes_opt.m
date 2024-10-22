% Add necessary paths
addpath("simulator", "arucoDetector", ...
        "arucoDetector/include", "arucoDetector/dictionary");

% Load parameters
load('arucoDetector/dictionary/arucoDict.mat');
load("calibrationSession.mat");
marker_length = 0.075;
cameraParams = calibrationSession.CameraParameters;

% Load the test dataset
load('test_dataset.mat');  % Ensure this file exists and contains the required data

% Load the ground truth data for RMSE computation
load('track_data.mat');    % Contains 'x_path' and 'y_path' for ground truth

% Define the ranges for the covariance parameters
sigxy_bounds  = [0.00005, 0.07];
sigth_bounds  = [0.00005, 0.08];
siglmx_bounds = [0.00005, 0.08];
siglmy_bounds = [0.00005, 0.08];

% Set up optimizable variables
vars = [
    optimizableVariable('sigxy',  sigxy_bounds, 'Transform', 'log');
    optimizableVariable('sigth',  sigth_bounds, 'Transform', 'log');
    optimizableVariable('siglmx', siglmx_bounds, 'Transform', 'log');
    optimizableVariable('siglmy', siglmy_bounds, 'Transform', 'log');
];

% Run Bayesian optimization
results = bayesopt(@(params) run_simulation_wrapper(params, test_dataset, marker_length, ...
                                                    cameraParams, arucoDict), ...
                   vars, 'AcquisitionFunctionName', 'expected-improvement-plus', ...
                   'MaxObjectiveEvaluations', 300, 'IsObjectiveDeterministic', true, ...
                   'UseParallel', false, 'Verbose', 1, ...
                   'PlotFcn', {@plotAcquisitionFunction}, ...
                   'UseParallel', true);

% Extract the best parameters
best_params = results.XAtMinObjective;
best_rmse = results.MinObjective;

% Save the best parameters
save('best_result.mat', 'best_params', 'best_rmse');

fprintf('Final Best RMSE achieved with:\n');
fprintf('sigxy = %.5f\n', best_params.sigxy);
fprintf('sigth = %.5f\n', best_params.sigth);
fprintf('siglmx = %.5f\n', best_params.siglmx);
fprintf('siglmy = %.5f\n', best_params.siglmy);
fprintf('RMSE = %.5f\n', best_rmse);

% Objective function for Bayesian optimization
function rmse = run_simulation_wrapper(params, test_dataset, marker_length, cameraParams, arucoDict)
    % Extract parameters
    sigxy  = params.sigxy;
    sigth  = params.sigth;
    siglmx = params.siglmx;
    siglmy = params.siglmy;
    
    % Run the EKF SLAM simulation with the given parameters
    rmse = run_ekf_slam_simulation(test_dataset, marker_length, ...
                                   cameraParams, arucoDict, ...
                                   sigxy, sigth, siglmx, siglmy);
end

% Function to run the EKF SLAM simulation with given covariance parameters
function rmse = run_ekf_slam_simulation(test_dataset, marker_length, ...
                                        cameraParams, arucoDict, ...
                                        sigxy, sigth, siglmx, siglmy)
    % Initialize EKF SLAM with given covariances
    EKF = ekf_slam();
    EKF.sigxy  = sigxy;
    EKF.sigth  = sigth;
    EKF.siglmx = siglmx;
    EKF.siglmy = siglmy;
     % Initialize thresholds and counters for line following (if still needed)
     blackPixelThreshold = 90;        % Minimum number of black pixels to consider the line as present
    consecutiveThreshold = 2;        % Number of consecutive frames below the threshold to confirm end of line
    belowThresholdCount = 0;         % Counter for consecutive frames below the threshold
    
    % Initialize visualization data struct for RMSE computation
    vis_data = struct('time', {}, 'robot_pos', {}, 'robot_cov', {}, ...
                      'landmark_pos', {}, 'landmark_cov', {}, 'landmark_nums', {});

    % INITIAL STATE
    u = 0;  % Linear velocity (assumed constant or as per your dataset)
    q = 0;  % Angular velocity (assumed constant or as per your dataset)
    
    % Iterate through each entry in the test dataset
    num_entries = length(test_dataset);
    end_time = 240;
    start_time = tic; % Start timer for the entire run
    for i = 1:num_entries
        % Get the current data entry
        dt = test_dataset(i).dt;
        img = test_dataset(i).image;
        
        % EKF PREDICT
        EKF.predict(dt, u, q);
        
        % Measure landmarks and update EKF
        [marker_nums, landmark_centres, ~] = detectArucoPoses(img, marker_length, cameraParams, arucoDict);
        
        if ~isempty(marker_nums)
            % Filter out markers with IDs >= 30 and those more than 2m away
            valid_markers = (marker_nums < 30) & (vecnorm(landmark_centres(:,1:2), 2, 2) <= 2);
            marker_nums = marker_nums(valid_markers);
            landmark_centres = landmark_centres(valid_markers, 1:2)';
            
            % Call EKF update
            EKF.update(landmark_centres, marker_nums);
        end
        
        % Get estimates from EKF
        [robot_pos, robot_cov] = EKF.output_robot();
        [landmarks_pos, landmarks_cov] = EKF.output_landmarks();
        
        % Store data for RMSE computation
        vis_data(end+1).robot_pos    = robot_pos(1:3);
        vis_data(end).robot_cov       = robot_cov(1:2, 1:2);
        vis_data(end).landmark_pos    = landmarks_pos;
        vis_data(end).landmark_cov    = landmarks_cov;
        vis_data(end).landmark_nums   = EKF.idx2num;
        
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
        
        current_time = toc(start_time);
        
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
    
    % Compute RMSE between estimated landmark positions and ground truth
    rmse = evaluate_landmarks(vis_data, false);
    disp(['RMSE: ', num2str(rmse)]);
end
