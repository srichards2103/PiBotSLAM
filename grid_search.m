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

% Define refined ranges for covariance parameters

% Linear velocity covariance (sigxy)
% Adjust based on robot's motion precision
sigxy_range = 0.01:0.005:0.07;    % From low to high uncertainty

% Angular velocity covariance (sigth)
% Adjust based on robot's rotation precision
sigth_range = 0.02:0.04:0.08;      % From low to high uncertainty

% Landmark X measurement covariance (siglmx)
% Adjust based on sensor's X-axis measurement accuracy
siglmx_range = 0.015:0.005:0.08;   % From high precision to low

% Landmark Y measurement covariance (siglmy)
% Adjust based on sensor's Y-axis measurement accuracy
siglmy_range = 0.01:0.005:0.045;   % From high precision to low

% Initialize variables to store results
num_combinations = length(sigxy_range) * length(sigth_range) * ...
                   length(siglmx_range) * length(siglmy_range);
results = struct('sigxy', {}, 'sigth', {}, 'siglmx', {}, 'siglmy', {}, 'rmse', {});

combination_idx = 1;

% Initialize best RMSE and best parameters
best_rmse = Inf;
best_params = struct('sigxy', NaN, 'sigth', NaN, 'siglmx', NaN, 'siglmy', NaN);

% Specify the file to save the best result
best_result_file = 'best_result.mat';

% Iterate over all combinations
for sigxy = sigxy_range
    for sigth = sigth_range
        for siglmx = siglmx_range
            for siglmy = siglmy_range
                % Run the simulation with the current covariance parameters
                rmse = run_ekf_slam_simulation(test_dataset, marker_length, ...
                                              cameraParams, arucoDict, ...
                                              sigxy, sigth, siglmx, siglmy);
                
                % Store the results
                results(combination_idx).sigxy = sigxy;
                results(combination_idx).sigth = sigth;
                results(combination_idx).siglmx = siglmx;
                results(combination_idx).siglmy = siglmy;
                results(combination_idx).rmse = rmse;
                
                % Display progress
                fprintf('Combination %d/%d: sigxy=%.3f, sigth=%.3f, ', ...
                        combination_idx, num_combinations, sigxy, sigth);
                fprintf('siglmx=%.3f, siglmy=%.3f, RMSE=%.4f\n', ...
                        siglmx, siglmy, rmse);
                
                % Check if current RMSE is better than the best RMSE
                if rmse < best_rmse
                    best_rmse = rmse;
                    best_params.sigxy = sigxy;
                    best_params.sigth = sigth;
                    best_params.siglmx = siglmx;
                    best_params.siglmy = siglmy;
                    
                    % Save the best parameters to a file
                    save(best_result_file, 'best_params', 'best_rmse');
                    
                    fprintf('--> New best RMSE: %.4f. Saved to %s\n', ...
                            best_rmse, best_result_file);
                end
                
                combination_idx = combination_idx + 1;
            end
        end
    end
end

% Final save of the best parameters after all simulations
save(best_result_file, 'best_params', 'best_rmse');

fprintf('Final Best RMSE achieved with:\n');
fprintf('sigxy = %.3f\n', best_params.sigxy);
fprintf('sigth = %.3f\n', best_params.sigth);
fprintf('siglmx = %.3f\n', best_params.siglmx);
fprintf('siglmy = %.3f\n', best_params.siglmy);
fprintf('RMSE = %.4f\n', best_rmse);

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
    disp(rmse);
end
