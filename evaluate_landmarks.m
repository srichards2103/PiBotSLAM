function rms_error = evaluate_landmarks(vis_data, simulation, pb)
    % evaluate_landmarks - Calculate RMS error of estimated landmark positions
    %
    % Inputs:
    %   vis_data - struct containing estimated landmark positions and numbers
    %   simulation - boolean flag indicating whether this is a simulation
    %   pb - PiBot or PiBotSim object (only needed for simulation)
    %
    % Output:
    %   rms_error - Root Mean Square error of landmark position estimates
    
    if simulation
        true_landmarks = pb.worldLandmarkPositions;
        true_landmark_nums = 1:size(true_landmarks, 2);
    else
        % Load true landmarks from a file
        % load('true_landmarks.mat');
        error('Non-simulation mode not implemented yet');
    end

    % Initialize variables
    num_landmarks = length(true_landmark_nums);
    squared_errors = zeros(num_landmarks, 1);

    % Get the last frame of vis_data
    last_frame = vis_data(end);

    % Loop through each true landmark
    for i = 1:num_landmarks
        true_num = true_landmark_nums(i);
        true_pos = true_landmarks(:, i)';  % Transpose to make it a row vector
        
        % Find the corresponding estimated landmark
        est_idx = find(last_frame.landmark_nums == true_num, 1);
        
        if ~isempty(est_idx)
            est_pos = last_frame.landmark_pos(:, est_idx)';  % Transpose to make it a row vector
            
            % Calculate squared error
            squared_errors(i) = sum((true_pos - est_pos).^2);
        else
            % If landmark not found in estimates, set error to NaN
            squared_errors(i) = NaN;
        end
    end

    % Calculate RMS error, ignoring NaN values
    valid_errors = squared_errors(~isnan(squared_errors));
    rms_error = sqrt(mean(valid_errors));
end
