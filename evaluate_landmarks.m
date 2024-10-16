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
        % Load true landmarks from a JSON file
        fid = fopen('groundtruth_1016.json');
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
    end

    % Initialize variables
    num_landmarks = length(true_landmark_nums);
    
    % Get the last frame of vis_data
    last_frame = vis_data(end);
    
    % Prepare point sets for Kabsch algorithm
    true_points = [];
    est_points = [];
    
    % Loop through each true landmark
    for i = 1:num_landmarks
        true_num = true_landmark_nums(i);
        true_pos = true_landmarks(:, i);
        
        % Find the corresponding estimated landmark
        est_idx = find(last_frame.landmark_nums == true_num, 1);
        
        if ~isempty(est_idx)
            est_pos = last_frame.landmark_pos(:, est_idx);
            
            % Add points to the sets
            true_points = [true_points, true_pos];
            est_points = [est_points, est_pos];
        end
    end
    
    % Apply Kabsch algorithm
    [R, t] = kabsch(est_points, true_points);
    
    % Transform estimated points
    aligned_est_points = R * est_points + t;
    
    % Calculate RMSD
    squared_deviations = sum((true_points - aligned_est_points).^2, 1);
    rmsd = sqrt(mean(squared_deviations));
    
    % Set the output
    rms_error = rmsd;
end

% Kabsch algorithm implementation
function [R, t] = kabsch(P, Q)
    % Center the point sets
    centroid_P = mean(P, 2);
    centroid_Q = mean(Q, 2);
    P_centered = P - centroid_P;
    Q_centered = Q - centroid_Q;
    
    % Compute the covariance matrix
    H = P_centered * Q_centered';
    
    % Compute the optimal rotation matrix
    [U, ~, V] = svd(H);
    R = V * U';
    
    % Ensure a right-handed coordinate system
    if det(R) < 0
        V(:, 3) = -V(:, 3);
        R = V * U';
    end
    
    % Compute the translation
    t = centroid_Q - R * centroid_P;
end
