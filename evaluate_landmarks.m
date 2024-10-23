function [rms_error, R, t] = evaluate_landmarks(vis_data, simulation, pb)
    % evaluate_landmarks - Calculate RMS error of estimated landmark positions
    %
    % Inputs:
    %   vis_data - struct containing estimated landmark positions and numbers
    %   simulation - boolean flag indicating whether this is a simulation
    %   pb - (optional) PiBot or PiBotSim object (only needed for simulation)

    %
    % Output:
    %   rms_error - Root Mean Square error of landmark position estimates
    
    if simulation
        true_landmarks = pb.worldLandmarkPositions;
        true_landmark_nums = 1:size(true_landmarks, 2);
    else
        % Load true landmarks from a JSON file
        fid = fopen('groundtruth_demo.json');
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

    % Calculate RMSD of non aligned points
    squared_deviations = sum((true_points - est_points).^2, 1);
    rmsd_non_aligned = sqrt(mean(squared_deviations));

    disp("RMSD of non aligned points: ");
    disp(rmsd_non_aligned);
    
    % Save the landmarks to a JSON file
    save_landmarks_to_json(last_frame.landmark_nums, last_frame.landmark_pos, 'estimated_landmarks.json');
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
% Add this new function at the end of the file
function save_landmarks_to_json(landmark_nums, landmark_pos, filename)
    % Create a container for the landmark data
    landmark_data = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
    % Populate the container with landmark numbers and positions
    for i = 1:length(landmark_nums)
        key = num2str(landmark_nums(i));
        landmark_data(key) = landmark_pos(:, i)';
    end
    
    % Sort the keys (landmark numbers)
    sorted_keys = sort(cellfun(@str2num, landmark_data.keys));
    
    % Create the JSON string manually
    json_str = '{';
    for i = 1:length(sorted_keys)
        key = num2str(sorted_keys(i));
        value = landmark_data(key);
        json_str = [json_str sprintf('"%s": [%.3f, %.3f]', key, value(1), value(2))];
        if i < length(sorted_keys)
            json_str = [json_str, ', '];
        end
    end
    json_str = [json_str '}'];
    
    % Write the JSON string to a file
    fid = fopen(filename, 'w');
    if fid == -1
        error('Cannot create JSON file');
    end
    fwrite(fid, json_str, 'char');
    fclose(fid);
    
    fprintf('Landmarks saved to %s\n', filename);
end
