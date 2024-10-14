% ... (after the main simulation loop)

% Prepare data for landmark evaluation
vis_data.positions = slam.x(4:2:end-1, end)';  % Assuming last column is final estimate
vis_data.nums = 1:size(landmarks, 2);  % Assuming landmark numbers are 1, 2, 3, ...
true_landmark_nums = 1:size(landmarks, 2);

% Evaluate landmark estimates
rms_error = evaluate_landmarks(vis_data, landmarks', true_landmark_nums);

% Display RMS error
fprintf('Landmark position RMS error: %.4f m\n', rms_error);
