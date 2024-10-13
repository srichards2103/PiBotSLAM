function animate_trajectory(vis_data)
    % Create a new figure
    fig = figure('Name', 'Robot and Landmark Trajectory', 'Position', [100, 100, 800, 600]);
    
    % Get the total number of frames
    num_frames = length(vis_data);
    
    % Initialize plots
    hold on;
    robot_plot = plot(NaN, NaN, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'none');
    landmark_plots = cell(1, 30);  % Assuming maximum of 30 landmarks
    landmark_labels = cell(1, 30);  % New: Text objects for landmark numbers
    
    % Set axis limits
    xlim([-1, 5]);
    ylim([-1, 5]);
    grid on;
    title('Robot and Landmark Positions Over Time');
    xlabel('X (m)');
    ylabel('Y (m)');
    
    % Create a colormap for landmarks
    cmap = colormap(hsv(30));
    
    % Main animation loop
    for frame = 1:num_frames
        % Update robot position and covariance
        robot_pos = vis_data(frame).robot_pos;
        robot_cov = vis_data(frame).robot_cov;
        
        % Calculate robot marker size based on covariance
        robot_size = 10 + 200 * sqrt(trace(robot_cov));
        set(robot_plot, 'XData', robot_pos(1), 'YData', robot_pos(2), 'MarkerSize', robot_size);
        
        % Update landmark positions and covariances
        landmark_pos = vis_data(frame).landmark_pos;
        landmark_cov = vis_data(frame).landmark_cov;
        landmark_nums = vis_data(frame).landmark_nums;  % New: Get landmark numbers
        
        for i = 1:size(landmark_pos, 2)
            if isempty(landmark_plots{i})
                landmark_plots{i} = plot(NaN, NaN, 'o', 'Color', cmap(i,:), 'MarkerSize', 8, 'MarkerFaceColor', 'none');
                % New: Create text object for landmark number
                landmark_labels{i} = text(NaN, NaN, '', 'Color', cmap(i,:), 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            end
            
            % Calculate landmark marker size based on covariance
            landmark_cov_i = landmark_cov(2*i-1:2*i, 2*i-1:2*i);
            landmark_size = 8 + 200 * sqrt(trace(landmark_cov_i));
            
            set(landmark_plots{i}, 'XData', landmark_pos(1,i), 'YData', landmark_pos(2,i), 'MarkerSize', landmark_size);
            
            % New: Update text position and content
            set(landmark_labels{i}, 'Position', [landmark_pos(1,i), landmark_pos(2,i)], 'String', num2str(landmark_nums(i)));
        end
        
        % Update title with current time
        title(sprintf('Robot and Landmark Positions at t = %.2f s', vis_data(frame).time));
        
        % Force drawing update
        drawnow;
        
        % Wait for user input before moving to the next frame
        if frame < num_frames
            waitforbuttonpress;
        end
    end
end
