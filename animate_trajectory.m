function animate_trajectory(vis_data)
    % Create a new figure
    fig = figure('Name', 'Robot and Landmark Trajectory', 'Position', [100, 100, 800, 600]);
    
    % Get the total number of frames
    num_frames = length(vis_data);
    
    % Initialize plots
    hold on;
    robot_plot = plot(NaN, NaN, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'none');
    landmark_plots = cell(1, 30);  % Assuming maximum of 30 landmarks
    landmark_labels = cell(1, 30);  % Text objects for landmark numbers
    
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
        landmark_nums = vis_data(frame).landmark_nums;
        
        for i = 1:size(landmark_pos, 2)
            if isempty(landmark_plots{i})
                landmark_plots{i} = plot(NaN, NaN, 'Color', cmap(i,:), 'LineWidth', 2);
                landmark_labels{i} = text(NaN, NaN, '', 'Color', cmap(i,:), 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            end
            
            % Extract landmark covariance
            landmark_cov_i = landmark_cov(2*i-1:2*i, 2*i-1:2*i);
            
            % Calculate ellipse parameters
            [ellipse_x, ellipse_y] = calculate_error_ellipse(landmark_pos(:,i), landmark_cov_i);
            
            % Update ellipse plot
            set(landmark_plots{i}, 'XData', ellipse_x, 'YData', ellipse_y);
            
            % Update text position and content
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

function [x, y] = calculate_error_ellipse(mean, covariance)
    % Calculate the eigenvectors and eigenvalues
    [eigenvec, eigenval] = eig(covariance);
    
    % Get the index of the largest eigenvector
    [largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
    largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);
    
    % Get the largest eigenvalue
    largest_eigenval = max(max(eigenval));
    
    % Get the smallest eigenvector and eigenvalue
    if largest_eigenvec_ind_c == 1
        smallest_eigenval = eigenval(2,2);
        smallest_eigenvec = eigenvec(:,2);
    else
        smallest_eigenval = eigenval(1,1);
        smallest_eigenvec = eigenvec(1,:);
    end
    
    % Calculate the angle between the x-axis and the largest eigenvector
    angle = atan2(largest_eigenvec(2), largest_eigenvec(1));
    
    % This angle is between -pi and pi.
    % Let's shift it such that the angle is between 0 and 2pi
    if(angle < 0)
        angle = angle + 2*pi;
    end
    
    % Get the 95% confidence interval error ellipse
    chisquare_val = 5.991;
    theta_grid = linspace(0, 2*pi);
    phi = angle;
    X0 = mean(1);
    Y0 = mean(2);
    a = sqrt(chisquare_val * largest_eigenval);
    b = sqrt(chisquare_val * smallest_eigenval);
    
    % The ellipse in x and y coordinates
    ellipse_x_r = a * cos(theta_grid);
    ellipse_y_r = b * sin(theta_grid);
    
    % Define a rotation matrix
    R = [cos(phi) sin(phi); -sin(phi) cos(phi)];
    
    % Rotate the ellipse to some angle phi
    r_ellipse = [ellipse_x_r; ellipse_y_r]' * R;
    
    % Draw the error ellipse
    x = r_ellipse(:,1) + X0;
    y = r_ellipse(:,2) + Y0;
end
