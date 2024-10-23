function plot_trajectory(vis_data, R, t)
    persistent plotted_landmarks
    
    figure;
    hold on;
    
    % Plot robot trajectory with color gradient, varying circle size, and transparency
    robot_positions = cat(2, vis_data.robot_pos);
    num_positions = size(robot_positions, 2);
    colors = jet(num_positions); % Create a color map
    
    % Get the current number of landmarks
    current_num_landmarks = size(vis_data(1).landmark_pos, 2);
    
    % Initialize or resize plotted_landmarks if necessary
    if isempty(plotted_landmarks) || length(plotted_landmarks) ~= current_num_landmarks
        plotted_landmarks = zeros(1, current_num_landmarks);
    end
    
    for i = 1:num_positions
        % Get position uncertainty (assuming it's stored in vis_data.robot_cov)
        pos_cov = vis_data(i).robot_cov(1:2, 1:2);
        uncertainty = sqrt(trace(pos_cov));
        
        % Scale circle size based on uncertainty (adjust scaling factor as needed)
        circle_size = 5 + 1000 * uncertainty;
        
        % Plot each position with color based on time, size based on uncertainty, and partial transparency
        scatter(robot_positions(1,i), robot_positions(2,i), circle_size, colors(i,:), 'filled', 'MarkerFaceAlpha', 0.5);
        
        % Plot landmark estimates for this time step
        landmark_pos = vis_data(i).landmark_pos;
        landmark_cov = vis_data(i).landmark_cov;
        num_landmarks = size(landmark_pos, 2);
        
        for j = 1:num_landmarks
            % Ensure plotted_landmarks has an entry for this landmark
            if j > length(plotted_landmarks)
                plotted_landmarks(j) = 0;
            end
            
            % Check if we've already plotted this landmark a few times
            if plotted_landmarks(j) >= 20
                continue;  % Skip this landmark if we've already plotted it 10 times
            end
            
            % Get landmark uncertainty
            lm_cov = landmark_cov(2*j-1:2*j, 2*j-1:2*j);
            lm_uncertainty = sqrt(trace(lm_cov));
            
            % Scale marker size based on uncertainty
            lm_size = 1000 * lm_uncertainty;
            
            % Plot landmark estimate with size based on uncertainty
            scatter(landmark_pos(1,j), landmark_pos(2,j), lm_size, 'b+', 'MarkerEdgeAlpha', 0.5);
            
            % Increment the count for this landmark
            plotted_landmarks(j) = plotted_landmarks(j) + 1;
        end
    end
    
    % Plot final landmark positions with landmark numbers
    final_landmark_pos = vis_data(end).landmark_pos;
    num_landmarks = size(final_landmark_pos, 2);
    
    for j = 1:num_landmarks
        x = final_landmark_pos(1,j);
        y = final_landmark_pos(2,j);
        landmark_num = vis_data(end).landmark_nums(j);
        text(x, y, num2str(landmark_num), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'r', 'FontWeight', 'bold');
    end
    
    title('Robot Trajectory and Landmark Positions');
    xlabel('X (m)');
    ylabel('Y (m)');
    colorbar('Ticks', [0, 0.5, 1], 'TickLabels', {'Start', 'Middle', 'End'});
    legend('Robot Trajectory', 'Landmark Estimates', 'Final Landmark Positions');
    grid on;
    axis equal;
    hold off;

% function plot_trajectory(vis_data, R, t)
%     % plot_trajectory - Plot robot trajectory and landmark positions
%     %
%     % Inputs:
%     %   vis_data - Struct containing robot and landmark positions
%     %   R        - (Optional) Rotation matrix for alignment
%     %   t        - (Optional) Translation vector for alignment
%     %
%     % Example Usage:
%     %   plot_trajectory(vis_data); % Plot original trajectory
%     %   plot_trajectory(vis_data, R, t); % Plot original and aligned trajectories

%     % Use a persistent variable to track plotted landmarks across multiple calls
%     persistent plotted_landmarks

%     % Determine the maximum number of landmarks across all frames
%     max_landmarks = max(arrayfun(@(x) size(x.landmark_pos, 2), vis_data));

%     % Initialize or resize plotted_landmarks if necessary
%     if isempty(plotted_landmarks)
%         plotted_landmarks = zeros(1, max_landmarks);
%     elseif length(plotted_landmarks) < max_landmarks
%         plotted_landmarks(end+1:max_landmarks) = 0;
%     end

%     % Create a new figure
%     figure;
%     hold on;

%     % Extract robot positions
%     robot_positions = cat(2, vis_data.robot_pos); % 2xN matrix
%     num_positions = size(robot_positions, 2);
%     colors = jet(num_positions); % Create a color map based on time

%     % % % Plot Original Robot Trajectory
%     % scatter(robot_positions(1, :), robot_positions(2, :), 36, colors, 'filled', ...
%     %     'MarkerFaceAlpha', 0.5, 'DisplayName', 'Original Trajectory');

%     % Plot Aligned Robot Trajectory if R and t are provided
%     if nargin == 3 && ~isempty(R) && ~isempty(t)
%         % Apply rotation and translation to robot positions
%         transformed_robot_positions = R * robot_positions(1:2, :) + t;

%         % Plot Aligned Robot Trajectory
%         scatter(transformed_robot_positions(1, :), transformed_robot_positions(2, :), 36, colors, ...
%             'filled', 'MarkerFaceAlpha', 0.5, 'DisplayName', 'Aligned Trajectory');
%     end

%     % Plot Landmark Estimates
%     for i = 1:num_positions
%         landmark_pos = vis_data(i).landmark_pos; % 2xM
%         landmark_cov = vis_data(i).landmark_cov;
%         num_landmarks = size(landmark_pos, 2);

%         for j = 1:num_landmarks
%             % Ensure plotted_landmarks has an entry for this landmark
%             if j > length(plotted_landmarks)
%                 % Expand plotted_landmarks to accommodate new landmark
%                 plotted_landmarks(j) = 0;
%             end

%             % Skip if landmark has been plotted enough times
%             if plotted_landmarks(j) >= 20
%                 continue;
%             end

%             % Get landmark uncertainty
%             lm_cov = landmark_cov(2*j-1:2*j, 2*j-1:2*j);
%             lm_uncertainty = sqrt(trace(lm_cov));

%             % Scale marker size based on uncertainty
%             lm_size = 1000 * lm_uncertainty;

%             % Plot landmark estimate with size based on uncertainty
%             scatter(landmark_pos(1, j), landmark_pos(2, j), lm_size, 'bx', ...
%                 'MarkerEdgeAlpha', 0.5, 'DisplayName', 'Landmark Estimates');

%             % Increment the count for this landmark
%             plotted_landmarks(j) = plotted_landmarks(j) + 1;
%         end
%     end

%     % Plot Final Landmark Positions with Landmark Numbers
%     final_landmark_pos = vis_data(end).landmark_pos; % 2xM
%     num_final_landmarks = size(final_landmark_pos, 2);

%     for j = 1:num_final_landmarks
%         x = final_landmark_pos(1, j);
%         y = final_landmark_pos(2, j);
%         landmark_num = vis_data(end).landmark_nums(j);
%         text(x, y, num2str(landmark_num), 'HorizontalAlignment', 'center', ...
%             'VerticalAlignment', 'middle', 'Color', 'r', 'FontWeight', 'bold');
%     end


%     % Enhance Plot
%     title('Robot Trajectory and Landmark Positions');
%     xlabel('X (m)');
%     ylabel('Y (m)');
%     colorbar('Ticks', [0, 0.5, 1], 'TickLabels', {'Start', 'Middle', 'End'});
%     if nargin == 3 && ~isempty(R) && ~isempty(t)
%         legend('Aligned Trajectory', 'Landmark Estimates', ...
%             'Final Landmark Positions', 'Location', 'best');
%     else
%         legend('Original Trajectory', 'Landmark Estimates', 'Final Landmark Positions', ...
%             'Location', 'best');
%     end
%     grid on;
%     axis equal;
%     hold off;
% end
