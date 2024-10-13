function plot_trajectory(vis_data)
    figure;
    hold on;
    % Plot robot trajectory
    robot_positions = cat(2, vis_data.robot_pos);
    plot(robot_positions(1,:), robot_positions(2,:), 'r-', 'LineWidth', 2);
    % Plot final landmark positions
    final_landmark_pos = vis_data(end).landmark_pos;
    scatter(final_landmark_pos(1,:), final_landmark_pos(2,:), 100, 'b*');
    title('Robot Trajectory and Landmark Positions');
    xlabel('X (m)');
    ylabel('Y (m)');
    legend('Robot Trajectory', 'Landmarks');
    grid on;
    axis equal;
    hold off;