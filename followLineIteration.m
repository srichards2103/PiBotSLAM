function [u, q, wl, wr] = followLineIteration(height, width, bottom_third_bin_img)
    % Find the center of mass of the line pixels in the cropped image
    [rows, cols] = find(bottom_third_bin_img == 1);

    % Ensure that some pixels are detected to avoid NaN values
    if isempty(cols) || isempty(rows)
        centerOfMass = [0, 0];
    else
        centerOfMass = [mean(cols), mean(rows)];
    end

    % Normalize the center of mass relative to the image dimensions
    centerOfMass = (centerOfMass - [width/2, height/6]) ./ [width/2, height/6];

    % Proportional control for angular velocity based on the center of mass
    % angular_velocity = -0.8  * centerOfMass(1);

    % % Determine linear velocity based on the position of the line
    % if abs(centerOfMass(1)) > 0.2
    %     linear_velocity = 0.15;
    % else
    %     linear_velocity = 0.25;
    % end
    angular_velocity = -0.8  * centerOfMass(1);

    % Determine linear velocity based on the position of the line
    if abs(centerOfMass(1)) > 0.2
        linear_velocity = 0.125;
    else
        linear_velocity = 0.175;
    end

    % Calculate wheel velocities using inverse kinematics
    [wl, wr] = inverse_kinematics(linear_velocity, angular_velocity);

    % Helper function to round wheel velocities to the nearest lower multiple of 5
    round_to_lower_5 = @(value) 5 * floor(value / 5);
    wl = round_to_lower_5(wl);
    wr = round_to_lower_5(wr);

    % Calculate the actual velocities after rounding
    [u, q] = forward_kinematics(wl, wr);
    
end