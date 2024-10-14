function [wl, wr] = inverse_kinematics(u, q)
    % Compute the left and right wheel velocities (wl, wr) required for the robot
    % to achieve a forward speed u and angular speed q.

    % The scale parameter and wheel track required to solve this are provided here.
    % You can find these values in the robot simulator as well.
    % In real-life, you would have to measure or calibrate them!
    % scale_parameter = 4.50e-3;
    % scale_parameter = 5.33e-3;
    % wheel_track = 0.;
    % wheel_track = 0.156; % The distance between the robot wheels (m).
    wheel_track = 0.156;
    scale_parameter = 5.43e-3;

    wl = 1/scale_parameter * u - wheel_track/(2*scale_parameter)*q;
    wr = 1/scale_parameter * u + wheel_track/(2*scale_parameter)*q;

    % Check if wl or wr are outside the range [-100, 100] and issue a warning
    if wl < -100 || wl > 100
        warning('Left wheel velocity (wl) = %.2f is out of the acceptable range [-100, 100]', wl);
    end

    if wr < -100 || wr > 100
        warning('Right wheel velocity (wr) = %.2f is out of the acceptable range [-100, 100]', wr);
    end
end
