function [u, q] = forward_kinematics(wl, wr)
% Compute the forward linear and angular velocities (u, q) from the left and right wheel velocities (wl, wr)

% The scale parameter and wheel track used in inverse kinematics
wheel_track = 0.156; % The distance between the robot wheels (m).
scale_parameter = 5.43e-3;

% Compute the linear and angular velocities
u = scale_parameter * (wl + wr) / 2;
q = scale_parameter * (wr - wl) / wheel_track;
end
