function new_state = integrate_kinematics(state, dt, lin_velocity, ang_velocity)
%INTEGRATE_KINEMATICS integrate the kinematics of the robot
%   state is the current state, and has the form [x;y;theta]
%   dt is the length of time for which to integrate
%   lin_velocity is the (forward) linear velocity of the robot
%   ang_velocity is the angular velocity of the robot
%   new_state is the state after integration, also in the form [x;y;theta]

    u = lin_velocity;
    q = ang_velocity;
    
    theta = state(3, 1);
    
    if q == 0
        x = state(1, 1) + u * dt * cos(theta);
        y = state(2, 1) + u * dt * sin(theta);
    else
        x = state(1, 1) + u/q * (sin(theta + q * dt) - sin(theta));
        y = state(2, 1) - u/q * (cos(theta + q * dt) - cos(theta));
    end
    
    theta = theta + dt * q;
    new_state = [x; y; theta];
end
