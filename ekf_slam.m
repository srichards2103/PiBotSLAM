classdef ekf_slam < handle
    %EKF_SLAM The EKF algorithm for SLAM

    properties
        x = zeros(3,1); % The estimated state vector
        P = zeros(3,3); % The estimated state covariance

        % The covariance values provided here are NOT correct!
        sigxy = 0.1; % The covariance of linear velocity
        sigth = 0.01; % The covariance of angular velocity
        siglm = 0.01; % The covariance of landmark measurements

        % R = [siglm, 0; 0, siglm]; % Covariance matrix for landmark measurements

        idx2num = []; % The map from state vector index to landmark id.
        n = 0; % The number of landmarks
        m = 0; % The number of measured landmarks
    end

    methods
        function predict(obj, dt, lin_velocity, ang_velocity)
            % Perform the predict step of the EKF. This involves updating
            % the state and covariance estimates using the input velocity,
            % the time step, and the covariance of the update step.
            obj.x(1:3) = f(obj.x, dt, lin_velocity, ang_velocity);

            A = jac_f(obj.x, dt, lin_velocity);
            % Add the NxN identity matrix to the diagonal of A (where N is the number of landmarks)
            
            A = [A, zeros(3, 2*obj.n); zeros(2*obj.n, 3), eye(2*obj.n)];

            obj.P = A * obj.P * A';

        end

        function update(obj, measurements, nums)
            % Perform the innovation step of the EKF. This involves adding
            % new (not previously seen) landmarks to the state vector and
            % implementing the EKF innovation equations. You will need the
            % landmark measurements and you will need to be careful about
            % matching up landmark ids with their indices in the state
            % vector and covariance matrix.
            
            % transform measurements to ref frame
            world_measurements = obj.meas2y(measurements);
            obj.add_new_landmarks(world_measurements, nums);

            idx = find(ismember(obj.idx2num, nums));

            C_k = jac_h(obj.x, idx);
            R_k = [obj.siglm, 0; 0, obj.siglm]; % Covariance matrix for landmark measurements
            
            disp('Matrix sizes:');
            disp(['obj.P: ', mat2str(size(obj.P)), ', C_k: ', mat2str(size(C_k)), ...
                  ', obj.x: ', mat2str(size(obj.x)), ', world_measurements: ', mat2str(size(world_measurements))]);
            disp('Desired Matrix sizes:');
            disp(['A: ', mat2str([3+2*obj.n,3+2*obj.n]), ...
            ', C: ', mat2str([2*length(idx),3+2*obj.n])]);

            % From lecture Notes:
            K = obj.P * C_k' / (C_k * obj.P * C_k' + R_k); % Kalman Gain
            
            obj.x = obj.x + K * (world_measurements - h(obj.x, idx)); % Update State
            obj.P = obj.P - K * C_k * obj.P; % Update Covariance

        end
        
        function add_new_landmarks(obj, y, nums)
            % Add a new (not seen before) landmark to the state vector and
            % covariance matrix. You will need to associate the landmark's
            % id number with its index in the state vector.
            for num = nums(~(ismember(nums, obj.idx2num)))
                obj.x = [obj.x; y(:)];

                obj.P(3+2*obj.n+1,3+2*obj.n+1) = obj.siglm;
                obj.P(3+2*obj.n+2,3+2*obj.n+2) = obj.siglm;

                obj.n = obj.n+1;
                obj.idx2num = [obj.idx2num,num];
            end
        end

        % function add_new_landmarks(obj, y, nums)
        %     % Add a new (not seen before) landmark to the state vector and
        %     % covariance matrix. You will need to associate the landmark's
        %     % id number with its index in the state vector.
        %     % transform landmark to world frame
        %     % loop through each landmark
        %     for i = 1:size(y, 2)
        %         landmark_world = transform_landmark(obj.x, y(1, i), y(2, i));
        %         % Update state vector x
        %         fprintf("Length of state vector: %d \n", length(obj.x))
        %         obj.x = [obj.x, landmark_world];  % Append landmark_world horizontally
        %         fprintf("Length of state vector new: %d \n", length(obj.x))
                
        %         % Update covariance matrix P
        %         n = size(obj.P, 1);
        %         obj.P = [obj.P, zeros(n, 2); zeros(2, n), diag([obj.siglm, obj.siglm])];
        %         obj.idx2num = [obj.idx2num, nums(i)];
        %     end
        % end

        function [robot, cov] = output_robot(obj)
            % Suggested: output the part of the state vector and covariance
            % matrix corresponding only to the robot.
            robot = obj.x(1:3);
            cov = obj.P(1:3, 1:3);
        end

        function [landmarks, cov] = output_landmarks(obj)
            % Suggested: output the part of the state vector and covariance
            % matrix corresponding only to the landmarks.
            landmarks = obj.x(4:end);
            cov = obj.P(4:end, 4:end);
        end

        function y = meas2y(obj,measurements)
            % meas2y: Transform landmark measurements from robot to world frame
            % Input: measurements (2xN matrix) in robot frame
            % Output: y (2xN matrix) in world frame
            y = zeros(size(measurements));
            for i = 1:size(measurements,2)
                theta = obj.x(3);
                Rot = [cos(theta) -sin(theta); 
                     sin(theta)  cos(theta)];
                y(:,i) = obj.x(1:2) + Rot * measurements(:,i);
            end
        end

    end
end

% Jacobians and System Functions

function x1 = f(x0, dt, u, q)
% integrate the input u from the state x0 to obtain x1.
    x1 = integrate_kinematics(x0, dt, u, q);
end

function F = jac_f(x0, dt, u)
% Given the state x0 and input signal u, compute the Jacobian of f.
    F = eye(3);
    F(1, 3) = - dt*sin(x0(3)*u);
    F(2,3) = dt*cos(x0(3)*u);
end

function y = h(x, idx)
    % Given the state x and a list of indices idx, compute the state measurement y.
    % Extract the state variables
    x_k = x(1);  % x position
    y_k = x(2);  % y position
    theta_k = x(3);  % orientation (theta)

    % Initialize y based on the number of indices (m = length of idx)
    m = length(idx);
    y = zeros(2*m, 1);

    % Loop through each index to compute the corresponding measurement
    for i = 1:m
        % Retrieve the corresponding l_ix and l_iy based on idx
        % Get the index of idx in obj.idx2num
        l_ix = x(3+idx(i));  % x-offset for index i
        l_iy = x(3+idx(i) + 1);  % y-offset for index i

        % Compute the transformed position using the formula from the image
        h_x = -cos(theta_k) * (x_k - l_ix) - sin(theta_k) * (y_k - l_iy);
        h_y =  sin(theta_k) * (x_k - l_ix) - cos(theta_k) * (y_k - l_iy);

        % Store the result in y (as a vector with two components per index)
        y(i, :) = h_x;
        y(i+1, :) = h_y;
    end
end

function H = jac_h(x, idx)
    % Given the state x and a list of indices idx, compute the Jacobian of
    % the measurement function h.

    % Extract the state variables
    x_k = x(1);  % x position
    y_k = x(2);  % y position
    theta_k = x(3);  % orientation (theta)
    
    % Number of landmarks and total state dimension
    N = (length(x)-3)/2;  % total number of landmarks in the state
    m = length(idx);  % number of observed landmarks

    % Initialize the full Jacobian matrix H (2m x (3 + 2N))
    H = zeros(2 * m, 3 + 2 * N);
    disp(size(H))
    % Loop through each observed landmark index
    for i = 1:m
        % Retrieve the corresponding landmark offsets l_ix and l_iy
        landmark_idx = idx(i);
        l_ix = x(3 + 2*(landmark_idx-1) + 1);
        l_iy = x(3 + 2*(landmark_idx-1) + 2);

        % Compute the partial derivatives for the ith landmark
        delta_x = x_k - l_ix;
        delta_y = y_k - l_iy;

        % The submatrix C_tilde^i_k based on the structure from the image
        C_tilde = [-cos(theta_k), -sin(theta_k),  sin(theta_k) * delta_x - cos(theta_k) * delta_y;
                    sin(theta_k), -cos(theta_k), -cos(theta_k) * delta_x - sin(theta_k) * delta_y];

        % Place the C_tilde submatrix in the appropriate position in the full Jacobian matrix
        H(2*i-1:2*i, 1:3) = C_tilde;

        % Fill in the zeros for the other entries, except the identity for the landmark part
        H(2*i-1:2*i, 3 + 2*(i-1) + 1:3 + 2*i) = [cos(theta_k), sin(theta_k); -sin(theta_k), cos(theta_k)];
        disp(size(H))
    end
end

function ref = transform_landmark(x, l_ix, l_iy)
    % Transform landmark coordinates from world frame to robot frame
    x_k = x(1);
    y_k = x(2);
    theta_k = x(3);

    ref_x = -cos(theta_k) * (x_k - l_ix) - sin(theta_k) * (y_k - l_iy);
    ref_y =  sin(theta_k) * (x_k - l_ix) - cos(theta_k) * (y_k - l_iy);

    ref = [ref_x; ref_y];
end
