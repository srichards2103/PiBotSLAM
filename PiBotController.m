classdef PiBotController
    properties
        pb
        cameraParams
        arucoDict
        markerLength
        state
        stateHistory
        detectedArucoIDs
        plottedMarkers
        markerPositions
        blackPixelThreshold
        consecutiveThreshold
        belowThresholdCount
        camAxes
        stateAxes
        pastTime
    end

    methods
        function obj = PiBotController(ipAddress)
            % Constructor to initialize the PiBot and necessary parameters
            addpath("../simulator/");
            addpath('arucoDetector');
            addpath('arucoDetector/include');
            addpath('arucoDetector/dictionary');

            % Load calibration and Aruco dictionary
            load('calibrationSession.mat');
            load('arucoDetector/dictionary/arucoDict.mat');
            obj.cameraParams = calibrationSession.CameraParameters;
            obj.arucoDict = arucoDict;
            obj.markerLength = 0.073;

            % Initialize PiBot
            obj.pb = PiBot(ipAddress);

            % Initialize plotting figures
            figure('Name', 'Robot Camera and State');
            obj.camAxes = subplot(1,2,1);
            title(obj.camAxes, 'Robot Camera');

            obj.stateAxes = subplot(1,2,2);
            title(obj.stateAxes, 'Robot Path');
            xlabel(obj.stateAxes, 'X Position');
            ylabel(obj.stateAxes, 'Y Position');
            hold(obj.stateAxes, 'on');

            % Initialize state
            obj.state = [0; 0; 0]; % [X; Y; Theta]
            obj.pastTime = datetime('now');
            obj.stateHistory = obj.state';
            obj.detectedArucoIDs = [];
            obj.plottedMarkers = containers.Map('KeyType', 'double', 'ValueType', 'logical');
            obj.markerPositions = containers.Map('KeyType', 'double', 'ValueType', 'any');

            % Initialize stopping criteria
            obj.blackPixelThreshold = 90;
            obj.consecutiveThreshold = 5;
            obj.belowThresholdCount = 0;
        end

        function followLine(obj)
            % Main loop to follow the line
            end_of_line = false;
            while ~end_of_line
                tic;
                % Get the current camera frame
                img = obj.pb.getImage();

                % Detect landmarks using Aruco markers
                [marker_nums, landmark_centres, marker_corners] = detectArucoPoses(img, obj.markerLength, obj.cameraParams, obj.arucoDict);

                if ~isempty(marker_nums)
                    % Append detected marker numbers to the list
                    obj.detectedArucoIDs = [obj.detectedArucoIDs, marker_nums'];
                    fprintf("Marker %d detected\n", marker_nums)

                    % Iterate through each detected marker
                    for i = 1:length(marker_nums)
                        marker_id = marker_nums(i);
                        if marker_id > 30
                            continue;
                        end

                        % Check if the marker has already been plotted
                        if ~isKey(obj.plottedMarkers, marker_id)
                            % Get the landmark center in robot frame
                            landmark_robot = landmark_centres(i, :)'; % [X; Y; Z] in robot frame

                            landmark_global = obj.getLandmarkGlobal(landmark_robot);

                            % Plot the marker on the stateAxes
                            plot(obj.stateAxes, landmark_global(1), landmark_global(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
                            text(obj.stateAxes, landmark_global(1)+0.05, landmark_global(2)+0.05, num2str(marker_id), 'Color', 'r', 'FontSize', 12);

                            % Mark the marker as plotted
                            obj.plottedMarkers(marker_id) = true;

                            % Store Marker Global Position
                            obj.markerPositions(marker_id) = landmark_global;
                        end
                    end
                end

                % Binarize the image with a threshold
                gray_img = rgb2gray(img);
                bin_img = ~imbinarize(gray_img, 0.4);

                % Crop the binary image to the bottom third for line detection
                [height, width] = size(bin_img);
                bottom_third_bin_img = bin_img(round(2*height/3):end, :);

                % Display the cropped binary image in the camera subplot
                imshow(bottom_third_bin_img, 'Parent', obj.camAxes);

                % Count the number of black pixels in the cropped image
                blackPixelCount = sum(bottom_third_bin_img(:));

                % Update the consecutive frame counter based on blackPixelCount
                if blackPixelCount < obj.blackPixelThreshold
                    obj.belowThresholdCount = obj.belowThresholdCount + 1;
                else
                    obj.belowThresholdCount = 0; % Reset if the count is above threshold
                end

                % Check if the robot has reached the end of the line
                if obj.belowThresholdCount >= obj.consecutiveThreshold
                    end_of_line = true;
                    break;
                end

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
                angular_velocity = -0.8  * centerOfMass(1);

                % Determine linear velocity based on the position of the line
                if abs(centerOfMass(1)) > 0.2
                    linear_velocity = 0.15;
                else
                    linear_velocity = 0.25;
                end

                % Calculate wheel velocities using inverse kinematics
                [wl, wr] = inverse_kinematics(linear_velocity, angular_velocity);

                % Helper function to round wheel velocities to the nearest lower multiple of 5
                round_to_lower_5 = @(value) 5 * floor(value / 5);
                wl = round_to_lower_5(wl);
                wr = round_to_lower_5(wr);

                % Calculate the actual velocities after rounding
                [real_u, real_q] = forward_kinematics(wl, wr);

                % Update the robot's state based on kinematics
                current_time = datetime('now');
                dt = seconds(current_time - obj.pastTime);
                obj.state = integrate_kinematics(obj.state, dt, real_u, real_q);
                obj.pastTime = current_time;

                % Append the current state to the history for plotting
                obj.stateHistory = [obj.stateHistory; obj.state'];

                % Set the robot's wheel velocities
                obj.pb.setVelocity(wl, wr);

                % Forward kinematics to confirm real velocities
                [real_u, real_q] = forward_kinematics(wl, wr);

                % Plot the robot's path in the state subplot
                plot(obj.stateAxes, obj.stateHistory(:,1), obj.stateHistory(:,2), 'b-');
                drawnow;

                toc
            end

            % Stop the robot by setting wheel velocities to zero
            obj.pb.stop();
            disp('Robot has stopped at the end of the line.');

            % Display all detected unique Aruco marker IDs if detection was enabled
            obj.detectedArucoIDs = obj.detectedArucoIDs(obj.detectedArucoIDs < 30);
            disp(unique(obj.detectedArucoIDs))

            % Final plot of the entire robot path
            figure('Name', 'Final Robot Path with Aruco Markers');
            plot(obj.stateHistory(:,1), obj.stateHistory(:,2), 'b-', 'LineWidth', 2);
            hold on;

            % Plot the detected Aruco markers
            marker_keys = keys(obj.markerPositions);
            for i = 1:length(marker_keys)
                marker_id = marker_keys{i};
                marker_pos = obj.markerPositions(marker_id);
                plot(marker_pos(1), marker_pos(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
                text(marker_pos(1)+0.05, marker_pos(2)+0.05, num2str(marker_id), 'Color', 'r', 'FontSize', 12);
            end

            xlabel('X Position');
            ylabel('Y Position');
            title('Final Robot Path with Aruco Markers');
            legend('Robot Path', 'Aruco Markers');
            grid on;
            axis equal;
            hold off;
        end

        function stop(obj)
            % Method to stop the robot
            obj.pb.stop();
            disp('Robot has been stopped.');
        end

        function landmark_global = getLandmarkGlobal(obj, landmark_robot)
            % Transform landmark position to global frame based on robot's state
            theta = obj.state(3);
            rotation_matrix = [cos(theta) -sin(theta) 0;
                sin(theta)  cos(theta) 0;
                0           0          1];
            landmark_global = obj.state(1:2) + rotation_matrix(1:2,1:2) * landmark_robot(1:2);
        end
    end
end