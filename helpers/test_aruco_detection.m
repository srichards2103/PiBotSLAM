% Test Robot Sensor Accuracy

% Add necessary paths
addpath("arucoDetector")
addpath("arucoDetector/include")
addpath("arucoDetector/dictionary")

% Load required data
load('arucoDetector/dictionary/arucoDict.mat');
load("calibrationSession.mat")
marker_length = 0.075;

% Initialize the PiBot
pb = PiBot('192.168.50.1');

% Set up camera parameters
cameraParams = calibrationSession.CameraParameters;

% Main loop
while true
    % Wait for keypress
    disp('Press any key to capture an image and estimate poses (or "q" to quit)...');
    key = input('', 's');
    
    if strcmpi(key, 'q')
        break;
    end
    
    % Capture image from the robot
    img = pb.getImage();
    
    % Detect ArUco markers and estimate poses
    [marker_nums, landmark_centres, marker_corners] = detectArucoPoses(img, marker_length, cameraParams, arucoDict);
    
    % Display results
    disp('Detected markers:');
    for i = 1:length(marker_nums)
        fprintf('Marker %d: Position = [%.3f, %.3f, %.3f]\n', marker_nums(i), landmark_centres(i,1), landmark_centres(i,2), landmark_centres(i,3));
    end
    
    % Visualize the detected markers on the image
    figure;
    imshow(img);
    hold on;
    for i = 1:size(marker_corners, 1)
        plot(marker_corners(i,:,1), marker_corners(i,:,2), 'g-', 'LineWidth', 2);
        text(marker_corners(i,1,1), marker_corners(i,1,2), num2str(marker_nums(i)), 'Color', 'r', 'FontSize', 12);
    end
    hold off;
    title('Detected ArUco Markers');
end

% Clean up
clear pb;

