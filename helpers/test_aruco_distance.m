% Add necessary paths
addpath("arucoDetector")
addpath("arucoDetector/include")
addpath("arucoDetector/dictionary")
% Load parameters
load("arucoDict.mat")
load("calibrationSession.mat")
marker_length = 0.075; % Adjust this to match your ArUco marker size

% Initialize the pibot connection
pb = PiBot('192.168.50.1');

% Main loop
while true
    % Capture an image
    img = pb.getImage();
    
    % Detect ArUco markers
    [marker_nums, landmark_centres, marker_corners] = detectArucoPoses(img, marker_length, cameraParams, arucoDict);
    
    % If markers are detected
    if ~isempty(marker_nums)
        % Calculate distances
        distances = sqrt(sum(landmark_centres.^2, 2));
        
        % Print marker numbers and distances
        for i = 1:length(marker_nums)
            fprintf('Marker %d: Distance = %.2f m\n', marker_nums(i), distances(i));
        end
    else
        fprintf('No markers detected\n');
    end
    
    % Add a small delay to avoid overwhelming the system
    pause(0.1);
    
    % Check for user input to exit the loop
    if ishghandle(1) && strcmp(get(gcf,'CurrentCharacter'),'q')
        break;
    end
end

% Clean up
clear pb;