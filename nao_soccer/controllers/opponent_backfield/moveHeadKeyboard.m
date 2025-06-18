% Initialize Webots API and enable keyboard
wb_robot_init();
wb_keyboard_enable(32); % 32 ms timestep for keyboard updates

% Initialize head positions
headYaw = 0;    % initial yaw angle
headPitch = 0;  % initial pitch angle

% Define step size for each arrow key press
yawStep = 0.1;  % radians to move left/right
pitchStep = 0.1; % radians to move up/down

% Initialize clientID for Webots
clientID = wb_robot_get_time(); % assuming time as clientID for testing

% Simulation loop
while wb_robot_step(32) ~= -1
    % Read keyboard input
    key = wb_keyboard_get_key();
    
    % Adjust head angles based on arrow key input
    switch key
        case 315  % Up arrow key
            headPitch = headPitch + pitchStep;
        case 317  % Down arrow key
            headPitch = headPitch - pitchStep;
        case 316  % Right arrow key
            headYaw = headYaw + yawStep;
        case 314  % Left arrow key
            headYaw = headYaw - yawStep;
    end
    
    % Constrain headYaw and headPitch to avoid extreme angles
    headYaw = max(min(headYaw, 1.0), -1.0);   % limit yaw to [-1, 1] radians
    headPitch = max(min(headPitch, 0.5), -0.5); % limit pitch to [-0.5, 0.5] radians

    % Move head to new position
    moveNaoHead(clientID, headYaw, headPitch);
end

% Cleanup
wb_robot_cleanup();
