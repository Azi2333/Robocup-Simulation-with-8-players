function moveNaoHead(clientID, headYaw, headPitch)
    % Function to move NAO robot's head in Webots simulation using MATLAB
    % Arguments:
    %   clientID - Connection ID for Webots
    %   headYaw - Desired head yaw angle in radians
    %   headPitch - Desired head pitch angle in radians

    % Define device names for head motors
    yawMotor = wb_robot_get_device('HeadYaw');
    pitchMotor = wb_robot_get_device('HeadPitch');
    
    % Set the position of the head motors
    wb_motor_set_position(yawMotor, headYaw);
    wb_motor_set_position(pitchMotor, headPitch);
    
    % Optional: Pause for a brief moment to observe movement
    pause(1);

    % Get current head position for confirmation
    currentYaw = wb_motor_get_target_position(yawMotor);
    currentPitch = wb_motor_get_target_position(pitchMotor);
    fprintf('Head moved to: Yaw = %.2f, Pitch = %.2f radians\n', currentYaw, currentPitch);
end