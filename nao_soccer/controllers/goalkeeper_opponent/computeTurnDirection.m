function [angle_diff, turn_direction] = computeTurnDirection(current_quat, goal_yaw, threshold)
    % compute_turn_direction
    % Computes the minimal rotation angle and best turn direction to reach a
    % desired yaw angle from the robot's current quaternion orientation.
    % If within a threshold, returns 'stop'.
    %
    % Inputs:
    %   current_quat : [w x y z] current orientation quaternion from IMU
    %   goal_yaw     : desired yaw angle (in radians, from -pi to pi)
    %   threshold    : threshold (radians) within which robot stops turning
    %
    % Outputs:
    %   angle_diff     : absolute minimal angle difference to rotate (radians)
    %   turn_direction : 'left', 'right', or 'stop'

    % Convert quaternion to Euler angles (ZYX intrinsic rotation)
    eul = quat2eul(current_quat, 'ZYX');
    current_yaw = eul(1);  % yaw angle around Z-axis

    % Calculate raw angle difference
    angle_diff = goal_yaw - current_yaw;

    % Normalize angle_diff to range (-pi, pi]
    angle_diff = atan2(sin(angle_diff), cos(angle_diff));

    % Check threshold condition
    if abs(angle_diff) <= threshold
        turn_direction = 'stop';
        angle_diff = 0;
    else
        % Determine direction
        if angle_diff > 0
            turn_direction = 'left';
        else
            turn_direction = 'right';
        end
        angle_diff = abs(angle_diff);
    end
end