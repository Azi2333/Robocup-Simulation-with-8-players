function [headYawPosition] = trackBallHead(headYawMotor, headYawSensor, currentAngle)
    %UNTITLED This function track the ball with the head of Nao
    %   Detailed explanation goes here
    angle_threshold = deg2rad(0.5); %Degrees
    yaw_step = deg2rad(0.5);
    headYawPosition = wb_position_sensor_get_value(headYawSensor); 
    if (~isnan(currentAngle))
        currentAngleRad =  deg2rad(currentAngle);
        if (currentAngleRad > angle_threshold)
        headYawSetpoint =  headYawPosition + yaw_step;
        elseif (currentAngleRad < -angle_threshold)
        headYawSetpoint =  headYawPosition - yaw_step;
        else 
            return
        end
        % Constrain headYaw and headPitch to avoid extreme angles
        headYawSetpoint = max(min(headYawSetpoint, pi), -pi);   % limit yaw to [-pi, pi] radians
        wb_motor_set_position(headYawMotor, headYawSetpoint);
    end
end