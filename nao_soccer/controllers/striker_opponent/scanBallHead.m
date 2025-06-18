function [direction] = scanBallHead(headYawMotor, headYawSensor, direction, maxYawAngle)
    %UNTITLED This function move the head of Nao to look for the ball
    arguments
        headYawMotor
        headYawSensor
        direction
        maxYawAngle = pi/2;
    end
    yaw_step = deg2rad(2);
    motor_position = wb_position_sensor_get_value(headYawSensor); 
    temp_postition =  motor_position + direction * yaw_step;
    if (temp_postition < -maxYawAngle || temp_postition > maxYawAngle)
        direction =  direction * -1;
    else
        wb_motor_set_position(headYawMotor, temp_postition);
    end
end