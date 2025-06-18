function [outputArg1,outputArg2] = turnBody2Ball(headYawSensor,inputArg2)
%TURNBODY2BALL Function that turn the body of Nao to be in front of the
% ball
% Define motions needed
turn_l_40_motion = wbu_motion_new('../../motions/TurnLeft40.motion');
turn_l_60_motion = wbu_motion_new('../../motions/TurnLeft60.motion');
turn_r_40_motion = wbu_motion_new('../../motions/TurnRight40.motion');
turn_r_60_motion = wbu_motion_new('../../motions/TurnRight60.motion');

outputArg1 = inputArg1;
outputArg2 = inputArg2;
end

