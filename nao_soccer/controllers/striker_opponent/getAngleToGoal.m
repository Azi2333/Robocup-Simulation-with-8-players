function [angle_to_goal] = getAngleToGoal(goal_position,robot_position)
%getAngleToGoal Calculate angle from robot position to goalpost center
    opposite_cathetus =  goal_position(2) - robot_position(2);
    adjacent_cathetus =  goal_position(1) - robot_position(1);
    angle_to_goal = atan(opposite_cathetus/adjacent_cathetus);
end