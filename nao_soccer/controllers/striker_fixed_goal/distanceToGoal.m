function [distance] = distanceToGoal(goal_position,robot_position)
%distanceToGoal Calculates the distance t
distance =  sqrt(power(goal_position(1)-robot_position(1),2)+power(goal_position(2)-robot_position(2),2));
end