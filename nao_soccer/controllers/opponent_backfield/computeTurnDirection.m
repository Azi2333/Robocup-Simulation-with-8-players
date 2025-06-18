function [turn_direction] = computeTurnDirection(yaw, team, threshold)
% compute_turn_direction
% TODO: do a more flexible implementation, maybe using quaternions
if strcmp(team, 'opponent')
  if abs(yaw) < threshold
    turn_direction = 'stop';
  elseif yaw > 0
    turn_direction = 'right';
  else
    turn_direction = 'left';
  end
else
  if (yaw > 0 && yaw > (pi - threshold)) || (yaw < 0 && yaw < -(pi - threshold))
    turn_direction = 'stop';
  elseif yaw > 0
    turn_direction = 'left';
  else
    turn_direction = 'right';
  end
end
