function [angle,distance] = GetAngleDistanceFromBall(detectionBoxes, cam_fov, camFocalLength, sensorWidth)
%UNTITLED Get the angle and distance from the ball from detection boxes
%  
  ball_diam = 0.14;
  bbox_top = detectionBoxes(1, :);  % only consider the first detected ball
  ball_center_x_top = bbox_top(1) + bbox_top(3) / 2;
  angle = -(ball_center_x_top - sensorWidth / 2) * (cam_fov / sensorWidth);
  distance = (camFocalLength * ball_diam) / bbox_top(4);      
end