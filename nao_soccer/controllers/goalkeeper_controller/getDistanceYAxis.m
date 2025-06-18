function [distance] = getDistanceYAxis(xyzDistance, cameraFixedAngle, detectedAngle, headYawAngle)
%GETDISTANCEXYPLANE Summary of this function goes here
%   Detailed explanation goes here
distance =  (xyzDistance * cos(cameraFixedAngle))*sin(detectedAngle + headYawAngle);
end

