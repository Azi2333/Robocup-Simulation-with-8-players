% MATLAB controller for Webots: basic_controller.m
function goalkeeper_opponent
%Set team 
team_type = 'opponent';
% set time step for each simulation step in milliseconds
TIME_STEP = 64;

% get and enable devices
headYawOffset= deg2rad(25);
camera_top = wb_robot_get_device('CameraTop');
camera_bottom = wb_robot_get_device('CameraBottom');
wb_camera_enable(camera_top, TIME_STEP);
wb_camera_enable(camera_bottom, TIME_STEP);
camera_top_angle = headYawOffset + deg2rad(1.2);

% camera parameters based on 640*480
focalLength = 560;  % estimate this value based on the camera properties
sensorWidth = 640;  % width of the camera in pixels
fov = 60;           % field of view of the camera in degrees

% Set initial position
% Head motors
headYawMotor = wb_robot_get_device('HeadYaw');
headPitchMotor = wb_robot_get_device('HeadPitch');
wb_motor_set_position(headPitchMotor, headYawOffset); % 25 degrees - max positive angle

% Arms Position
LShoulderPitch = wb_robot_get_device('LShoulderPitch');
RShoulderPitch = wb_robot_get_device('RShoulderPitch');
wb_motor_set_position(LShoulderPitch, pi/2)
wb_motor_set_position(RShoulderPitch, pi/2)
% Sensors Initilisation
headYawSensor  = wb_robot_get_device('HeadYawS');
wb_position_sensor_enable(headYawSensor, TIME_STEP)
wb_keyboard_enable(TIME_STEP)

% basic motions
forwards_motion = wbu_motion_new('../../motions/Forwards.motion');
backwards_motion = wbu_motion_new('../../motions/Backwards.motion');
turn_l_180_motion = wbu_motion_new('../../motions/TurnLeft180.motion');
left_step_motion = wbu_motion_new('../../motions/SideStepLeft.motion');
right_step_motion = wbu_motion_new('../../motions/SideStepRight.motion');
% Variables initialisation
% headYawDirection = 1; %Positive direction
headYawDirection = -1; % Negative direction
state =  'idle';
% make the robot walk continuously
% wbu_motion_set_loop(forwards_motion, true);
% wbu_motion_play(forwards_motion);

% initialize YOLOv4 model
model = yolov4ObjectDetector('tiny-yolov4-coco');
loop_counter = 0;
approach_counter = 0;
backwards_counter = 0;

isMoving = false;
currentMotion = left_step_motion;
while wb_robot_step(TIME_STEP) ~= -1

  % read the sensors
  rgb_top = wb_camera_get_image(camera_top);
  rgb_bottom = wb_camera_get_image(camera_bottom);

  % initialize display text for angle and distance for both cameras
  angle_top = NaN;
  distance_top = NaN;
  angle_bottom = NaN;
  distance_bottom = NaN;

  % perform object detection using the specified threshold
  confidenceThreshold = 0.5;
  [bboxes_top, ~, labels_top] = detect(model, rgb_top, 'Threshold', confidenceThreshold);
  [bboxes_bottom, ~, labels_bottom] = detect(model, rgb_bottom, 'Threshold', confidenceThreshold);

  % filter bounding boxes for detections
  labelsToDetect = 'sports ball';
  idx_top = string(labels_top) == labelsToDetect;
  idx_bottom = string(labels_bottom) == labelsToDetect;
  filteredBboxes_top = bboxes_top(idx_top, :);
  filteredBboxes_bottom = bboxes_bottom(idx_bottom, :);

  % calculate angle and distance for CameraTop
  if ~isempty(filteredBboxes_top)
    ball_detected_top = true;
    rgb_top = insertObjectAnnotation(rgb_top, 'rectangle', filteredBboxes_top, cellstr(labels_top(idx_top)));
    [angle_top,distance_top] = GetAngleDistanceFromBall(filteredBboxes_top, fov, focalLength,sensorWidth);
  else
    ball_detected_top = false;
  end

  % calculate angle and distance for CameraBottom
  if ~isempty(filteredBboxes_bottom)
    ball_detected_bottom = true;
    rgb_bottom = insertObjectAnnotation(rgb_bottom, 'rectangle', filteredBboxes_bottom, cellstr(labels_bottom(idx_bottom)));
    [angle_bottom,distance_bottom] = GetAngleDistanceFromBall(filteredBboxes_bottom, fov, focalLength,sensorWidth);
  else
    ball_detected_bottom = false;
  end

  % display camera top image
  subplot(2, 2, 1);
  image(rgb_top);
  title('CameraTop');
  text(10, 10, sprintf('Angle: %.1f°  Distance: %.2f m', angle_top, distance_top), ...
    'Units', 'normalized', 'Position', [0.5, -0.2], 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'k');

  % display camera bottom image
  subplot(2, 2, 2);
  image(rgb_bottom);
  title('CameraBottom');
  text(10, 10, sprintf('Angle: %.1f°  Distance: %.2f m', angle_bottom, distance_bottom), ...
    'Units', 'normalized', 'Position', [0.5, -0.2], 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'k');
  %drawnow;
  if (isMoving == true)
    isMotionOver = wbu_motion_is_over(currentMotion);
    if (isMotionOver)
      isMoving = false;
    else
      continue;
    end
  end

% Check if the robot have fall
  % Read Euler angles (Roll, Pitch, Yaw)
  angles = wb_inertial_unit_get_roll_pitch_yaw(imu);
  pitch = angles(2); % Pitch is the second element (in radians)

  % Check if robot has fallen
  if abs(pitch) > 1.2
    state = 'standup';
  end
  
  switch state
    case 'standup'
      if pitch > 1.2  % Adjust threshold as needed
        wbu_motion_play(standup_front_motion);
        isMoving = true;
        currentMotion = standup_front_motion;
      elseif pitch < -1.2
        wbu_motion_play(standup_back_motion);
        isMoving = true;
        currentMotion = standup_back_motion;
      end
      state = 'orientation_recovery';
    case 'orientation_recovery'
      yaw = angles(3);
      turn_direction = computeTurnDirection(yaw, team_type, pi/20);

      if strcmp(turn_direction,'stop') 
        wbu_motion_stop(currentMotion);
        state = 'scan';
      elseif (wbu_motion_is_over(currentMotion) == true)
          if strcmp(turn_direction,'right') 
            wbu_motion_set_reverse(turn_l_180_motion, true)
            wbu_motion_play(turn_l_180_motion);
            currentMotion = turn_l_180_motion;
          elseif strcmp(turn_direction,'left') 
            wbu_motion_set_reverse(turn_l_180_motion, false)
            wbu_motion_play(turn_l_180_motion);
            currentMotion = turn_l_180_motion;
          end
      end
    case 'idle'
      key = wb_keyboard_get_key();
      wb_console_print(sprintf('Key: %i\n', key), WB_STDOUT);
      if (key == 4) % Enter
        state =  'scan';
      end
    case 'scan'
      if(ball_detected_top)
        state = 'track_mode_top';
      elseif(ball_detected_bottom)
        state = 'track_mode_bottom';
      else
        headYawDirection = scanBallHead(headYawMotor, headYawSensor, headYawDirection);
      end
    case 'track_mode_top'
      yDistThreshold = 0.025;
      headYawPosition = trackBallHead(headYawMotor, headYawSensor, angle_top);
      if(ball_detected_top)
        distance = getDistanceYAxis(distance_top, camera_top_angle, deg2rad(angle_top), headYawPosition);
      else
        state = 'scan';
        continue;
      end
      if (distance > yDistThreshold)
        wbu_motion_play(left_step_motion);
        currentMotion = left_step_motion;
        isMoving = true;
      elseif(distance < -yDistThreshold)
        wbu_motion_play(right_step_motion);
        currentMotion = right_step_motion;
        isMoving = true;
      end
      wb_console_print(sprintf('Distance Y: %s\n', distance), WB_STDOUT);
      wb_console_print(sprintf('Yaw: %s\n', headYawPosition), WB_STDOUT);
    case 'track_mode_bottom'
      angleThreshold = 0.1;
      if(~ball_detected_bottom)
        state = 'scan';
        continue;
      end
      headYawPosition = trackBallHead(headYawMotor, headYawSensor, angle_bottom);
      if ((headYawPosition + deg2rad(angle_bottom)) > angleThreshold)
        wbu_motion_play(left_step_motion);
        currentMotion = left_step_motion;
        isMoving = true;
      elseif((headYawPosition + deg2rad(angle_bottom)) < -angleThreshold)
        wbu_motion_play(right_step_motion);
        currentMotion = right_step_motion;
        isMoving = true;
      elseif(distance_bottom < 0.5)
        state = 'approach';
        continue;
      end
      wb_console_print(sprintf('Angle Y: %s\n', angle_bottom), WB_STDOUT);
      wb_console_print(sprintf('Yaw: %s\n', headYawPosition), WB_STDOUT);
    case 'approach'
      if (approach_counter == 10)
        approach_counter = 0;
        state = 'backwards_walk';
        continue;
      end
      wbu_motion_play(forwards_motion);
      isMoving = true;
      currentMotion = forwards_motion;
      approach_counter = approach_counter + 1;
      trackBallHead(headYawMotor, headYawSensor, angle_bottom);

    case 'backwards_walk'
      if (backwards_counter == 10)
        backwards_counter = 0;
        state = 'scan';
        continue;
      end
      wbu_motion_play(backwards_motion);
      isMoving = true;
      currentMotion = backwards_motion;
      backwards_counter = backwards_counter + 1;
      trackBallHead(headYawMotor, headYawSensor, angle_bottom);


  end
  wb_console_print(sprintf('GoalKeeper state: %s\n', state), WB_STDOUT);
  wb_console_print(sprintf('Approach count: %i\n', approach_counter), WB_STDOUT);
  wb_console_print(sprintf('Backwards count: %i\n', backwards_counter), WB_STDOUT);
  % ensure graphical outputs are rendered in real time
  loop_counter = loop_counter + 1;

end
end
