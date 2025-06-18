% MATLAB controller for Webots: basic_controller.m
function striker_fixed_goal

% set time step for each simulation step in milliseconds
TIME_STEP = 64;

% get and enable devices
headYawOffset= deg2rad(25);
camera_top = wb_robot_get_device('CameraTop');
camera_bottom = wb_robot_get_device('CameraBottom');
wb_camera_enable(camera_top, TIME_STEP);
wb_camera_enable(camera_bottom, TIME_STEP);
camera_top_angle = headYawOffset + deg2rad(1.2);
camera_bottom_angle = headYawOffset + deg2rad(39.7);

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
% Arms Position
LShoulderPitch = wb_robot_get_device('LShoulderPitch');
RShoulderPitch = wb_robot_get_device('RShoulderPitch');
LElbowRoll = wb_robot_get_device('LElbowRoll');
RElbowRoll = wb_robot_get_device('RElbowRoll');
wb_motor_set_position(LShoulderPitch, pi/2);
wb_motor_set_position(RShoulderPitch, pi/2);
wb_motor_set_position(LElbowRoll, 0);
wb_motor_set_position(RElbowRoll, 0);

% Sensors Initilisation
headYawSensor  = wb_robot_get_device('HeadYawS');
wb_position_sensor_enable(headYawSensor, TIME_STEP)
wb_keyboard_enable(TIME_STEP)

% Set inertial unit
inertialUnit = wb_robot_get_device('IMU');
wb_inertial_unit_enable(inertialUnit, 1000);

% Set GPS
gps = wb_robot_get_device('GPS');
wb_gps_enable(gps, 1000);


% basic motions
forwards_motion = wbu_motion_new('../../motions/Forwards50.motion');
backwards_motion = wbu_motion_new('../../motions/Backwards.motion');
turn_l_40_motion = wbu_motion_new('../../motions/TurnLeft40.motion');
turn_l_60_motion = wbu_motion_new('../../motions/TurnLeft60.motion');
turn_r_40_motion = wbu_motion_new('../../motions/TurnRight40.motion');
turn_r_60_motion = wbu_motion_new('../../motions/TurnRight60.motion');
shoot_motion = wbu_motion_new('../../motions/Shoot.motion');
left_step_motion = wbu_motion_new('../../motions/SideStepLeft.motion');
right_step_motion = wbu_motion_new('../../motions/SideStepRight.motion');
standup_front_motion = wbu_motion_new('../../motions/StandUpFromFront.motion');
standup_back_motion = wbu_motion_new('../../motions/StandUpFromBack.motion');

% Get IMU handle
imu = wb_robot_get_device('IMU');
wb_inertial_unit_enable(imu, TIME_STEP);

% Variables initialisation
headYawDirection = 1; %Positive direction
lastHeadYawDirection = 1; %Positive direction
changeHeadDirCounter = 0;
% headYawDirection = -1; % Negative direction
state =  'idle';

% initialize YOLOv4 model
model = yolov4ObjectDetector('tiny-yolov4-coco');
loop_counter = 0;

% Goal position
opponent_goal_position = [5,0];
%opponent_goal_position = [-5,0];

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
  angle_goal =  NaN;

  % perform object detection using the specified threshold
  confidenceThreshold = 0.4;
  [bboxes_top, scores_top, labels_top] = detect(model, rgb_top, 'Threshold', confidenceThreshold);
  [bboxes_bottom, scores_bottom, labels_bottom] = detect(model, rgb_bottom, 'Threshold', confidenceThreshold);

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
  imshow(rgb_top);
  title('CameraTop');
  text(10, 10, sprintf('Angle: %.1f°  Distance: %.2f m', angle_top, distance_top), ...
    'Units', 'normalized', 'Position', [0.5, -0.2], 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'k');

  % display camera bottom image
  subplot(2, 2, 2);
  imshow(rgb_bottom);
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
  pitch = angles(2) % Pitch is the second element (in radians)

  % Check if robot has fallen
  if pitch > 1.2  % Adjust threshold as needed
    wbu_motion_play(standup_front_motion);
    isMoving = true;
    currentMotion = standup_front_motion;
  elseif pitch < -1.2
    wbu_motion_play(standup_back_motion);
    isMoving = true;
    currentMotion = standup_back_motion;
  end

  switch state
    case 'idle'
      key = wb_keyboard_get_key();
      %wb_console_print(sprintf('Key: %i\n', key), WB_STDOUT);
      if (key == 4) % Enter
        state =  'scan';
      end
    case 'scan'
      wb_motor_set_position(LElbowRoll, 0);
      wb_motor_set_position(RElbowRoll, 0);
      if(ball_detected_top)
        state = 'track_mode_top';
      elseif(ball_detected_bottom)
        state = 'track_mode_bottom';
      else
        headYawDirection = scanBallHead(headYawMotor, headYawSensor, headYawDirection, pi/4);
        if (lastHeadYawDirection ~= headYawDirection)
          changeHeadDirCounter = changeHeadDirCounter + 1;
          lastHeadYawDirection = headYawDirection;
        end
        if (changeHeadDirCounter >= 2)
          wbu_motion_play(turn_l_60_motion);
          currentMotion = turn_l_60_motion;
          isMoving = true;
          %wb_console_print(sprintf('Turning 60\n'), WB_STDOUT);
          changeHeadDirCounter = 0;
        end
      end
    case 'track_mode_top'
      yDistThreshold = 0.025;
      headYawPosition = trackBallHead(headYawMotor, headYawSensor, angle_top);
      distance  = 0;
      if(ball_detected_top)
        distance = getDistanceYAxis(distance_top, camera_top_angle, deg2rad(angle_top), headYawPosition);
      else
        state = 'scan';
        changeHeadDirCounter = 0;
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
      else
        state = 'turn2goal';
      end
      %wb_console_print(sprintf('Distance Y: %s\n', distance), WB_STDOUT);
      %wb_console_print(sprintf('Yaw: %s\n', headYawPosition), WB_STDOUT);
    case 'track_mode_bottom'
      angleThreshold = 0.075;
      if(~ball_detected_bottom)
        state = 'scan';
        changeHeadDirCounter = 0;
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
      else
        state = 'turn2goal';
        continue;
      end
      %wb_console_print(sprintf('Angle: %s\n', deg2rad(angle_bottom)), WB_STDOUT);
      %wb_console_print(sprintf('Yaw: %s\n', headYawPosition), WB_STDOUT);
    case 'turn2goal'
      opponent_goal_position;
      angle_to_goal = getAngleToGoal(opponent_goal_position, robot_position);
      robot_orientation = wb_inertial_unit_get_roll_pitch_yaw(inertialUnit);
      if (opponent_goal_position == [5,0])
        angle_difference = robot_orientation(3) - angle_to_goal;
      else
        if (robot_orientation(3) < 0)
          angle_difference = (robot_orientation(3) + pi) - angle_to_goal;
        else
          angle_difference = (robot_orientation(3) - pi) - angle_to_goal;
        end
      end
      if ( angle_difference > deg2rad(20))
        %wb_console_print(sprintf('Turning Right\n'), WB_STDOUT);
        wbu_motion_play(turn_r_40_motion);
        currentMotion = turn_r_40_motion;
        isMoving = true;
        state = 'scan' ;
        changeHeadDirCounter = 0;
      elseif (angle_difference < deg2rad(-20))
        %wb_console_print(sprintf('Turning Left\n'), WB_STDOUT);
        wbu_motion_play(turn_l_40_motion);
        currentMotion = turn_l_40_motion;
        isMoving = true;
        state = 'scan' ;
        changeHeadDirCounter = 0;
      else
        state = 'forward';
      end
    case 'forward'
      angleThreshold = 0.075;
      if(ball_detected_bottom)
        headYawPosition = trackBallHead(headYawMotor, headYawSensor, angle_bottom);
        abs_angle_to_ball = abs(headYawPosition + deg2rad(angle_bottom));
        if (abs_angle_to_ball > angleThreshold)
          state = 'scan';
          changeHeadDirCounter = 0;
        end
      else
        state = 'scan';
        changeHeadDirCounter = 0;
      end
      distance_to_goal = distanceToGoal(opponent_goal_position, robot_position);
      if (distance_to_goal < 0.5)
        state = 'stop';
      end
      %wb_console_print(sprintf('Forward\n'), WB_STDOUT);
      wbu_motion_play(forwards_motion);
      currentMotion = forwards_motion;
      isMoving = true;
  end
  %wb_console_print(sprintf('Striker A state: %s\n', state), WB_STDOUT);
  % ensure graphical outputs are rendered in real time
  loop_counter = loop_counter + 1;
  robot_orientation = wb_inertial_unit_get_roll_pitch_yaw(inertialUnit);
  robot_position = wb_gps_get_values(gps);
end
end
