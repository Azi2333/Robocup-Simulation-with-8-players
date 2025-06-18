function forward_defender
% set time step for each simulation step in milliseconds
TIME_STEP = 64;
%Set team 
team_type = '';
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
steps_remaining = 0;  % 初始化步数计数变量
headYawMotor = wb_robot_get_device('HeadYaw');
headPitchMotor = wb_robot_get_device('HeadPitch');
wb_motor_set_position(headPitchMotor, headYawOffset); % 25 degrees - max positive angle

%回城初始化
steps_left = 0;
steps_right = 0;
steps_forward = 0;
steps_backward = 0;
ball_lost_timer = 0;          % 初始化球丢失计时器，初始值为 0
ball_lost_threshold = 20000;  % 设置球丢失的时间阈值（单位毫秒，10 秒
isReturning = false;  % 是否处于归位状态的标志


% Arms Position
LShoulderPitch = wb_robot_get_device('LShoulderPitch');
RShoulderPitch = wb_robot_get_device('RShoulderPitch');
LElbowRoll = wb_robot_get_device('LElbowRoll');
RElbowRoll = wb_robot_get_device('RElbowRoll');
wb_motor_set_position(LShoulderPitch, pi/2)
wb_motor_set_position(RShoulderPitch, pi/2)
% Sensors Initilisation
headYawSensor  = wb_robot_get_device('HeadYawS');
wb_position_sensor_enable(headYawSensor, TIME_STEP)
wb_keyboard_enable(TIME_STEP)

% Set GPS
gps = wb_robot_get_device('GPS');
wb_gps_enable(gps, 2000);
position = wb_gps_get_values(gps);
x_pos = position(1); % 机器人 X 轴位置
y_pos = position(2); % Y 轴方向（通常是高度）
z_pos = position(3); % 机器人前后方向位置
%wb_console_print(sprintf('Current Position: X=%.2f, Y=%.2f, Z=%.2f', x_pos, y_pos, z_pos), WB_STDOUT);

% basic motions
forwards_motion = wbu_motion_new('../../motions/Forwards.motion');
backwards_motion = wbu_motion_new('../../motions/Backwards.motion');
turn_l_180_motion = wbu_motion_new('../../motions/TurnLeft180.motion');
shoot_motion = wbu_motion_new('../../motions/Shoot.motion');
left_step_motion = wbu_motion_new('../../motions/SideStepLeft.motion');
right_step_motion = wbu_motion_new('../../motions/SideStepRight.motion');
standup_front_motion = wbu_motion_new('../../motions/StandUpFromFront.motion');
standup_back_motion = wbu_motion_new('../../motions/StandUpFromBack.motion');

% Get IMU handle
imu = wb_robot_get_device('IMU');
wb_inertial_unit_enable(imu, TIME_STEP);

% Variables initialisation
% headYawDirection = 1; %Positive direction
%headYawDirection = -1; % Negative direction
headYawDirection = 1; %Positive direction
lastHeadYawDirection = 1; %Positive direction
changeHeadDirCounter = 0;
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
  confidenceThreshold = 0.15;
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
      %wb_console_print(sprintf('Key: %i\n', key), WB_STDOUT);
      if (key == 4) % Enter
        state =  'scan';
      end

    case 'scan'
      wb_motor_set_position(LElbowRoll, 0);
      wb_motor_set_position(RElbowRoll, 0);
      wb_motor_set_position(headPitchMotor, 0.5);

      if (ball_detected_bottom)
        state = 'track_mode_bottom';  % 检测到球，切换到底部跟踪模式

      elseif (ball_detected_top)
        state = 'track_mode_top';  % 检测到球，切换到顶部跟踪模式
      else
        headYawDirection = scanBallHead(headYawMotor, headYawSensor, headYawDirection);
        if (lastHeadYawDirection ~= headYawDirection)
          changeHeadDirCounter = changeHeadDirCounter + 1;
          lastHeadYawDirection = headYawDirection;
        end
        if (changeHeadDirCounter >= 2)
          state = 'backwards_walk';
          changeHeadDirCounter = 0;
        end
      end






    case 'track_mode_top'
      yDistThreshold = 0.03;   % 对齐的水平角度阈值
      distanceThreshold = 1.45; % 距离阈值，单位：米
      headYawPosition = trackBallHead(headYawMotor, headYawSensor, angle_top);
      distance = 0;

      % 获取机器人当前位置
      position = wb_gps_get_values(gps);
      x_pos = position(1); % 机器人 X 轴位置（前进方向）

      % 如果顶部摄像头检测到球，计算距离和方向
      if (ball_detected_top)
        distance = getDistanceYAxis(distance_top, camera_top_angle, deg2rad(angle_top), headYawPosition);
      else
        state = 'scan';  % 未检测到球，切换到扫描状态
        continue;
      end

      % ⚠️ **新的逻辑：只限制前进，不限制左右移动**
      if (x_pos < 1.1)
        % 如果球在左侧，机器人仍然可以侧移对齐
        if (distance > yDistThreshold)
          wbu_motion_play(left_step_motion);
          currentMotion = left_step_motion;
          isMoving = true;
          steps_left = steps_left + 1;

          % 如果球在右侧，机器人仍然可以侧移对齐
        elseif (distance < -yDistThreshold)
          wbu_motion_play(right_step_motion);
          currentMotion = right_step_motion;
          isMoving = true;
          steps_right = steps_right + 1;

        else
          % 机器人已经在1.9的边界，且对齐完成，才回到 scan
          state = 'scan';
        end
        continue;
      end

      % **如果没有超出边界，则继续正常的前进逻辑**
      if (distance_top > distanceThreshold)
        wbu_motion_play(forwards_motion);
        currentMotion = forwards_motion;
        isMoving = true;
      else
        if (distance > yDistThreshold)
          wbu_motion_play(left_step_motion);
          currentMotion = left_step_motion;
          isMoving = true;
          steps_left = steps_left + 1;
        elseif (distance < -yDistThreshold)
          wbu_motion_play(right_step_motion);
          currentMotion = right_step_motion;
          isMoving = true;
          steps_right = steps_right + 1;
        else
          wbu_motion_play(forwards_motion);
          currentMotion = forwards_motion;
          isMoving = true;

          if (ball_detected_bottom)
            state = 'track_mode_bottom';
            ball_lost_timer = 0;
          end

          if (distance_top < 1)
            state = 'approach';
            continue;
          end
        end






        % 检查是否进入 CameraBottom 视野
        if (ball_detected_bottom)
          state = 'track_mode_bottom';  % 切换到底部摄像头模式
          ball_lost_timer = 0;          % 重置计时器
        end
      end
      %wb_console_print(sprintf('Distance Y: %s\n', distance), WB_STDOUT);
      %wb_console_print(sprintf('Yaw: %s\n', headYawPosition), WB_STDOUT);

    case 'track_mode_bottom'
      angleThreshold = 0.085;
      if(~ball_detected_bottom)
        state = 'track_mode_top';
        continue;
      end
      headYawPosition = trackBallHead(headYawMotor, headYawSensor, angle_bottom);
      if ((headYawPosition + deg2rad(angle_bottom)) > angleThreshold)
        wbu_motion_play(left_step_motion);
        currentMotion = left_step_motion;
        isMoving = true;
        steps_left = steps_left + 1;  % 左移步数计数

      elseif((headYawPosition + deg2rad(angle_bottom)) < -angleThreshold)
        wbu_motion_play(right_step_motion);
        currentMotion = right_step_motion;
        isMoving = true;
        steps_right = steps_right + 1;  % 右移计数

      elseif(distance_bottom < 5)
        state = 'approach';
        continue;
      end

      %wb_console_print(sprintf('Angle Y: %s\n', angle_bottom), WB_STDOUT);
      %wb_console_print(sprintf('Yaw: %s\n', headYawPosition), WB_STDOUT);
      %wb_console_print(sprintf('Distance to ball: %.2f m\n', distance_bottom), WB_STDOUT);

    case 'approach'
      if ~(ball_detected_top || ball_detected_bottom)
        state = 'scan';
        continue;
      end
      % 获取机器人当前位置
      position = wb_gps_get_values(gps);
      x_pos = position(1); % 机器人 X 轴位置（前进方向）

      % **前进位置限制：如果 x_pos < 1.9，就停止前进**
      if x_pos < 1.1
        state = 'scan'; % 进入扫描模式，重新寻找球
        continue;
      end

      ball_offset = deg2rad(angle_bottom) + headYawPosition;
      angleThreshold = 0.2;
      if abs(ball_offset) > angleThreshold
        state = 'scan';
        continue;
      end

      if (distance_bottom < 0.365)
        state = 'shoot';
        wbu_motion_play(shoot_motion);
        currentMotion = shoot_motion;
        isMoving = true;
        continue;
      end

      wbu_motion_play(forwards_motion);
      isMoving = true;
      currentMotion = forwards_motion;
      steps_forward = steps_forward + 1;
      headYawPosition = trackBallHead(headYawMotor, headYawSensor, angle_bottom);

    case 'shoot'  % <== 新增
      wbu_motion_play(shoot_motion);
      currentMotion = shoot_motion;
      isMoving = true;

      while ~wbu_motion_is_over(currentMotion)
        wb_robot_step(TIME_STEP);
      end

      state = 'scan';  % <== 确保射门后回到 `scan` 继续寻找球

    case 'backwards_walk'
      % 获取机器人当前位置
      position = wb_gps_get_values(gps);
      x_pos = position(1); % 获取Z方向坐标（前后）

      % 如果检测到球，停止后退，回到 `scan`
      if (ball_detected_top || ball_detected_bottom)
        state = 'scan';
        continue;
      end

      % 检查是否超过底线（-3.7），如果超过则停止后退，回到 `scan`
      if x_pos > 1.8
        state = 'scan'; % 到达底线，停止后退
        continue;
      end

      % 仍然可以后退
      wbu_motion_play(backwards_motion);
      isMoving = true;
      currentMotion = backwards_motion;
      steps_backward = steps_backward + 1;
      headYawPosition = trackBallHead(headYawMotor, headYawSensor, angle_bottom);
  end
end