%local path planner for cute rover
%based on camera, IMU and PWM controlled motors, hall sensors are absent.

%grids and graphs here.

map = [];
coor_map = [];
currentNode = [];
targetNode = [];

[obstacle, MAX_X, MAX_Y] = camFeedback();
PathPoints = map2path(obstacles, xTarget, yTarget, azimuth, elevation, pan, MAX_X, MAX_Y);
traversed = drive(PathPoints);
