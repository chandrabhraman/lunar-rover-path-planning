%%
function PathPoints = map2path( obstacle, xTarget, yTarget, xStart, yStart, azimuth, elevation, MAX_X, MAX_Y)
%UNTITLED3 Summary of this function goes here
%   based on potential simple exponential potential field in path planning.

%obstacle contain 1.coordinate_X 2. coordinate_Y 3.Height 4. radius(width)
%creating positive potential field near obstacle.

%These function parameters are for testing. Original parameters would be
%obtained from the camera by stereo imaging the obstacles and plotting
%them. This function performs path planning based on the obstacles defined
%by stereo imaging, a predefined target and starting point.
fieldMap = zeros(100,100);
xTarget = 21;
yTarget = 99;
xStart = 70;
yStart = 10;
azimuth = 45;
elevation = 45;
obstacle = [10,10,54,39; 21,21,37,88; 57,21,89,100; 91,43,45,100; 53,21, 84,32];

%GENERAATING FEILD MAP BASED ON THE DERIVED INFORMATION FROM CAMAERA.
sz = size(fieldMap);
fieldMap = zeros(sz);
targetAperture = min(sz(:,1),sz(:,2)); %define the thickness of exponential function of target.
%should be of the order of smaller dimension of map.
MAX_X = sz(:,1);
MAX_Y = sz(:,2);

%Adding obstacle feild.
for a=1:size(obstacle)
    for b=1:sz(:,1)
        for c=1:sz(:,2)
            y = b;
            x = c;
            fieldMap(b,c) = fieldMap(b,c) + exp(-abs((x-obstacle(a,1)).^2 + (y-obstacle(a,2)).^2 - obstacle(a,4))*1/obstacle(a,3)); %obstacle
        end
    end
end

%adding target field
for b = 1:sz(:,1)
    for c=1:sz(:,2)
        y = b;
        x = c;
        fieldMap(b,c) = fieldMap(b,c) - exp(-abs((x-xTarget).^2 + (y-yTarget).^2)*1/targetAperture);
    end
end

%visualizing fieldMap
x = 1:sz(:,1);
y = 1:sz(:,2);
grid on;
surf(x,y,fieldMap(x,y));
hold on;

%generating waypoints to target based on field map based fmincon solver.
numWayPoints = 20;
xWayPoints = linspace(xStart,xTarget,numWayPoints+2)';
yWayPoints = linspace(yStart,yTarget,numWayPoints+2)';
h_wp = plot(xWayPoints,yWayPoints,'color','k','linestyle','none','marker','.','markersize',16);

% Generate a continuous path from the waypoints
PathPoints = WayPoints_To_Path([xWayPoints,yWayPoints],'linear',MAX_X,MAX_Y,201);
h_path = plot(PathPoints(:,1),PathPoints(:,2),'k','linewidth',2);

% Calculate the cost taken
StraightLineCost = solverCostLocal(PathPoints,fieldMap,azimuth,elevation,xTarget,yTarget,xStart,yStart,MAX_X, MAX_Y,'linear');
fprintf('straight line Cost: %.1f\n', StraightLineCost);

%% Find an optimal path using FMINCON
% Define Objective Function
objectiveFun = @(P) solverCostLocal(P,fieldMap,azimuth,elevation,xTarget,yTarget,xStart,yStart,MAX_X, MAX_Y,'linear');

% Set optimization options
opts = optimset('fmincon');
opts.Display = 'iter';
opts.Algorithm = 'active-set';
opts.MaxFunEvals = 10000;

% Initial Conditions
xWayPoints = linspace(xStart,xTarget,numWayPoints+2)';
yWayPoints = linspace(yStart,yTarget,numWayPoints+2)';
ic = [xWayPoints(2:end-1)'; yWayPoints(2:end-1)'];
ic = ic(:);

% Bounds
lb = ones(size(ic(:)));
ub = reshape([MAX_X*ones(1,numWayPoints); MAX_Y*ones(1,numWayPoints)],[],1);

%Do the optimizaiton
optimalWayPoints = fmincon(objectiveFun, ic(:), [],[],[],[],lb,ub,[],opts);

%% Plot the optimal solution:
delete([h_wp h_path]);
optimalWayPoints = [xStart yStart; reshape(optimalWayPoints,2,[])'; xTarget yTarget];

xWayPoints = optimalWayPoints(:,1);
yWayPoints = optimalWayPoints(:,2);
h_wp = plot(xWayPoints,yWayPoints,'color','k','linestyle','none','marker','.','markersize',16);

PathPoints = WayPoints_To_Path([xWayPoints,yWayPoints],'linear',MAX_X,MAX_Y,101);
h_path = plot(PathPoints(:,1),PathPoints(:,2),'k','linewidth',2);
LineCost = solverCostLocal(PathPoints,fieldMap,azimuth,elevation,xTarget,yTarget,xStart,yStart,MAX_X, MAX_Y,'linear');
fprintf('total Cost: %.1f\n', LineCost);
end