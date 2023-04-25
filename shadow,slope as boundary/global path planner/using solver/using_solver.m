%% 
R1 = 0;
C1 = 0;
C2 = 2;
R2 = 9999;    
azimuth = 45;   %azimuth
elevation = 30; %elevation
unit_block = 100; %unit block in cms
fineness = 51; %number of interpolated points between waypoints.
    %% 
while(R2<12435501)
    %% 
    
    %reading a 1000 values at a time.
    map_triplet = csvread('E:\team indus\2016\dem.csv',R1,C1,[R1 C1 R2 C2]);
    num = size(map_triplet);
    R1 = R2;
    R2 = R2 + num(:,1);
    
    %for converting to easting, northing coordinates.
    for a = 1:num(:,1)
        map_triplet(a,1) = 360 + map_triplet(a,1);
    end
    
    F = scatteredInterpolant(map_triplet(:,1), map_triplet(:,2), map_triplet(:,3));
    max_Long = max(map_triplet(:,1));
    min_Long = min(map_triplet(:,1));
    max_Lat = max(map_triplet(:,2));
    min_Lat = min(map_triplet(:,2));
    %[xq, yq] = meshgrid(min_Long:((max_Long-min_Long)/haversine(max_Long, min_Long, min_Lat, min_Lat)*unit_block):max_Long, min_Lat:((max_Lat-min_Lat)/haversine(max_Long, min_Long, min_Lat, min_Lat)*unit_block):max_Lat);

    x = min_Long:((max_Long-min_Long)/haversine(max_Long, min_Long, min_Lat, min_Lat)*unit_block):max_Long;
    y = min_Lat:((max_Lat-min_Lat)/haversine(max_Long, min_Long, min_Lat, min_Lat)*unit_block):max_Lat;
    
    %%
    [~, MAX_X] = size(x);
    [~, MAX_Y] = size(y);
    %x = reshape(xq, MAX_X*MAX_Y, 1);
    %y = reshape(yq, MAX_X*MAX_Y, 1);

    %creating topography map in accordance with coordinate map.
    map = ones(MAX_X, MAX_Y);
    for a = 1:MAX_X
        for b = 1:MAX_Y
            map(a,b) = F(x(1,a), y(1,b));
            coor_map(a,1) = x(1,a);
            coor_map(b,2) = y(1,b);
        end
    end
    
    %%
    grid on;
    hold on;
    axis equal tight
    pcolor(map')
    %begin interactive start and target
    pause(1);
    h=msgbox('Please Select the Target using the Left Mouse button');
    if ishandle(h) == 1
        delete(h);
    end
    
    xlabel('Please Select the Target using the Left Mouse button','Color','black');
    but=0;
    while (but ~= 1) %Repeat until the Left button is not clicked
        [xval,yval,but]=ginput(1);
    end

    %xval=floor(xval);
    %yval=floor(yval);
    xTarget=xval;%X Coordinate of the Target
    yTarget=yval;%Y Coordinate of the Target
    
    %not initializing map with target location cause map should be untouched.
    
    plot(xval,yval,'gd');
    %%why, 0.5?
    text(xval+1,yval+.5,'Target')

    %% vehicle's initial position
    pause(1);
    h=msgbox('Please Select the vehicle initial position using the Left Mouse button');
    uiwait(h,5);
    if ishandle(h) == 1
        delete(h);
    end
    xlabel('Please Select the Vehicle initial position ','Color','black');
    but=0;
    while (but ~= 1) %Repeat until the Left button is not clicked
        [xval,yval,but]=ginput(1);
        %xval=floor(xval);
        %yval=floor(yval);
    end
    xStart=xval;%Starting Position
    yStart=yval;%Starting Position
    plot(xval,yval,'bo');
    text(xval+1,yval+.5,'starting point');
    
    %%
    numWayPoints = 15;
    xWayPoints = linspace(xStart,xTarget,numWayPoints+2)';
    yWayPoints = linspace(yStart,yTarget,numWayPoints+2)';
    
    h_wp = plot(xWayPoints,yWayPoints,'color','k','linestyle','none','marker','.','markersize',16);

    %% Generate a continuous path from the waypoints
    PathPoints = WayPoints_To_Path([xWayPoints,yWayPoints],'linear',MAX_X,MAX_Y,fineness);
    h_path = plot(PathPoints(:,1),PathPoints(:,2),'k','linewidth',2);
    
    %% Calculate the cost taken
    StraightLineCost = solverCost( PathPoints, map, coor_map, azimuth, xTarget, yTarget, xStart, yStart, MAX_X, MAX_Y, 'linear', fineness);
    
    %% Find an optimal path using FMINCON
    % Define Objective Function
    objectiveFun = @(P) solverCost(P,map,coor_map,azimuth,xTarget, yTarget, xStart, yStart,MAX_X, MAX_Y, 'linear', fineness);
    
    % Set optimization options
    opts = optimset('fmincon');
    opts.Display = 'iter';
    opts.Algorithm = 'active-set';
    opts.MaxFunEvals = 20000;
    
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

    PathPoints = WayPoints_To_Path([xWayPoints,yWayPoints],'cubic',MAX_X,MAX_Y,fineness);
    h_path = plot(PathPoints(:,1),PathPoints(:,2),'k','linewidth',2);
    LineCost = solverCost(PathPoints, map, coor_map, azimuth,xTarget, yTarget, xStart, yStart,MAX_X, MAX_Y, 'linear', fineness);
    fprintf('total Cost: %.1f\n', LineCost);
    pause(2);
end