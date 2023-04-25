function cost = solverCost( PathPoints, map, coor_map, azimuth,xTarget, yTarget, xStart, yStart,MAX_X, MAX_Y, METHOD, fineness )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% safetylimit = tand(30); %change the safetylimit of slopes here.
% weightTopo = 1;
% weightShadow = 50; %enter weight for shadow function.
% weightDistance = 1;
slopeSafety = tand(18); %for 18 degree slopes.
frictionSafety=30; %33 degree mue is the lowest coeff of friction seen on moon.
%0 - 1 is being scaled for 0 - 360. Just calculate the allowable arc length put the
%correspong scale between 0 - 1. Currently taken as 180 degree.
shadowSafety=0.5;

if isvector(PathPoints)
    PathPoints = [xStart yStart; reshape(PathPoints,2,[])'; xTarget yTarget];
    PathPoints = WayPoints_To_Path(PathPoints,METHOD,MAX_X,MAX_Y,fineness);
end

% Interpolate the topography at all the points in PathPoints.
topo = [interp2(map,PathPoints(1:end,1),PathPoints(1:end,2),'linear')];
coor = [interp1(coor_map(:,1),PathPoints(1:end,1),'linear'), interp1(coor_map(:,2),PathPoints(1:end,2),'linear')];

height = 400; %enter the height of the vehicle.
width  = 500; %enter the lowest width of vehicle. alternatively length is 600 mm.
length = diff(topo);  %in m.

dist_hav = haversineArr(coor); %in m
X = abs(length./dist_hav);  

%%assigning the bounday conditions.
for a=1:size(X)
    if X(a,1)>slopeSafety || atand(X(a,1))>frictionSafety;
       X(a,1)=exp(15*(1+X(a,1)/slopeSafety)); %taking a percentage with maximum slope limit
    else
       X(a,1) = 0;
    end
    if isnan(X(a,1)) || isinf(X(a,1))
        X(a,1) = exp(15); %since undefined therefore assgnging a high cost.
    end
end


ratio = diff(PathPoints);
angle = atan2d(ratio(:,2), ratio(:,1));
sz = size(angle);
for a=1:sz(:,1)
    if angle(a,:)<0
        angle(a,:) = angle(a,:)+360;
    end
    phase_shift(a,:) = angle(a,:)-90;
    if phase_shift(a,:)<0
        phase_shift(a,:) = phase_shift(a,:)+360;
    end
end

if 0<=azimuth && azimuth<180
    for a=1:sz(:,1)
        if (180+azimuth)<=phase_shift(a,:) && phase_shift(a,:)<360
            phase_shift(a,:) = phase_shift(a,:)-360;
        end
    end
elseif 180<=azimuth && azimuth<360
    for a=1:sz(:,1)
        if 0 <= phase_shift(a,:) && phase_shift(a,:) < azimuth-180
            phase_shift(a,:) = phase_shift(a,:)+360;
        end
    end
end
for a=1:sz(:,1)
    if (azimuth-180) <= phase_shift(a,:) && phase_shift(a,:) < azimuth
        Y(a,:) = abs( (phase_shift(a,:)-azimuth+180)/180);
        %Applying the safety threshold for considering the boundary
        %condition.
        if Y(a,:)>shadowSafety
            Y(a,:)=exp(15*(1+(Y(a,:)))); %selecting a numerical value analogous to infinity cost.
            %Y(a,:)=exp(15);
        else
            Y(a,:)=0;
        end
    elseif azimuth <= phase_shift(a,:) && phase_shift(a,:) <= azimuth+180
        Y(a,:) = abs( (phase_shift(a,:)-azimuth-180)/(-180));
        %Applying the safety threshold for considering the boundary
        %condition.
        if Y(a,:)>shadowSafety
            Y(a,:)=exp(15*(1+(Y(a,:)))); %selecting a numerical value analogous to infinity cost.
            %Y(a,:)=exp(15);
        else
            Y(a,:)=0;
        end
    end
    
end



%finding distance (Euclidian)
Z = sqrt( (PathPoints(2:end,1)-PathPoints(1:end-1,1)).^2 + (PathPoints(2:end,2)-PathPoints(1:end-1,2)).^2 );
absolute = sqrt( (PathPoints(end,1)-PathPoints(1,1)).^2 + (PathPoints(end,2)-PathPoints(1,2)).^2 );

%slope is X, sun deviation is Y, distance is Z.

C1 = X;                           %slope
C2 = Y;                                %sun deviation
C3 = ((sum(Z)/absolute) - 1);                %distance

slopeCostSum = sum(C1);
deviationCostSum = sum(C2);
distanceCostSum = sum(Z);
cost = (slopeCostSum + deviationCostSum)/fineness + C3;
%fprintf('total Cost: %.1f  slope cost: %.1f sun deviation cost: %.1f distance cost: %.1f\n', cost, slopeCostSum, deviationCostSum, distanceCostSum);
end