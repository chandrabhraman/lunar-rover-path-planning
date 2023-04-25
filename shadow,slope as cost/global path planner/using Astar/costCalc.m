function X = costCalc( openNode, currentNode, targetNode, coor_map, map, azimuth)
%All the costs including distances.
%Detailed explanation goes here.
%Using haversine function for calculating
%distance between 2 points on a sphere.

height = 400; %enter the height of the vehicle.
width  = 500; %enter the lowest width of vehicle.
length =  map(openNode(:,1), openNode(:,2)) - map(currentNode(:,1),currentNode(:,2))  ; %in m.
if length ~=0 %to prevent logical errors due to finer interpolation.
    dist_hav = haversine( coor_map(openNode(:,1),1), coor_map(currentNode(:,1),1), coor_map(openNode(:,2),2), coor_map(currentNode(:,2),2) ); %in m
    X(1) = abs(length/dist_hav);
else
    X(1) = 0;
end
%33 degrees being the lowest coeffient of friction of regolith.
if X(1)>width/height || atand(X(1))>30
    X(1)=inf;
end

%slope thing
ratio = [(openNode(:,1)-currentNode(:,1)), (openNode(:,2)-currentNode(:,2)) ];
angle = atan2d(ratio(:,2), ratio(:,1));
if angle<0
    angle = angle+360;
end
phase_shift = angle-90;
if phase_shift<0
    phase_shift = phase_shift + 360;
end

%Restricted azi condition.
if 0<=azimuth && azimuth<180 && (180+azimuth)<phase_shift && phase_shift<360
    phase_shift = phase_shift-360;
elseif 180<=azimuth && azimuth<360 && 0 < phase_shift && phase_shift < azimuth-180
    phase_shift = phase_shift+360;
end


if (azimuth-180) <= phase_shift && phase_shift < azimuth
    X(2) = abs((phase_shift-azimuth+180)/180*2);
elseif azimuth <= phase_shift && phase_shift <= azimuth+180
    X(2) = abs((phase_shift-azimuth-180)/(-180)*2);
end

%finding distance
disxy = [ abs(targetNode(:,1)-openNode(:,1)), abs(targetNode(:,2)-openNode(:,2)) ];
X(3) = 1.414213562373095*min(disxy(:,1),disxy(:,2)) + abs(disxy(:,1)-disxy(:,2));
end