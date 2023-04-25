function cost = solverCostLocal( PathPoints, fieldmap, azimuth, elevation, xTarget, yTarget, xStart, yStart, MAX_X, MAX_Y, METHOD)
%UNTITLED Summary of this function goes here
%%%   Detailed explanation goes here
pan=20;
if isvector(PathPoints)
    PathPoints = [xStart yStart; reshape(PathPoints,2,[])'; xTarget yTarget];
    PathPoints = WayPoints_To_Path(PathPoints,METHOD,MAX_X,MAX_Y,101);
end

% Interpolate the potential at all the points in PathPoints.
potField = [interp2(fieldmap,PathPoints(1:end,1),PathPoints(1:end,2),'cubic')];
X = potField;


eff = 1/25;
% cost due to mast's shadow
ratio = [diff(PathPoints(1:end,1)), diff(PathPoints(1:end,2))];
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
        if (180+azimuth)<phase_shift(a,:) && phase_shift(a,:)<360
            phase_shift(a,:) = phase_shift(a,:)-360;
        end
    end
elseif 180<=azimuth && azimuth<360
    for a=1:sz(:,1)
        if 0 < phase_shift(a,:) && phase_shift(a,:) < azimuth-180
            phase_shift(a,:) = phase_shift(a,:)+360;
        end
    end
end


for a=1:sz(:,1)
    
%     t = abs(azimuth-phase_shift(a,:));
%     %considering only smaller of two angles
%     if t>180
%         t = 360-t;
%     end
%     if t<90
%         eff = efficiencyPanel(azimuth,elevation,pan,phase_shift(a,:));
%     else
%         eff = 1-1/25;
%     end
    
    eff = 1-1/25;
    if (azimuth-180) <= phase_shift(a,:) && phase_shift(a,:) < azimuth
        Y(a,:) = abs( (phase_shift(a,:)-azimuth+180)/180*50*(1-eff));
    elseif azimuth <= phase_shift(a,:) && phase_shift(a,:) <= azimuth+180
        Y(a,:) = abs( (phase_shift(a,:)-azimuth-180)/(-180)*50*(1-eff));
    end
end


% cost distance
Z = sqrt( (PathPoints(2:end,1)-PathPoints(1:end-1,1)).^2 + (PathPoints(2:end,2)-PathPoints(1:end-1,2)).^2 );
absolute = sqrt( (PathPoints(end,1)-PathPoints(1,1)).^2 + (PathPoints(end,2)-PathPoints(1,2)).^2 );

C1 = X;                                     %slope
C2 = Y;                                %sun deviation.
C3 = (sum(Z)/absolute) - 1;                 %distance

obstacleCostSum = sum(C1);
shadowCostSum = sum(C2);
distanceCostSum = sum(Z);
cost = (obstacleCostSum + shadowCostSum)/101 + C3; %101 is the total path points interpolated between 15 points.

%fprintf('total Cost: %.1f  slope cost: %.1f sun deviation cost: %.1f distance cost: %.1f\n', cost, slopeCostSum, deviationCostSum, distanceCostSum);
end