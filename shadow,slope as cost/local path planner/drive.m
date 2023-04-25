function traversed = drive( pathpoints, fieldMap )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
safetyLimit = 30;
accnGravity = 1.62519;
orientation = IMUFeedBack();

%orientation 1.pitch 2.roll 3.z'accn
costPitch = orientation(:,1)/30;
costRoll = orientation(:,2)/30;
costRough = orientation(:,3)/accnGravity;

%X Y distance traversed.
%rough estimate thorugh 1.PWM cycles*efficiency 2.Pose estimator through
%IMU 3.efficiency estimation through Torque 4. 


%Distance left to the target.
Z = sqrt( (PathPoints(2:end,1)-PathPoints(1:end-1,1)).^2 + (PathPoints(2:end,2)-PathPoints(1:end-1,2)).^2 );
absolute = sqrt( (PathPoints(end,1)-PathPoints(1,1)).^2 + (PathPoints(end,2)-PathPoints(1,2)).^2 );

Distance = Z./absolute;

end