function distance = haversineArr( coor)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

radius = 1737.4e+03; %mean radius of moon in m.
dlong = diff(coor(:,1));
dlat = diff(coor(:,2));
distance = 2*radius*asin( sqrt( sind(dlat./2).^2+cosd(coor(1:end-1,2)).*cosd(coor(2:end,2)).*(sind(dlong./2)).^2  ));


end

