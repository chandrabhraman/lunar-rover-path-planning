function distance = haversine( long2, long1, lat2, lat1 )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    radius = 1737.4e+03; %mean radius of moon in m.
    distance = 2*radius*asin( sqrt(  (sind((lat2 - lat1)/2)).^2 + cosd(lat1)*cosd(lat2)*(sind((long2 - long1)/2)).^2  ));
end