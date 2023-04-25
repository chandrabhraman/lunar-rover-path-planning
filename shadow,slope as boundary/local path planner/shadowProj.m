function [ X,Y ] = shadowProj( x,y,z,elevation,diffan )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    
    %assuming azimuth-elevation diffanerence is positive. Therefore considering the sign
    %convention of angle, projection is done on only one half of the
    %system.
    if diffan>0
        X = x+z/tan(elevation)*cos(diffan);
        Y = y-z/tan(elevation)*sin(diffan);
    else
        X = x-z/tan(elevation)*cos(diffan);
        Y = y+z/tan(elevation)*sin(diffan);
    end
end