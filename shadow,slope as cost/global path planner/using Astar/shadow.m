function RESTRICTED = shadow( map, theta, phi )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%shadows are obstacles with infinite. calculating obstacles using
%topographic values.
%problem of localized depth needed to be solved.
    num_obs = 1;
    [MAX_X, MAX_Y] = size(map);
    for a = 2:MAX_X
        for b = 2:MAX_Y
            shadow_length = (map(a,b)*cotd(theta);
            if(floor(shadow_length)==0)
                continue
            end
            
            %considering a phase shift of 180 degrees.
            eq_x = a - shadow_length*cosd(phi);
            eq_y = b - shadow_length*sind(phi);
            RESTRICTED(num_obs,1) = ceil(eq_x);
            RESTRICTED(num_obs,2) = ceil(eq_y);
            num_obs = num_obs+1;

            %plot(ceil(eq_x),ceil(eq_y),'bo');
            %equation of line
            while(ceil(eq_x) ~= a)
                if eq_x<0
                    eq_x = eq_x+1;
                
                elseif eq_x>0
                    eq_x = eq_x-1;
                end
                eq_y = ( tand(phi)*(eq_x-a)+b ); %finding the value of corresponding y
                RESTRICTED(num_obs,1) = ceil(eq_x);
                RESTRICTED(num_obs,2) = ceil(eq_y);
                num_obs = num_obs+1;
            end
        end
    end
end