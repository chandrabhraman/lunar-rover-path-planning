function node = nextPoint( OPEN, Node, Target, coor_map, azimuth, map )
%UNTITLED Summary of this function goes here
%Detailed explanation goes here
%first for loop for calculating standard deviation and average for topography, distance and
%angle.

sz = size(OPEN);
for num = 1:sz
    %for distance
    X(num,:) = costCalc(OPEN(num,:), Node, Target, coor_map, map, azimuth);
end

%sequentially X1 = slope, X2 = deviation from sun, X3 = octile distance.
cost=[];

%slope = [ min( X(:,1)), max( X(:,1)) ];
%dev = [ min( X(:,2)), max( X(:,2)) ];
dis = [ min( X(:,3)), max(X(:,3)) ];

for num = 1:sz 
    %normalizing the quantities.

    %C1 = (  X(num,1)-slope(:,1)  )/(  slope(:,2)-slope(:,1)  );
    %C2 = (  X(num,2)-dev(:,1)    )/(     dev(:,2)-dev(:,1)   );          
    C3 = (  X(num,3)-dis(:,1)    )/(     dis(:,2) - dis(:,1) );        %using normalization of quantities
    C1 = X(num,1)/tand(30);
    C2 = X(num,2);
    cost(num) = C3+C2+C1; 
end
    [~,index] = min(cost);
    node = OPEN(index,:);
end