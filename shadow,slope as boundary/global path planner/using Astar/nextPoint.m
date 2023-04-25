%%
%Function for determining the set of next neighbou points.
function node = nextPoint( OPEN, Node, Target, coor_map, azimuth, map )
%first for loop for calculating standard deviation and average for topography, distance and
%angle.

sz = size(OPEN);
for num = 1:sz
    X(num,:) = costCalc(OPEN(num,:), Node, Target, coor_map, map, azimuth);
end

%sequentially X1 = slope, X2 = deviation from sun, X3 = octile distance.
cost=[];
dis = [ min( X(:,3)), max(X(:,3)) ];

for num = 1:sz 
    %normalizing the distance criteria.
    C3 = (  X(num,3)-dis(:,1)    )/(     dis(:,2) - dis(:,1) ); %using normalization of quantities.
    %standard method.
    C1 = X(num,1);
    C2 = X(num,2);
    %Total cost.
    cost(num) = C3+C2+C1; 
end
    [~,index] = min(cost);
    node = OPEN(index,:);
end