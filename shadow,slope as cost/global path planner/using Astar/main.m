R1 = 0;
C1 = 0;
C2 = 2;
R2 = 9999;
while(R2<12435501)
    %reading a 1000 values at a time.
    map_triplet = csvread('dem.csv',R1,C1,[R1 C1 R2 C2]); % add your dem here
    num = size(map_triplet);
    R1 = R2;
    R2 = R2 + num(:,1);
    
    %for converting to easting, northing coordinates.
    for a = 1:num(:,1)
        map_triplet(a,1) = 360 + map_triplet(a,1);
    end
    
    phi = 45;   %azi
    theta = 30; %elevation
    unit_block = 150; %unit block in cms

    F = scatteredInterpolant(map_triplet(:,1), map_triplet(:,2), map_triplet(:,3));
    max_Long = max(map_triplet(:,1));
    min_Long = min(map_triplet(:,1));
    max_Lat = max(map_triplet(:,2));
    min_Lat = min(map_triplet(:,2));
    %[xq, yq] = meshgrid(min_Long:((max_Long-min_Long)/haversine(max_Long, min_Long, min_Lat, min_Lat)*unit_block):max_Long, min_Lat:((max_Lat-min_Lat)/haversine(max_Long, min_Long, min_Lat, min_Lat)*unit_block):max_Lat);

    x = min_Long:((max_Long-min_Long)/haversine(max_Long, min_Long, min_Lat, min_Lat)*unit_block):max_Long;
    y = min_Lat:((max_Lat-min_Lat)/haversine(max_Long, min_Long, min_Lat, min_Lat)*unit_block):max_Lat;
    
    [~, MAX_X] = size(x);
    [~, MAX_Y] = size(y);
    %x = reshape(xq, MAX_X*MAX_Y, 1);
    %y = reshape(yq, MAX_X*MAX_Y, 1);

    %creating map and putting topography values for evaluation.
    map = ones(MAX_X, MAX_Y);
    for a = 1:MAX_X
        for b = 1:MAX_Y
            map(a,b) = F(x(1,a), y(1,b));
            coor_map(a,1) = x(1,a);
            coor_map(b,2) = y(1,b);
        end
    end
    
    grid on;
    hold on;
    pcolor(map')
    colormap(gray(2000))

    %begin interactive start and target
    pause(1);
    h=msgbox('Please Select the Target using the Left Mouse button');
    if ishandle(h) == 1
        delete(h);
    end
    
    xlabel('Please Select the Target using the Left Mouse button','Color','black');
    but=0;
    while (but ~= 1) %Repeat until the Left button is not clicked
        [xval,yval,but]=ginput(1);
    end

    xval=floor(xval);
    yval=floor(yval);
    xTarget=xval;%X Coordinate of the Target
    yTarget=yval;%Y Coordinate of the Target

    %not initializing map with target location cause map should be untouched.
    plot(xval,yval,'gd');
    %%why, 0.5?
    text(xval+1,yval+.5,'Target')

    pause(1);
    h=msgbox('Please Select the vehicle initial position using the Left Mouse button');
    uiwait(h,5);
    if ishandle(h) == 1
        delete(h);
    end
    xlabel('Please Select the Vehicle initial position ','Color','black');
    but=0;
    while (but ~= 1) %Repeat until the Left button is not clicked
        [xval,yval,but]=ginput(1);
        xval=floor(xval);
        yval=floor(yval);
    end
    xStart=xval;%Starting Position
    yStart=yval;%Starting Position

    plot(xval,yval,'bo');
    text(xval+1,yval+.5,'starting point');
    
    %End of vehicle-Target pickup
    %RESTRICTED = shadow(map, theta, phi); %need revision

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    RESTRICTED=[]; %restricted points with more deviation than std(map).
    OPENList=[];
    TRAVERSED=[]; %for keeping track of traversed path (USED FOR PLOTTING). analogous to CLOSED[].
    OPTIMUM=[]; %could be used for building hueristics. for keeping track of most optimum path
    %(avoiding deadlocks).

    %set the starting node as the first node
    Node=[xStart, yStart];
    Prev=Node;
    Raised = [-1,-1];
    num_trav = 0;
    num_opt = 0;
    deadlock = 0;
    Target = [xTarget, yTarget];

    while (Node(:,1)~=Target(:,1)) || (Node(:,2)~=Target(:,2))
        OPENList = openNodes(Node, Prev, Raised, RESTRICTED, TRAVERSED, MAX_X, MAX_Y);
        sz = size(OPENList);
        Prev = Node;
        if sz(:,1)~=0
            num_opt = num_opt+1;
            OPTIMUM(num_opt,:) = Node;
            Node = nextPoint(OPENList, Node, Target, coor_map, phi, map);
            OPENList=[];
            
        %condition when deadlock occurs. No points available for traversal so no points are open
        elseif num_trav>0 && sz(:,1)==0
            
            %raissed points cannot be traversed again (COULD'NT BE OPEN).
            
            %for extracting the previous points.
            %Node = TRAVERSED(num_trav-num_trav_bck-1,:); 

            %DELETING THE LAST ELEMENT FROM OPTIMUM BEACAUSE THAT IS A
            %DEADLOCK.
            Node = OPTIMUM(num_opt,:);
            OPTIMUM(num_opt,:) = [];
            num_opt = num_opt - 1;
            
            %but we need a step back for taking the raised points.
            %num_trav_bck = num_trav_bck + 2;
        else
            h=msgbox('Sorry, No path exists to the Target!','warn');
            uiwait(h,5);
            break;
        end
        num_trav = num_trav +1;
        TRAVERSED(num_trav,:) = Node;
    end

    j=size(OPTIMUM,1);
    if ( (Node(:,1) == Target(:,1)) && (Node(:,2) == Target(:,2)) )
   
    %code for verification of path traversed, calculating costs for
    %all grids and plotting it on map.
    
    
    %the most cost efficient path can be obtained from the equation
    %[X'X]-1X*(theta) = Y, where X are the cost matrix and Y is topography.
    
    
    
    %PLOTTING TRAVERSED PATH FOR COMPARISON.
     j=size(TRAVERSED,1);
     p=plot(TRAVERSED(j,1),TRAVERSED(j,2),'bo');
     j=j-1;
     for i=j:-1:1
         pause(.005);
         set(p,'XData',TRAVERSED(i,1),'YData',TRAVERSED(i,2));
         drawnow ;
     end;
     plot(TRAVERSED(:,1),TRAVERSED(:,2));
     else
         pause(1);
         h=msgbox('Sorry, No path exists to the Target!','warn');
         uiwait(h,5);
     end

     %Plot the OPTIMUM Path!
     p=plot(OPTIMUM(j,1),OPTIMUM(j,2),'bo');
     j=j-1;
     for i=j:-1:1
         pause(.005);
         set(p,'XData',OPTIMUM(i,1),'YData',OPTIMUM(i,2));
         drawnow;
     end;
     plot(OPTIMUM(:,1),OPTIMUM(:,2));
     hold on;
end