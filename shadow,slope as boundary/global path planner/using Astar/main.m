%%variables to divide the DEM into various subsections. Picking 10000
%%points at a time from the file.
R1 = 0;
C1 = 0;
C2 = 2;
R2 = 9999;
%%
%12435501 is the number of data points avaiable in the DEM file.
while(R2<12435501)
    %%
    %taking only 4 iterations for publishing the document.
    %reading a 10000 values at a time and rearranging and interpolating.
    map_triplet = csvread('dem.csv',R1,C1,[R1 C1 R2 C2]); % add your dem here
    num = size(map_triplet);
    R1 = R2;
    R2 = R2 + num(:,1);
    unit_block = 150; %unit block in cms
    
    %%
    %for converting to easting, northing coordinates.
    for a = 1:num(:,1)
        map_triplet(a,1) = 360 + map_triplet(a,1);
    end
    F = scatteredInterpolant(map_triplet(:,1), map_triplet(:,2), map_triplet(:,3));
    max_Long = max(map_triplet(:,1));
    min_Long = min(map_triplet(:,1));
    max_Lat = max(map_triplet(:,2));
    min_Lat = min(map_triplet(:,2));
    
    %%
    %using the haversine function for interpolating the extracted
    %information.
    x = min_Long:((max_Long-min_Long)/haversine(max_Long, min_Long, min_Lat, min_Lat)*unit_block):max_Long;
    y = min_Lat:((max_Lat-min_Lat)/haversine(max_Long, min_Long, min_Lat, min_Lat)*unit_block):max_Lat;
    [~, MAX_X] = size(x);
    [~, MAX_Y] = size(y);
    %x = reshape(xq, MAX_X*MAX_Y, 1);
    %y = reshape(yq, MAX_X*MAX_Y, 1);

    %%
    %creating map and putting topography values for evaluation.
    map = ones(MAX_X, MAX_Y);
    for a = 1:MAX_X
        for b = 1:MAX_Y
            map(a,b) = F(x(1,a), y(1,b));
            coor_map(a,1) = x(1,a);
            coor_map(b,2) = y(1,b);
        end
    end
    
    %%
    %Defining the variables for cost determination.
    phi = 45;   %azimuth
    theta = 30; %elevation
    
    grid on;
    hold on;
    pcolor(map')
    colormap(gray(2000))

    %%
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
    
    %getting the values from pointer.
    xval=floor(xval);
    yval=floor(yval);
    xTarget=xval;%X Coordinate of the Target
    yTarget=yval;%Y Coordinate of the Target

    %%
    %not initializing map with target location cause map should be untouched.
    plot(xval,yval,'gd');
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
    
    %%
    %End of vehicle-Target pickup

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

    %%
    %Satisfying the least cost criteria, the ASTAR would get the Target
    %node starting from the start point to the end point. While loop starts
    %here
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
            
            %If an element is traversed again, it can create a deadlock
            %situation as the least cost node will be traversed again.
            %DELETING THE LAST ELEMENT FROM OPTIMUM BEACAUSE THAT IS A
            %DEADLOCK.
            Node = OPTIMUM(num_opt,:);
            OPTIMUM(num_opt,:) = [];
            num_opt = num_opt - 1;
        else
            h=msgbox('Sorry, No path exists to the Target!','warn');
            uiwait(h,5);
            break;
        end
        num_trav = num_trav +1;
        TRAVERSED(num_trav,:) = Node;
    end
    
    %%
    %Code for plotting the most optimum path.
    j=size(OPTIMUM,1);
    if ( (Node(:,1) == Target(:,1)) && (Node(:,2) == Target(:,2)) )
    %code for verification of path traversed, calculating costs for
    %all grids and plotting it on map.
    %Plot the OPTIMUM Path!
     j=size(OPTIMUM,1);
     p=plot(OPTIMUM(j,1),OPTIMUM(j,2),'bo');
     j=j-1;
     for i=j:-1:1
         pause(.005);
         set(p,'XData',OPTIMUM(i,1),'YData',OPTIMUM(i,2));
         drawnow;
     end;
     plot(OPTIMUM(:,1),OPTIMUM(:,2));
     hold on;
    
     %%
     %PLOTTING TRAVERSED PATH FOR COMPARISON.
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
end