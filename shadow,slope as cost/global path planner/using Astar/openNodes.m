function openList = openNodes( Node, prev, Raised, RESTRICTED, TRAVERSED, max_dim_x, max_dim_y )
%openNodes for determining all the corner points surrounding current node.
    %iteration for filtering the previously traversed node and boundary
    %nodes. Need to recheck the bondary condition though.
    
    OPEN_COUNT = 1;
        for i = -1:1
            a = Node(:,1)+i;
            for j = -1:1
                b = Node(:,2)+j;
                 flag = 0;
                if (a==prev(:,1))&&(b==prev(:,2)) || (a==Raised(:,1))&&(b==Raised(:,2)) || a<=1 || b<=1 || a>max_dim_x || b>max_dim_y
                        flag = 1;
                end 
                
                %checking for restricted points
                
                    for c = 1:size(RESTRICTED)
                        if  (a==RESTRICTED(c,1))&&(b==RESTRICTED(c,2))
                            flag = 1;
                        end
                    end           
                
                %checking for traversed points. In A star a point traversed again wil result in deadlock 

                    for c = 1:size(TRAVERSED)
                        if (a==TRAVERSED(c,1))&&(b==TRAVERSED(c,2))
                            flag = 1;
                        end
                    end
                    
                    
                if flag == 0 
                    openList(OPEN_COUNT,:) = [a,b];
                    OPEN_COUNT = OPEN_COUNT+1;
                else
                    flag = 0;
                end
            end
        end
        
        if OPEN_COUNT==1
            openList = [];
        end
end