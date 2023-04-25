function eff = efficiencyPanel( azimuth,elevation,pan,phase_shift )
%UNTITLED Summary of this function goes here
% considering zero coordinates at mast and placed at the geometric center
%ahead of rover.
%assuming azimuth-elevation difference is positive. Therefore considering the sign
%convention of angle, projection calculations is done on only one half of the
%cartesian plane.

%units in mm
Hm=90;              
w=78.41;
l=192.44;
xbn=-325;
xbp=325;
x1=30;
x2=-146;
r=51;
gap=30;     %is not the actual gap
y1=-(301+w/2+gap);
y2=-(453+w/2+gap);
D=sqrt(w.^2+l.^2);
Tp=pan+atand(w/l);
Tn=pan-atand(w/l);

diffan=abs(azimuth-phase_shift);
if diffan>180
    diffan = 360-diffan;
end

%since rover is not symmetric, considering the sign convention now.
if azimuth>phase_shift %light falling from positive half of rover cartesian.
    diffan = -abs(diffan);
end

%continuing the above calulation efficiency calulations.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now feeding 3D coordinate values of obstructing plane projected by object.
%no plane projected by panel.
if diffan>0
    panel = [x2,-r-gap,0; x2,y1,0; x1,y1,0; x1,-r-gap,0; xbp,-r-gap,0; xbp,y2,0; xbn,y2,0; xbn,-r-gap,0]; %clockwise
    %Diagonal plane projected by camera.
    cameraPlane = [D/2*cos(Tp),D/2*sin(Tp),Hm+w; D/2*cos(Tp),D/2*sin(Tp),0; -D/2*cos(Tp),-D/2*sin(Tp),0; -D/2*cos(Tp),-D/2*sin(Tp),Hm+w]; %clockwise
    %tilt not included. For including both, calculate using rotation matrix
    %instead. Diagonal plane projected by mast.
    mastPlane = [r*cos(diffan),r*sin(diffan),Hm; r*cos(diffan),r*sin(diffan),0; -r*cos(diffan),-r*sin(diffan),0; -r*cos(diffan),-r*sin(diffan),Hm]; %clockwise
else
    panel = [x2,-r-gap,0; x2,y1,0; x1,y1,0; x1,-r-gap,0; xbp,-r-gap,0; xbp,y2,0; xbn,y2,0; xbn,-r-gap,0]; %clockwise
    cameraPlane = [D/2*cos(Tn),-D/2*sin(Tn),Hm+w; D/2*cos(Tn),-D/2*sin(Tn),0; -D/2*cos(Tn),D/2*sin(Tn),0; -D/2*cos(Tn),D/2*sin(Tn),Hm+w]; %clockwise
    mastPlane = [r*cos(diffan),-r*sin(diffan),Hm; r*cos(diffan),-r*sin(diffan),0; -r*cos(diffan),r*sin(diffan),0; -r*cos(diffan),r*sin(diffan),Hm];
end


sz=size(panel);
for a=1:sz(:,1)
    [panelPlane(a,1),panelPlane(a,2)] = shadowProj(panel(a,1), panel(a,2), panel(a,3), elevation, diffan);
end

sz=size(cameraPlane);
for a=1:sz(:,1)
    [cameraPlaneShadow(a,1),cameraPlaneShadow(a,2)] = shadowProj(cameraPlane(a,1), cameraPlane(a,2), cameraPlane(a,3), elevation, diffan);
end

sz=size(mastPlane);
for a=1:sz(:,1)
    [mastPlaneShadow(a,1),mastPlaneShadow(a,2)] = shadowProj(mastPlane(a,1), mastPlane(a,2), mastPlane(a,3), elevation, diffan);
end

%using polybool to calculate overlapping region.
[XPnC,YPnC] = polybool('intersection', panelPlane(:,1), panelPlane(:,2), cameraPlaneShadow(:,1), cameraPlaneShadow(:,2));
[XPnM,YPnM] = polybool('intersection', panelPlane(:,1), panelPlane(:,2), mastPlaneShadow(:,1), mastPlaneShadow(:,2));

area = polyarea(XPnC,YPnC)+polyarea(XPnM,YPnM);
totalAreaPanel = polyarea(panelPlane(:,1),panelPlane(:,2));

eff = 1-area/totalAreaPanel;
end