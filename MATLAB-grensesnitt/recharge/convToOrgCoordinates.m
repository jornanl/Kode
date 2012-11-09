function actualWaypoints = convToOrgCoordinates(waypoints,xOffset,yOffset,sizeMapMatrix)
%CONVTOORGCOORDINATES Summary of this function goes here
%   Detailed explanation goes here

%Check if the offset is negative. If it is positive it is unused and have
%not been compensated for earlier
if xOffset > 0;
    xOffset = 0;
end
if yOffset > 0;
    yOffset = 0;
end

%Compansating for the previous flipdim in findShpath

for i=1:2:(size(waypoints,2))
    waypoints(i+1) = sizeMapMatrix - waypoints(i+1);
    
    %Changing the resolution of the vector to match the original map and
    %subtracting the offset from previous positive map
    waypoints(i) = waypoints(i)*0.1 + xOffset;
    waypoints(i+1) = waypoints(i+1)*0.1 + yOffset;
    
end
actualWaypoints = [waypoints];
end

