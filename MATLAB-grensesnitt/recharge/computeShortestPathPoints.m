function shortestPathVector = computeShortestPathPoints(globalMap,globalMap1,myRobotPath)
%COMPUTESHORTESTPATH Summary of this function goes here
%   Detailed explanation goes here
W = globalMap1;
Q = globalMap;
mapVectorPoints = createMapVectorPoints(W);
mapVector = createMapVector(Q);

 if isempty(mapVectorPoints);
    mapVectorPoints = [1 1];
 end
[A,ri,rf,xOffset,yOffset] = createMapMatrixPoints(mapVector,mapVectorPoints,Q,myRobotPath);
tmpM = A(ri(1):rf(1),ri(2):rf(2));
tmpM = find(tmpM, 1);
if ~isempty(tmpM);
    shortestPath = findShpath(A,ri,rf);
    waypoints = findWaypoints(shortestPath,ri);
    shortestPathVector = convToOrgCoordinates(waypoints,xOffset,yOffset,size(A,1));
else
    hold on
    plot([ri(1) rf(1)],[ri(2) rf(2)],'g','LineWidth',1);
    plot(rf(1),rf(2),'kx','LineWidth',2,'MarkerSize',5);
    hold off
    shortestPathVector = [0 0];
end

end