function shortestPathVector = computeShortestPath(globalMap,myRobotPath)
%COMPUTESHORTESTPATH Summary of this function goes here
%   Detailed explanation goes here

Q = globalMap;
mapVector = createMapVector(Q);
[A,ri,rf,xOffset,yOffset] = createMapMatrix(mapVector,Q,myRobotPath);

tmpM = A(ri(1):rf(1),ri(2):rf(2));
tmpM = find(tmpM, 1);
if ~isempty(tmpM);
    shortestPath = findShpath(A,ri,rf);
    waypoints = findWaypoints(shortestPath,ri);
    shortestPathVector = convToOrgCoordinates(waypoints,xOffset,yOffset,size(A,1));
else
    shortestPathVector = [0 0];
end

end
