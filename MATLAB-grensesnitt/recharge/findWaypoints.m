function waypoints = findWaypoints(shortestPathVector,robotStartLoc)
%A function for creating waypoints from a vector

%Inserting robot start location
waypoints = [];
x = shortestPathVector(:,1);
y = shortestPathVector(:,2);
%Limit value for concidered waypoints
limit = 0.01;
k=1;
oldChange = [0 0];
%Adding breakpoints from the shortest path route
for i=1:1:(size(shortestPathVector,1)-1)
    newChange = y(i+1)-y(i);
    if abs(newChange - oldChange) > limit;
        waypoints = [waypoints x(i+1) y(i+1)];
        k=k+1;
    end
    oldChange = newChange;

end

%Inserting the robot startposition and the endpoint of the shortest line
%vector

waypoints = [waypoints x(size(x,1)) y(size(y,1))];



end

