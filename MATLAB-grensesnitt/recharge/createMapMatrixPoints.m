function [mapMatrix,robotStartLoc,robotLoc,xOffset,yOffset] = createMapMatrixPoints(mapVector,mapVectorPoints,globalMap,myRobotPath)
%ShowMap is a function for drawing privioulsy explored map. It requires
%mapVector and globalMap for computaion. The resault of this function is a
%figure with robot start position, current robot position and discoverd
%objects.
%   Detailed explanation goes here



%Current robot position
robotLoc  = get(globalMap{1,1},'x');
robotLoc = [robotLoc(1) robotLoc(2)];

%Finding the smallest element in mapVector
xOffset = 9*10^99;
yOffset = 9*10^99;
for k=1:2:size(mapVector,2) 
    if mapVector(k) < xOffset;
       xOffset = mapVector(k);
    end
    if mapVector(k+1) < yOffset;
        yOffset = mapVector(k+1);
    end
end

%Finding the smallest element in mapVectorPoints
xOffset_p = 9*10^99;
yOffset_p = 9*10^99;
for k=1:1:size(mapVectorPoints,1) 
    if mapVectorPoints(k,1) < xOffset_p;
       xOffset_p = mapVectorPoints(k);
    end
    if mapVectorPoints(k,2) < yOffset_p;
        yOffset_p = mapVectorPoints(k,2);
    end
end

if xOffset_p < xOffset;
    xOffset = xOffset_p;
end
if yOffset_p < yOffset;
    yOffset = yOffset_p;
end

%Adding a positiv offset if there exist a negative number in x- or y-cords
%mapVectorPoint
if xOffset < 0 && yOffset < 0;
    mapVectorPoints(:,1) = mapVectorPoints(:,1) - xOffset;
    mapVectorPoints(:,2) = mapVectorPoints(:,2) - yOffset;
elseif xOffset < 0 && yOffset > 0;
    mapVectorPoints(:,1) = mapVectorPoints(:,1) - xOffset;
elseif xOffset > 0 && yOffset < 0;
    mapVectorPoints(:,2) = mapVectorPoints(:,2) - yOffset;
end
 
%Adding a positiv offset if there exist a negative number in x- or y-cords mapVector
if xOffset < 0 && yOffset < 0;
    robotLoc = [robotLoc(1)-xOffset robotLoc(2)-yOffset]; 
    for k=1:4:size(mapVector,2)
        mapVector(k) = mapVector(k) - xOffset;
        mapVector(k+2) = mapVector(k+2) - xOffset;
        mapVector(k+1) = mapVector(k+1) - yOffset;
        mapVector(k+3) = mapVector(k+3) - yOffset;
    end
elseif xOffset < 0 && yOffset > 0;
    robotLoc = [robotLoc(1)-xOffset robotLoc(2)];
    for k=1:4:size(mapVector,2)
        mapVector(k) = mapVector(k) - xOffset;
        mapVector(k+2) = mapVector(k+2) - xOffset;
    end
elseif xOffset > 0 && yOffset < 0;
    robotLoc = [robotLoc(1) robotLoc(2)-yOffset];
    for k=1:4:size(mapVector,2)
    mapVector(k+1) = mapVector(k+1) - yOffset;
    mapVector(k+3) = mapVector(k+3) - yOffset;
    end
end


robotLoc(1) = str2double(sprintf('%g',round(robotLoc(1)*10)/10));
robotLoc(1) = robotLoc(1)*10;
if robotLoc(1) == 0;
   robotLoc(1) = 1;
end
robotLoc(2) = str2double(sprintf('%g',round(robotLoc(2)*10)/10));
robotLoc(2) = robotLoc(2)*10;
if robotLoc(2) == 0;
   robotLoc(2) = 1;
end

xOffset_s = -xOffset;
yOffset_s = -yOffset;
xOffset_s = str2double(sprintf('%g',round(xOffset_s*10)/10));
xOffset_s = xOffset_s*10;
if xOffset_s == 0;
   xOffset_s = 1;
end
yOffset_s = str2double(sprintf('%g',round(yOffset_s*10)/10));
yOffset_s = yOffset_s*10;
if yOffset_s == 0;
   yOffset_s = 1;
end
%Robot start location
robotStartLoc = [xOffset_s yOffset_s];

% Check if there exsist circular objects in the map
% if mapVectorPoints ~= 0;
   %Compute the middel y-value
middel_y = sum(mapVectorPoints(:,2))/(size(mapVectorPoints,1));
lower_y = [];
upper_y = [];
%Check if the y value is lower or above the middel y-value. The pointvector
%is split into two lists.
for i=1:1:size(mapVectorPoints,1)
    %Convert the coords to mapMatrix coords
    y = str2double(sprintf('%g',round(mapVectorPoints(i,2)*10)/10));
    y = y*10;
    x = str2double(sprintf('%g',round(mapVectorPoints(i,1)*10)/10));
    x = x*10;
    if mapVectorPoints(i,2) < middel_y;
        lower_y = [lower_y; x y];
    else
        upper_y = [upper_y; x y];
    end
end

%Insertion sort
% sorted_lower_y = insertionSort(lower_y);
% sorted_upper_y = insertionSort(upper_y);
% sorted_upper_y = flipdim(sorted_upper_y,1);
% sorted_points = [sorted_lower_y;sorted_upper_y];
sorted_points = [lower_y;upper_y];

% for i=1:1:size(sorted_points,1)
%     if sorted_points(i,1) == 0;
%         sorted_points(i,1) = 1;
%     end
%     if sorted_points(i,2) == 0;
%         sorted_points(i,2) = 1;
%     end
%     M(sorted_points(i,2),sorted_points(i,1))=1;
% end
largest_x = 0;
largest_y = 0;
for k=1:2:size(mapVector,2) 
    if mapVector(k) > largest_x;
       largest_x =  mapVector(k);
    end
    if mapVector(k+1) > largest_y;
        largest_y = mapVector(k+1);
    end
end

largest_xp = 0;
largest_yp = 0;
for k=1:1:size(mapVectorPoints,1) 
    if mapVectorPoints(k,1) > largest_xp;
       largest_xp = mapVectorPoints(k);
    end
    if mapVectorPoints(k,2) > largest_yp;
        largest_yp = mapVectorPoints(k,2);
    end
end

if largest_xp > largest_x;
    largest_x = largest_xp;
end
if largest_yp > largest_y;
    largest_y = largest_yp;
end
largest = largest_x;
if largest_x < largest_y;
    largest = largest_y;
end
largest = str2double(sprintf('%g',round(largest*10)/10));
largest = largest*10;

%Robot deiscovery matrix
R = ones(largest+2);
%Visebility range
range = 4;

if xOffset < 0 && yOffset < 0;
    for k=1:1:size(myRobotPath,1)
        myRobotPath(k) = myRobotPath(k) - xOffset;
        myRobotPath(k,2) = myRobotPath(k,2) - yOffset;
    end
    elseif xOffset < 0 && yOffset > 0;
        for k=1:1:size(myRobotPath,1)
        myRobotPath(k) = myRobotPath(k) - xOffset;
        end
    elseif xOffset > 0 && yOffset < 0;
        for k=1:1:size(myRobotPath,1)
        myRobotPath(k,2) = myRobotPath(k,2) - yOffset;
        end
end

for k=1:1:size(myRobotPath,1)
    xRobotPos = str2double(sprintf('%g',round(myRobotPath(k,2)*10)/10));
    xRobotPos = xRobotPos*10;
    if xRobotPos == 0;
        xRobotPos = 1;
    end
    yRobotPos = str2double(sprintf('%g',round(myRobotPath(k,1)*10)/10));
    yRobotPos = yRobotPos*10;
    if yRobotPos == 0;
    yRobotPos = 1;
    end
    %Current robothouse is approx 20*20cm and range of ir-sensor is approx 80
    %cm. Creating a matrix for visible area of the robot. That gives a 180x180
    %cm visability matrix.
    %Checking if the visability area is inside the matrix
    %Not at the edge of the map
    if (xRobotPos-range) > 1 && (xRobotPos+range) < size(R,1) && (yRobotPos-range) > 1 && (yRobotPos+range) < size(R,1);
        R((xRobotPos-range):(xRobotPos+range),(yRobotPos-range):(yRobotPos+range)) = 0; 
        %At at the left side
    elseif (xRobotPos-range) < 1 && (yRobotPos-range) > 1;
        R(1:(xRobotPos+range),(yRobotPos-range):(yRobotPos+range)) = 0;
        %At at the right side
    elseif (xRobotPos+range) > size(R,1) && (yRobotPos+range) < size(R,1);
        R((xRobotPos-range):end-1,(yRobotPos-range):(yRobotPos+range)) = 0;
        %At at the top
    elseif (yRobotPos-range) < 1 && (xRobotPos-range) > 1;
        R(1:(xRobotPos+range),1:(yRobotPos+range)) = 0;
        %At at the bottom
    elseif (yRobotPos+range) > size(R,1) && (xRobotPos+range) < size(R,1);
        R((xRobotPos-range):(xRobotPos+range),(yRobotPos-range):end-1) = 0; 
        %In the upper left corner
    elseif (xRobotPos-range) < 1 && (xRobotPos+range) < size(R,1) && (yRobotPos-range) < 1 && (yRobotPos+range) < size(R,1);
        R(1:(xRobotPos+range),1:(yRobotPos+range)) = 0;
        %In the upper right corner
    elseif (xRobotPos-range) > 1 && (xRobotPos+range) > size(R,1) && (yRobotPos-range) > 1 && (yRobotPos+range) > size(R,1);
        R((xRobotPos-range):end-1,(yRobotPos-range):end-1) = 0; 
        %In the bottom left corner
    elseif (xRobotPos-range) < 1 && (xRobotPos+range) < size(R,1) && (yRobotPos-range) > 1 && (yRobotPos+range) > size(R,1);
        R(1:xRobotPos+range,(end-1):yRobotPos-range) = 0;  
        %In the bottom right corner
    elseif (xRobotPos-range) > 1 && (xRobotPos+range) > size(R,1) && (yRobotPos-range) > 1 && (yRobotPos+range) > size(R,1);
        R(end-1:xRobotPos-range,(end-1):yRobotPos-range) = 0; 
    end  

end
%% Placing the lower sorted values in the mapMatrix
% for i=1:1:size(sorted_lower_y,1)-1
%     if sorted_lower_y(i,1) == 0;
%         sorted_lower_y(i,1) = 1;
%     end
%     if sorted_lower_y(i,2) == 0;
%         sorted_lower_y(i,2) = 1;
%     end
%     b = [sorted_lower_y(i,1) sorted_lower_y(i,2)];
%     M(b(1),b(2)) = 1;
%     if (sorted_lower_y(i+1,1)-sorted_lower_y(i,1)) ~= 0;
%         a = ((sorted_lower_y(i+1,2)-(sorted_lower_y(i,2))))/((sorted_lower_y(i+1,1)-(sorted_lower_y(i,1))));
%         m = a;
%         for j=sorted_lower_y(i,1):1:sorted_lower_y(i+1,1)
%             if a < 0;
%                 M(j,sorted_lower_y(i,2):sorted_lower_y(i,2)+m) = 1;
%                  m = m + a;
%             elseif a > 0;
%                 M(j,sorted_lower_y(i,2):sorted_lower_y(i,2)-m) = 1;
%                 m = m - a;
%             end   
%         end 
%     end
% tmp1 = sorted_lower_y(size(sorted_lower_y,1),1);
% tmp2 = sorted_lower_y(size(sorted_lower_y,1),2);
% if tmp1 == 0;
%     tmp1 = 1;
% end
% if tmp2 == 0;
%     tmp2 = 1;
% end
% M(tmp1,tmp2) = 1;
% 
% end
% 
% %Placing the upper sorted values in the mapMatrix
% for i=1:1:size(sorted_upper_y,1)-1
%     if sorted_upper_y(i,1) == 0;
%         sorted_upper_y(i,1) = 1;
%     end
%     if sorted_upper_y(i,2) == 0;
%         sorted_upper_y(i,2) = 1;
%     end
%     b = [sorted_upper_y(i,1) sorted_upper_y(i,2)];
%     M(b(1),b(2)) = 1;
%     if (sorted_upper_y(i+1,1)-sorted_upper_y(i,1)) ~= 0;
%         a = round(((sorted_upper_y(i+1,2)-(sorted_upper_y(i,2))))/((sorted_upper_y(i+1,1)-(sorted_upper_y(i,1)))));
%         m = 0;
%         tmp = a;
%         for j=sorted_upper_y(i,1):-1:sorted_upper_y(i+1,1)
%             if sorted_upper_y(i,2)+tmp <= 0;
%                 M(j,sorted_upper_y(i,2)+m:1) = 1;
%             elseif sorted_upper_y(i,2)+m <= 0;
%                 M(j,1:sorted_upper_y(i,2)+tmp) = 1; 
%             else 
%             M(j,sorted_upper_y(i,2)+m:sorted_upper_y(i,2)+tmp) = 1;    
%             m = a;
%             tmp = a - m;
%             end
%         end 
%     end
% end
% 
% tmp1 = sorted_upper_y(size(sorted_upper_y,1),1);
% tmp2 = sorted_upper_y(size(sorted_upper_y,1),2);
% if tmp1 == 0;
%     tmp1 = 1;
% end
% if tmp2 == 0;
%     tmp2 = 1;
% end
% 
% M(tmp1,tmp2) = 1;
%   
% end

for i=1:1:size(sorted_points,1)
    if sorted_points(i,1) == 0;
        sorted_points(i,1) = 1;
    end
    if sorted_points(i,2) == 0;
        sorted_points(i,2) = 1;
    end
    R(sorted_points(i,2),sorted_points(i,1))=1;
end


for j=1:4:size(mapVector,2) 
     %Finding in witch row in y the first y value will be placed
    xStart = sprintf('%g',round(mapVector(j)*10)/10);
    xStart = str2double(xStart)*10;
    if xStart == 0;
       xStart = 1;
    end
    xStop = sprintf('%g',round(mapVector(j+2)*10)/10);
    xStop = str2double(xStop)*10;
    if xStop == 0;
       xStop = 1;
    end
    if xStop < xStart;
        tmp = xStart;
        xStart = xStop;
        xStop = tmp;
    end
    %Finding in witch row in y the first y value will be placed
    yStart = sprintf('%g',round(mapVector(j+1)*10)/10);
    yStart = str2double(yStart)*10;
    if yStart == 0;
       yStart = 1;
    end
    yStop = sprintf('%g',round(mapVector(j+3)*10)/10);
    yStop = str2double(yStop)*10;
    if yStop == 0;
       yStop = 1;
    end
    
    xDiff = xStop - xStart;
    if abs(xDiff) == 1;
        xDiff = 0;
    end
    yDiff = yStop - yStart;
    if abs(yDiff) == 1;
        yDiff = 0;
    end
    a = round(yDiff/xDiff);
    
    %Horizontal line
    if a == 0;
       R(yStart,xStart:xStop) = 1;
    %Vertical line
    elseif a == Inf;
       R(yStart:yStop,xStart) = 1;
    elseif a == -Inf;
       R(yStop:yStart,xStart) = 1; 
    elseif a >0 && abs(a) ~= Inf;
    %Tilted line
       yDraw = 0;
       for i=xStart:1:xStop
           if yDraw < yDiff;
               R(yStart+yDraw:yStart+yDraw+a,i) = 1;
           else
               R(yDraw:yDiff,i) = 1;
           end
           yDiff = yDiff - yDraw;
           yDraw = yDraw+a;
       end
    elseif a < 0 && abs(a) ~= Inf;
       yDraw = 0;
       for i=xStart:1:xStop
           if yDraw < yDiff;
               R(yStart+yDraw:yStart+yDraw+a,i) = 1;
           else
               R(yDraw:yDiff,i) = 1;
           end
           yDiff = yDiff - yDraw;
           yDraw = yDraw-a;
       end
    end
% figure(30)
% hM=pcolor(R);
% colormap([1 1 1;1 0 0])
% set(hM,'edgecolor','none','facecolor','flat')
% % make obstacle grid square edge boundaries plot exactly true:
% % set(hM,'xdata',linspace(.5,99.5,100),'ydata',linspace(.5,99.5,100))
% %axis ij
% % axis equal
% % axis tight
% % xlim([1 size(R,2)])
% % ylim([1 size(R,1)])
% set(gca,'color',[1 1 1])
% xlabel('Column Coordinate')
% ylabel('Row Coordinate')

end

mapMatrix = R;

% figure(30)
% hM=pcolor(R);
% colormap([1 1 1;1 0 0])
% % set(hM,'edgecolor','none','facecolor','flat')
% % % make obstacle grid square edge boundaries plot exactly true:
% % set(hM,'xdata',linspace(.5,99.5,100),'ydata',linspace(.5,99.5,100))
% % %axis ij
% % axis equal
% % axis tight
% % xlim([1 size(R,2)])
% % ylim([1 size(R,1)])
% % set(gca,'color',[1 1 1])
% xlabel('Column Coordinate')
% ylabel('Row Coordinate')
       
end
