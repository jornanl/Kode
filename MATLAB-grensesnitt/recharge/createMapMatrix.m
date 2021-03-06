function [mapMatrix,robotStartLoc,robotLoc,xOffset,yOffset] = createMapMatrix(mapVector,globalMap,myRobotPath)
%ShowMap is a function for drawing privioulsy explored map. It requires
%mapVector and globalMap for computaion. The resault of this function is a
%figure with robot start position, current robot position and discoverd
%objects.
%   Detailed explanation goes here
M = zeros(100);
%figure(99)
% cla(figure(99));
% hold on;
% title('Global map');

%Robot start location
robotStartLoc = [2 2];
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


for j=1:4:size(mapVector,2) 
    %Inserting vertical lines in map matrix.
    %Check if the x-cords are approximatly the same
    if str2double(sprintf('%g',round(mapVector(j)*100)/100)) == str2double(sprintf('%g',round(mapVector(j+2)*100)/100));
        %Finding witch collum in x the following y values will be placed
        xCollum = sprintf('%g',round(mapVector(j)*10)/10);
        xCollum = str2double(xCollum)*10;
        if xCollum == 0;
            xCollum = 1;
        end
        %Finding in witch row in y the first y value will be placed
        yStart = sprintf('%g',round(mapVector(j+1)*10)/10);
        yStart = str2double(yStart)*10;
         if yStart == 0;
            yStart = 1;
        end
        %Creating a line between points in mapVector
        %tmpY = linspace(mapVector(j+1),mapVector(j+3),10);
        yStop = sprintf('%g',round(mapVector(j+3)*10)/10);
        yStop = str2double(yStop)*10;
         if yStop == 0;
            yStop = 1;
        end
        %If the stop is lesser than start value change them.. (Can happen
        %depends on scanning..
        if yStop < yStart;
            tmp = yStart;
            yStart = yStop;
            yStop = tmp;
        end
        for i=yStart:1:yStop
            M(i,xCollum) = 1;
        end
            
    %Inserting horizontal lines in map matrix.
    %Check if the x-cords are approximatly the same
    elseif str2double(sprintf('%g',round(mapVector(j+1)*100)/100)) == str2double(sprintf('%g',round(mapVector(j+3)*100)/100));
        %Finding witch collum in x the following y values will be placed
        yRow = sprintf('%g',round(mapVector(j+1)*10)/10);
        yRow = str2double(yRow)*10;
        if yRow == 0;
            yRow = 1;
        end
        %Finding in witch row in y the first y value will be placed
        xStart = sprintf('%g',round(mapVector(j)*10)/10);
        xStart = str2double(xStart)*10;
         if xStart == 0;
            xStart = 1;
        end
        %Creating a line between points in mapVector
        %tmpY = linspace(mapVector(j+1),mapVector(j+3),10);
        xStop = sprintf('%g',round(mapVector(j+2)*10)/10);
        xStop = str2double(xStop)*10;
         if xStop == 0;
            xStop = 1;
        end
        %If the stop is lesser than start value change them.. (Can happen
        %depends on scanning..)
        if xStop < xStart;
            tmp = xStart;
            xStart = xStop;
            xStop = tmp;
        end
        for i=xStart:1:xStop
            M(yRow,i) = 1;
        end
        
    end
    %plot(M)
%     plot([mapVector(j) mapVector(j+2)],[mapVector(j+1) mapVector(j+3)],'LineWidth',2.7,'Color',[1 0 0]);
end;

%Robot deiscovery matrix
R = ones(size(M));
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
    
figure(29) 
%test of draw path
hM=pcolor(R);
colormap([0 0 .5;1 0 0])
set(hM,'edgecolor','none','facecolor','flat')
% make obstacle grid square edge boundaries plot exactly true:
set(hM,'xdata',linspace(.5,99.5,100),'ydata',linspace(.5,99.5,100))
% axis ij
axis equal
axis tight
xlim([1 size(R,2)])
ylim([1 size(R,1)])
set(gca,'color',[0 0 .5])
xlabel('Column Coordinate')
ylabel('Row Coordinate')
end

%Merging the wall matrix and robot path matrix

for i=1:1:size(R,1)
    for j=1:1:size(R,1)
        if R(i,j) ~= 1;
            R(i,j) = M(i,j);
        end
    end
end

mapMatrix = R;

% figure(29)
% hM=pcolor(R);
% colormap([0 0 .5;1 0 0])
% set(hM,'edgecolor','none','facecolor','flat')
% % make obstacle grid square edge boundaries plot exactly true:
% set(hM,'xdata',linspace(.5,99.5,100),'ydata',linspace(.5,99.5,100))
% % axis ij
% axis equal
% axis tight
% xlim([1 size(R,2)])
% ylim([1 size(R,1)])
% set(gca,'color',[0 0 .5])
% xlabel('Column Coordinate')
% ylabel('Row Coordinate')
       
end
