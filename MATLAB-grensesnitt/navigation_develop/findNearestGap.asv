% returns midpoint-coordinates of the nearest gap from the pathPoint.
function [gapX,gapY,gapAngleFinal]=findNearestGap(myNavData,pathPoint,gaps)

%find big enough gaps
longEnoughGaps=[];
for i=1:1:size(gaps,1) % Goes through all the gaps
    for j=1:4:size(gaps,2)
        length = sqrt( (gaps(i,j)-gaps(i,j+2))^2 + (gaps(i,j+1)-gaps(i,j+3))^2 ); % Length of the gap segment
        if( length > 0.25 )
            longEnoughGaps(size(longEnoughGaps,1)+1,1:1:4)=gaps(i,j:1:j+3); % Store the gaps where the segment is long enough
        end;
    end;
end;
 
% find the big gaps the robot hasn't travelled trough 
% Checks if the robots travelled path has crossed any of the gaps
interrestingGaps=[];
if( size(longEnoughGaps,1) > 0 )  

    for j=1:1:size(longEnoughGaps,1)
        interresting=1;        
        for i= 1:1:size(myNavData,1)-1,
            poseN = myNavData(i,1:2);
            poseM = myNavData(i+1,1:2);

            doTheyCrossEachOther = vectorCrossingChecker( longEnoughGaps(j,:), [poseN(1:2) poseM(1:2)] );
            if( doTheyCrossEachOther == 1 )
               interresting = 0;
            end;
        end;
        
        if(interresting)
            interrestingGaps(size(interrestingGaps,1)+1,1:1:4) = longEnoughGaps(j,:) ;
            
        end;
    end;        
end;
%debug
%figure(99);clf;hold on;
%for i=1:1:size(interrestingGaps,1),
%    plot([interrestingGaps(i,1) interrestingGaps(i,3)],...
%[interrestingGaps(i,2) interrestingGaps(i,4)]);
%end
%debug

% Find nearest gap
myPointX=myNavData(pathPoint,1); % pathPoint is the step where the last gap was before the backtracking 
myPointY=myNavData(pathPoint,2);
gapX=-1;
gapY=-1;
minDist=Inf;
if( size(interrestingGaps,1) > 0 )
    for m=1:1:size(interrestingGaps,1)
        interrestingGapMidPointX = 0.5*(interrestingGaps(m,1)+interrestingGaps(m,3));
        interrestingGapMidPointY = 0.5*(interrestingGaps(m,2)+interrestingGaps(m,4));
        
        warning off
        a=(interrestingGaps(m,4)-interrestingGaps(m,2))/(interrestingGaps(m,3)-interrestingGaps(m,1)); % Gradient of the gap segment, length of the gap
        warning on
        
        distVect(m)= (myPointX-interrestingGapMidPointX).^2 + (myPointY-interrestingGapMidPointY).^2; % The distance squard between the robots current position and the midpoint of the gap
        
        [dist,index]=min(distVect);
        
        if dist<minDist,
            minDist=dist;
            gapX = interrestingGapMidPointX;
            gapY = interrestingGapMidPointY;
            gapAngleFinal=180*(atan(a)+pi/2)/pi; % perpendicular to gap, in degrees
        end
    end
end 
