function [navDataNextIndex,gotoThroughThisGapX,gotoThroughThisGapY]...
    =findNextInterestingGapPathIndex2(myNavData,myNavDataAll,gaps)

gotoThroughThisGapX =0;
gotoThroughThisGapY=0;

%disp('---------------STARTING----------------')
%debug
% figure(20);clf;hold on; 
% gaps = [rand(1)*50 rand(1)*50 rand(1)*50 rand(1)*50; rand(1)*50 rand(1)*50 rand(1)*50 rand(1)*50];
% navData.robotPath(1,1:2)=[0 0];
% for i=2:1:20    
%     navData.robotPath(i,1:2) = navData.robotPath(i-1,1:2)+[rand(1)*10 rand(1)*10];
% end;
% for i=21:1:40    
%     navData.robotPath(i,1:2) = navData.robotPath(i-1,1:2)+[rand(1)*10 -rand(1)*10];
% end;
% 
% for i=1:1:size(gaps,1)
%     for j=1:4:size(gaps,2)
%         plot( [gaps(i,j) gaps(i,j+2)],[gaps(i,j+1) gaps(i,j+3) ],'r','LineWidth',2.7);
%         plot( gaps(i,j),gaps(i,j+1),'kx','LineWidth',2,'MarkerSize',9);
%         plot( gaps(i,j+2),gaps(i,j+3),'kx','LineWidth',2,'MarkerSize',9);
%     end;
% end;
% 
% for i=1:1:40-1
%     plot( [navData.robotPath(i,1) navData.robotPath(i+1,1)],[navData.robotPath(i,2) navData.robotPath(i+1,2)],'g','LineWidth',2.7);
%     plot( navData.robotPath(i,1),navData.robotPath(i,2),'kx','LineWidth',2,'MarkerSize',9);
%     plot( navData.robotPath(i+1,1),navData.robotPath(i+1,2),'kx','LineWidth',2,'MarkerSize',9);   
% end;
%debug    

%find big gaps
longEnoughGaps=[];
for i=1:1:size(gaps,1)
    for j=1:4:size(gaps,2)
        length = sqrt( (gaps(i,j)-gaps(i,j+2))^2 + (gaps(i,j+1)-gaps(i,j+3))^2 ); 
        if( length > 0.25 )
            longEnoughGaps(size(longEnoughGaps,1)+1,1:1:4)=gaps(i,j:1:j+3);
        end;
    end;
end;


% find the big gaps the robot hasn't travelled trough 
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

%figure(90);clf;hold on;
%for i=1:1:size(interrestingGaps,1),
%    plot([interrestingGaps(i,1) interrestingGaps(i,3)],...
%    [interrestingGaps(i,2) interrestingGaps(i,4)]);
%end

%interrestingGaps
%%find the interresting gap closest to robot, from a backtracking point of view
closestInterestingGapPathIndex = 0;
finalGapPoints=[];
if( size(interrestingGaps,1) > 0 )
    for m=1:1:size(interrestingGaps,1)
        interrestingGapMidPointX = 0.5*(interrestingGaps(m,1)+interrestingGaps(m,3));
        interrestingGapMidPointY = 0.5*(interrestingGaps(m,2)+interrestingGaps(m,4));
        
        distVect=(myNavData(:,1)-interrestingGapMidPointX).^2+(myNavData(:,2)-interrestingGapMidPointY).^2;
        [v,index]=min(distVect);
        change = closestInterestingGapPathIndex;
        closestInterestingGapPathIndex = max(closestInterestingGapPathIndex,index);
        
        if(change ~= closestInterestingGapPathIndex )
            %remember also the gapMidpoints
            finalGapPoints = [interrestingGapMidPointX interrestingGapMidPointY];
        end;
    end;    
end;

%debug
% if(closestInterestingGapPathIndex ~= 0)
%     plot( navData.robotPath(closestInterestingGapPathIndex,1),navData.robotPath(closestInterestingGapPathIndex,2),'rx','LineWidth',2,'MarkerSize',9);
% end;
%debug

numPoses=size(myNavDataAll,1);
if( size(interrestingGaps,1) > 0 )
    distVect2a=(myNavDataAll(1:closestInterestingGapPathIndex-4,1)-myNavDataAll(closestInterestingGapPathIndex,1)).^2+(myNavDataAll(1:closestInterestingGapPathIndex-4,2)-myNavDataAll(closestInterestingGapPathIndex,2)).^2;
    distVect2b=(myNavDataAll(closestInterestingGapPathIndex+4:numPoses-1,1)-myNavDataAll(closestInterestingGapPathIndex,1)).^2+(myNavDataAll(closestInterestingGapPathIndex+4:numPoses-1,2)-myNavDataAll(closestInterestingGapPathIndex,2)).^2;
    distVect2=[distVect2a; distVect2b];
    [d,index]=min(distVect2);
    %if d<0.10*0.10,
    %    disp('found a gap index where the robot already has travelled before..')
    %    closestInterestingGapPathIndex=-1;
    %    finalGapPoints=[-1 -1];
    %end
end

navDataNextIndex = closestInterestingGapPathIndex;
if(closestInterestingGapPathIndex== 0)
    return;
end;
gotoThroughThisGapX= finalGapPoints(1);
gotoThroughThisGapY= finalGapPoints(2);