
function G=fixCorners( Gmap, maxDistance, minLength )
%% This function merges startstop-points from two 
%% different segments ,if the points are close enough
%% and the segments them self are long enough....

maxDist=maxDistance*maxDistance;
minLen=minLength*minLength;
globalMap = get(Gmap,'x');



%debug
%figure(7);clf;hold on;
% for i=2:1:size(globalMap,2)
%     %points = get(globalMap{1,i},'ss');
%     testPoints = get( globalMap{1,i},'ss');
%     for j=1:4:size(testPoints,2)        
%         plot([testPoints(j) testPoints(j+2)],[testPoints(j+1) testPoints(j+3)],'g','LineWidth',2.7);
%         plot( testPoints(j),testPoints(j+1),'kx','LineWidth',2,'MarkerSize',9);
%         plot( testPoints(j+2),testPoints(j+3),'kx','LineWidth',2,'MarkerSize',9);                    
%     end;         
% end;
        
longEnoughSegs=[];
%Finds the segments that are long enough.
for i=2:length(globalMap)
    %get(globalMap{1,i},'type');
    %if isa(globalMap{1,i},'alpha,r line feature'),
    points = get( globalMap{1,i},'ss');
    ind=[];
    for k=1:4:size(points,2)        
        e=[points(k)-points(k+2) points(k+1)-points(k+3)];
        len = e*e';
        if( len > minLen ) %segment long enough
            longEnoughSegs(size(longEnoughSegs,1)+1,:)=[i k];
        end;
    end;
    %end
end;

%longEnoughSegs
%Finds two segments with "close enough" start/stopp points.
matchedLongEnoughSegs=[];
for i=1:size(longEnoughSegs,1)
    pointsA = get( globalMap{1,longEnoughSegs(i,1)},'ss');
    pointsAstart = pointsA( longEnoughSegs(i,2):1:longEnoughSegs(i,2)+1 );
    pointsAstopp = pointsA( longEnoughSegs(i,2)+2:1:longEnoughSegs(i,2)+3 );

    for k=1+i:1:size(longEnoughSegs,1)
         if(longEnoughSegs(k,1) ~= longEnoughSegs(i,1) ) % dont match same line
             pointsB = get(globalMap{1,longEnoughSegs(k,1)},'ss');
              pointsBStart = pointsB( longEnoughSegs(k,2):1:longEnoughSegs(k,2)+1 );
             pointsBStopp = pointsB( longEnoughSegs(k,2)+2:1:longEnoughSegs(k,2)+3 );        
         
            e = pointsAstart-pointsBStart;
            if( e*e' < maxDist )
                matchedLongEnoughSegs(size(matchedLongEnoughSegs,1)+1,:) = [longEnoughSegs(i,1) longEnoughSegs(i,2) longEnoughSegs(k,1) longEnoughSegs(k,2) 0 0];
            end;
            e = pointsAstart-pointsBStopp;
            if( e*e' < maxDist )
                matchedLongEnoughSegs(size(matchedLongEnoughSegs,1)+1,:) = [longEnoughSegs(i,1) longEnoughSegs(i,2) longEnoughSegs(k,1) longEnoughSegs(k,2) 0 2];
            end;
            e = pointsAstopp-pointsBStart;       
            if( e*e' < maxDist )
                matchedLongEnoughSegs(size(matchedLongEnoughSegs,1)+1,:) = [longEnoughSegs(i,1) longEnoughSegs(i,2) longEnoughSegs(k,1) longEnoughSegs(k,2) 2 0];
            end;
            e = pointsAstopp-pointsBStopp;
            if( e*e' < maxDist )        
                matchedLongEnoughSegs(size(matchedLongEnoughSegs,1)+1,:) = [longEnoughSegs(i,1) longEnoughSegs(i,2) longEnoughSegs(k,1) longEnoughSegs(k,2) 2 2];
            end;
        end;
    end;
end;

%matchedLongEnoughSegs;
%update the start and stopp points
intersectionPointX=[];
intersectionPointY=[];
for i = 1:1:size(matchedLongEnoughSegs,1)
     warning off;
     deltaLineAallpoints = get( globalMap{1,matchedLongEnoughSegs(i,1)},'ss');
     deltaLineApoints = deltaLineAallpoints(matchedLongEnoughSegs(i,2):1:matchedLongEnoughSegs(i,2)+3);
     deltaLineAdnum = deltaLineApoints(1)-deltaLineApoints(3);
     deltaLineAnum  = deltaLineApoints(2)-deltaLineApoints(4);
 
     deltaLineA = deltaLineAnum/deltaLineAdnum;
 
     deltaLineBallpoints = get( globalMap{1,matchedLongEnoughSegs(i,3)},'ss');
     deltaLineBpoints = deltaLineBallpoints(matchedLongEnoughSegs(i,4):1:matchedLongEnoughSegs(i,4)+3);
     deltaLineBdnum = deltaLineBpoints(1)-deltaLineBpoints(3);
     deltaLineBnum  = deltaLineBpoints(2)-deltaLineBpoints(4);
 
     deltaLineB = deltaLineBnum/deltaLineBdnum;
     warning on;
     
     fixedCorner = 0;
     
     
     % setting threshold
     if abs(deltaLineA)>1e10,
         deltaLineA=Inf;
     end
     if abs(deltaLineB)>1e10,
         deltaLineB=Inf;
     end
     
     
     if(abs(deltaLineA) == Inf && abs(deltaLineB) ==Inf )
         warning 'Bjørn, fixCorners is trying to link parallell lines...';
     elseif(abs(deltaLineA) == Inf && abs(deltaLineB) ~= Inf )
         intersectionPointX=deltaLineApoints(1);
         intersectionPointY=deltaLineB*(intersectionPointX-deltaLineBpoints(1))+deltaLineBpoints(2);
         fixedCorner = 1;                     
     elseif(abs(deltaLineA) ~= Inf && abs(deltaLineB) == Inf )
         intersectionPointX=deltaLineBpoints(1);
         intersectionPointY=deltaLineA*(intersectionPointX-deltaLineApoints(1))+deltaLineApoints(2);
         fixedCorner = 1;
     elseif(abs(deltaLineA) ~=Inf && abs(deltaLineB) ~=Inf )
         fixedCorner = 1;         
         intersectionPointX = (-deltaLineB*deltaLineBpoints(1)+deltaLineBpoints(2)+deltaLineA*deltaLineApoints(1)-deltaLineApoints(2))/(deltaLineA-deltaLineB);
         intersectionPointY = deltaLineA*(intersectionPointX-deltaLineApoints(1))+deltaLineApoints(2);        
     end;
     
     if( fixedCorner )
         deltaLineAallpoints(matchedLongEnoughSegs(i,2)+matchedLongEnoughSegs(i,5))   = intersectionPointX;
         deltaLineAallpoints(matchedLongEnoughSegs(i,2)+matchedLongEnoughSegs(i,5)+1 )= intersectionPointY;
         deltaLineBallpoints(matchedLongEnoughSegs(i,4)+matchedLongEnoughSegs(i,6))   = intersectionPointX;
         deltaLineBallpoints(matchedLongEnoughSegs(i,4)+matchedLongEnoughSegs(i,6)+1) = intersectionPointY;         
         globalMap{1,matchedLongEnoughSegs(i,1)} = set( globalMap{1,matchedLongEnoughSegs(i,1)},'ss',deltaLineAallpoints);
         globalMap{1,matchedLongEnoughSegs(i,3)} = set( globalMap{1,matchedLongEnoughSegs(i,3)},'ss',deltaLineBallpoints);
         %plot( intersectionPointX,intersectionPointY,'rx','LineWidth',2,'MarkerSize',9);
     end;

end;
G = set(Gmap,'x',globalMap);

%debug
% figure(9);clf;hold on;
% for i=2:1:size(globalMap,2)
%     %points = get(globalMap{1,i},'ss');
%     testPoints = get( globalMap{1,i},'ss');
%     for j=1:4:size(testPoints,2)        
%         plot([testPoints(j) testPoints(j+2)],[testPoints(j+1) testPoints(j+3)],'g','LineWidth',2.7);
%         plot( testPoints(j),testPoints(j+1),'kx','LineWidth',2,'MarkerSize',9);
%         plot( testPoints(j+2),testPoints(j+3),'kx','LineWidth',2,'MarkerSize',9);                    
%     end;         
% end;

