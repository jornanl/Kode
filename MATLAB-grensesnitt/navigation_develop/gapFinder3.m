function c=gapFinder3( gMap , myMinGap,myMaxGap,robotPath,initGaps)

globalMap = get(gMap,'x'); %
gapPonits=[0 0 0 0];
numGaps = 0;

% only consider arline features
pointf=globalMap;
pointf(1)=[];
f=length(globalMap);
while f>1,
    type=get(globalMap{f},'Type');
    if ~strcmp(type,'alpha,r line feature'),
        globalMap(f)=[];
    else
        pointf(f-1)=[];   
    end
    f=f-1;
end


for i=2:length(globalMap)
    points  = get(globalMap{1,i},'ss');
    nums = size(points,2);
    
    if( mod(nums,4) == 0 )
        if( nums > 4),
            alphaRad= get(globalMap{1,i},'x' );
            angle = alphaRad(1);            
            kvasiRotationMatrix=[sin(-angle) cos(-angle)];
            
            counter=1;
            yVal =[];
            for p=1:4:nums            
                yVal(counter)= kvasiRotationMatrix*points(p:p+1)';
                yVal(counter+1)= kvasiRotationMatrix*points(p+2:p+3)';
                counter=counter+2;
            end;
            
            [sorted,ind]=sort(yVal);

            if( mod(numel(ind),2) == 0 )            
                for m=2:2:numel(ind)-1,                    
                     numGaps = numGaps+1;
                     %gapPonits(numGaps,m*2-3:1:m*2-2)=points(ind(m)*2-1:ind(m)*2);
                     %gapPonits(numGaps,m*2-1:1:m*2  )=points(ind(m+1)*2-1:ind(m+1)*2);
                     
                     %added by Trond M
                     gapPonits(numGaps,1:2)=points(ind(m)*2-1:ind(m)*2);
                     gapPonits(numGaps,3:4)=points(ind(m+1)*2-1:ind(m+1)*2);
                     
                     
                     % excluding too long gaps...i.e. gaps> 0.8 meter
                     e=[gapPonits(numGaps,3)-gapPonits(numGaps,1) gapPonits(numGaps,4)-gapPonits(numGaps,2)];
                     len=e*e';
                     if len > 0.8,
                         if numGaps==1,
                             gapPonits=[];
                         else
                             gapPonits=gapPonits(1:numGaps-1,1:4);
                         end
                     end
                     % end added
                end;
            else
                disp('gapFinder error 2')
            end;
        end;
    else
        disp('gapFinder error 1')
    end;
end;
%added by Trond M
segs=[];
numberOfLines=0;
beginEndLines=[];
for i = 2:length(globalMap),
    alphaRad=get(globalMap{1,i},'x');
    alpha=alphaRad(1); radius=alphaRad(2);
    beginEndLines(i-1,1) = radius*cos(alpha) - 30*sin(alpha);
    beginEndLines(i-1,2) = radius*sin(alpha) + 30*cos(alpha);
    beginEndLines(i-1,3) = radius*cos(alpha) + 30*sin(alpha);
    beginEndLines(i-1,4) = radius*sin(alpha) - 30*cos(alpha);
end;
%for i = 2:length(globalMap),
%    alphaRad=get(globalMap{1,i},'x')
%end
    
%if length(globalMap)>1,    
%alphaRad=get(globalMap{1,2:length(globalMap)},'x');
%alpha=alphaRad(1,:); radius=alphaRad(2,:);
%beginEndLines(2:length(globalMap),1) = radius(1,:)*cos(alpha(1,:)) - 30*sin(alpha(1,:));
%beginEndLines(2:length(globalMap),2) = radius(1,:)*sin(alpha(1,:)) + 30*cos(alpha(1,:));
%beginEndLines(2:length(globalMap),3) = radius(1,:)*cos(alpha(1,:)) + 30*sin(alpha(1,:));
%beginEndLines(2:length(globalMap),4) = radius(1,:)*sin(alpha(1,:)) - 30*cos(alpha(1,:));
%end

numberOfLines=size(beginEndLines,1);

for i=1:numberOfLines-1, % for all lines except the last one 
    for j=i+1:1:numberOfLines, 
        %intersect lines i and j?
        vectA=[ beginEndLines(i,1) beginEndLines(i,2) beginEndLines(i,3) beginEndLines(i,4) ];
        vectB=[ beginEndLines(j,1) beginEndLines(j,2) beginEndLines(j,3) beginEndLines(j,4) ];
        
        %returns 1=true or 2=false + intersection coordinates   
        [intersect intersectX intersectY] = vectorCrossingChecker(vectA,vectB); 
        
        
        
        
        if intersect==1, % line i and j do intersect
            segments_i  = get(globalMap{1,i+1},'ss');
            segments_j  = get(globalMap{1,j+1},'ss');
            
            for k=1:4:size(segments_i,2), % for all segments in line i
                segs(k,1:4,i)=segments_i(k:k+3); %copy start and end-points of segment k into segs
                
                % checking if the intersection is in segment k of line i 
                if ~((intersectX < segs(k,1,i) && intersectX < segs(k,3,i) ) ||...
                    (intersectX > segs(k,1,i) && intersectX > segs(k,3,i) ) ||...
                    (intersectY < segs(k,2,i) && intersectY < segs(k,4,i)) ||...
                    (intersectY > segs(k,2,i) && intersectY > segs(k,4,i))),
                
                    %disp('intersection is in segment k of line i:')
                    %disp(i);
                    currMin=Inf;
                    ind=-1;
                    for k=1:2:size(segments_j,2), % for all segments in line j
                        tmp(1:2)=segments_j(k:k+1); %copy points from segments
                        e=[tmp(1)-intersectX tmp(2)-intersectY];
                        len=e*e';
                        if( len<currMin ) 
                            currMin=len;
                            ind=k;
                        end;
                    end
                    if (currMin> myMinGap*myMinGap) && currMin~=Inf...
                            && (currMin< myMaxGap*myMaxGap),
                        
                        gapPonits((size(gapPonits,1)+1), 1:4)=[intersectX intersectY segments_j(ind) segments_j(ind+1)];
                    end
                    
                else
                    %disp('intersection is not in segment k of line i')

                end
                
                
            end

            for k=1:4:size(segments_j,2), % for all segments in line j
                segs(k,1:4,j)=segments_j(k:k+3); %copying start and end-points of segment k into segs
                
                % checking if the intersection is in segment k of line j 
                if ~((intersectX < segs(k,1,j) && intersectX < segs(k,3,j) ) ||...
                    (intersectX > segs(k,1,j) && intersectX > segs(k,3,j) ) ||...
                    (intersectY < segs(k,2,j) && intersectY < segs(k,4,j)) ||...
                    (intersectY > segs(k,2,j) && intersectY > segs(k,4,j))),
                    
                    %disp('intersection is in segment k of line j')
                    %disp(j);
                    currMin=Inf;
                    ind=-1;
                    for k=1:2:size(segments_i,2), % for all segments in line j
                        tmp(1:2)=segments_i(k:k+1); %copy points from segments
                        e=[tmp(1)-intersectX tmp(2)-intersectY];
                        len=e*e';
                        if( len<currMin ) 
                            currMin=len;
                            ind=k;
                        end;
                    end
                    if (currMin> myMinGap*myMinGap) && currMin~=Inf...
                            && (currMin< myMaxGap*myMaxGap),
                        
                        gapPonits((size(gapPonits,1)+1), 1:4)=[intersectX intersectY segments_i(ind) segments_i(ind+1)];
                    end
                    
                else
                    %disp('intersection is not in segment k of line j');

                end
                
                
            
            end
        else
            %disp('no intersection between line i and j');
            
        end
            
    end
end

c = gapPonits;
% delete 0-rows
c(~any(c,2),:)=[];

c=vertcat(c,initGaps);

% fetch segments
Xs = globalMap;
segs=[];
for i=2:length(Xs),
    ss=get(Xs{1,i},'ss');
    tmp=[];
    for j=1:4:size(ss,2),
        tmp=vertcat(tmp,ss(j:j+3));
    end
    
    for t=1:size(tmp,1),
    if tmp(t,1)> tmp(t,3), % sort with respect to x
        tmp(t,1:4)=[tmp(t,3) tmp(t,4) tmp(t,1) tmp(t,2)];
    end
    end
    segs(size(segs,1)+1:size(tmp,1)-1+size(segs,1)+1,:)=tmp;
end

% delete gaps which intersect with segsments (not working as intended...)
i=size(c,1);
%i=0;
while i>0 && size(segs,1)>0,
    interS = intersectEdges2(c(i,1:4), segs);
    if any(~isnan(interS(:,1))),
        c(i,:)=[];
    end
    i=i-1;
end


% delete gaps already crossed by robot
if size(robotPath,1)>1,
vectorPath=zeros(size(robotPath,1)-1,4); % preallocating memory
for p=1:size(robotPath,1)-1,
    vectorPath(p,1:4)=horzcat(robotPath(p,1:2),robotPath(p+1,1:2));
end


i=size(c,1);
while i>0,
    interS = intersectEdges2(c(i,1:4), vectorPath);
    if any(~isnan(interS(:,1))),
        c(i,:)=[];
    end
    i=i-1;
end
end