function [unknownGaps,edges]=linkPoints2(points,maxDist,draw)
%LINKPOINTS
%   edges=linkpoints2(points,maxDist,draw) calculate segments between points with
%   a threshold of maxDist between points. Points is [x, y] and 
%   edges is [x1 y1 x2 y2]. In the multidimension case points is a [N x 2] array and edges is a [N x 4] array.
%   Draw equal to 1 plot all concatenated segments.
%
% Trond Magnussen, November 2007.

unknownGaps=[];
edges=[];
if size(points,1)>0,

%unknownGaps=[];
n=size(points,1);
%clst=zeros(n,n);
%k=0;
%edgLen=0.25;
%maxDist=0;
%semiMaxInd=[0 0];
%indMax=[0 0];
%maxInd=[0 0];
%semiMaxDist=0;
ind=zeros(n-1,n-1);
for i=1:n-1,
    t=1;
    for j=i+1:n,
        
        dist=sqrt((points(i,1)-points(j,1))^2 + (points(i,2)-points(j,2))^2) ;
        if dist<maxDist,
            ind(i,t)=j;
            t=t+1;
            %clst(i,j)=1;
            %clst(j,i)=1;
            if points(i,1)<=points(j,1),
                edges=vertcat(edges,[points(i,1) points(i,2) points(j,1) points(j,2)]);
                    
            else % sort with respect to x values
                edges=vertcat(edges,[points(j,1) points(j,2) points(i,1) points(i,2)]);
            end   
        end
    
    end
end

% check matrix for ones

%ind=0;
i=0;
maxLocalDist=0;
maxLocalSeg=[];
%maxInd=[];
maxSegs=[];
a=zeros(n,n);
count=0;
for e=1:n-1,
    %clstArray(1)=e;
    
    %rawArray=find(clst(e,e+1:end)==1)+e;
    
    if count==0 || ~any(a(count,:)==e)
        % element e not in ind(e,:), start a new row in a
        count=count+1;
        a(count,1)=e;
        uv=length(find(ind(e,:)~=0)); % =2
        a(count,2:end)=ind(e,:);
       
    else
        uw=length(find(ind(e,:)~=0));
        if uw~=0,
            a(count,uv+1+1:uv+1+uw)=ind(e,1:uw);
            uv=uv+1;
        end
    end
end

notNullrows=length(find(a(:,1)~=0));
maxSegs=[];
for r=1:notNullrows,
    numPoints=length(find(a(r,:)~=0)); 
    maxLocalDist=0;
    
    for e=1:numPoints-1,
        for f=2:numPoints,
            tmpDist=sqrt( (points(a(r,e),1)-points(a(r,f),1))^2 + (points(a(r,e),2)-points(a(r,f),2))^2 );
            if tmpDist>maxLocalDist,
                maxLocalDist=tmpDist;
                maxLocalSeg=[points(a(r,e),1) points(a(r,e),2) points(a(r,f),1) points(a(r,f),2)];
                maxInd=[e, f];
                
                segLen = (maxLocalSeg(3)-maxLocalSeg(1))^2 + (maxLocalSeg(4)-maxLocalSeg(2))^2;
                % if segment too long, split it up 
                if segLen>0.6,
                    maxLocalSeg1=[points(a(r,1),1) points(a(r,1),2) points(a(r,round(f/2)),1) points(a(r,round(f/2)),2)];
                    maxLocalSeg2=[points(a(r,round(f/2)),1) points(a(r,round(f/2)),2) points(a(r,f),1) points(a(r,f),2)];
                    
                    maxLocalSeg=[];
                    maxLocalSeg=vertcat(maxLocalSeg1,maxLocalSeg2);
                end
                
            end
        end
    end
    
    
    
    maxSegs=vertcat(maxSegs,maxLocalSeg);
end
        
unknownGaps=maxSegs;    


    

% debug
%figure(11);clf;hold on;axis equal;
if draw,
for p=1:size(edges,1),
    plot([edges(p,1) edges(p,3)],[edges(p,2) edges(p,4)],'y-');
    plot( edges(p,1),edges(p,2),'yx');
    plot( edges(p,3),edges(p,4),'yx');
end
for q=1:size(unknownGaps,1),
    plot([unknownGaps(q,1) unknownGaps(q,3)],[unknownGaps(q,2) unknownGaps(q,4)],'m-');
    plot( unknownGaps(q,1),unknownGaps(q,2),'mx');
    plot( unknownGaps(q,3),unknownGaps(q,4),'mx');
end
end
end
