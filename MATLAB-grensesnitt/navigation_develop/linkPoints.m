function [edges]=linkPoints(points,maxDist,draw)
%LINKPOINTS
%   edges=linkpoints(points,maxDist,draw) calculate segments between points with
%   a threshold of maxDist between points. Points is [x, y] and edges is [x1
%   y1 x2 y2]. In the multidimension case points is a [N x 2] array and edges is a [N x 4] array.
%   Draw equal to 1 plot all concatenated segments.
%
% Trond Magnussen, November 2007.


edges=[];
n=size(points,1);
for i=1:n-1,
   
    for j=i+1:n,
        
        dist=sqrt((points(i,1)-points(j,1))^2 + (points(i,2)-points(j,2))^2) ;
        if dist<maxDist,
            if points(i,1)<=points(j,1),
                edges=vertcat(edges,[points(i,1) points(i,2) points(j,1) points(j,2)]);
            else % sort with respect to x
                edges=vertcat(edges,[points(j,1) points(j,2) points(i,1) points(i,2)]);
            end
            
        end
        
    end
    
end
% debug
%figure(11);clf;hold on;
if draw,
for s=1:size(edges,1),
    startnstopp=edges;
    j=s;
    plot([startnstopp(j,1) startnstopp(j,3)],[startnstopp(j,2) startnstopp(j,4)],'LineWidth',2.7,'Color',[1 0 0]);
    plot( startnstopp(j,1),startnstopp(j,2),'kx','LineWidth',2,'MarkerSize',9);
    plot( startnstopp(j,3),startnstopp(j,4),'kx','LineWidth',2,'MarkerSize',9);       
end    
end