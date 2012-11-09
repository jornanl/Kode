function mapVectorPoints = createMapVectorPoints(globalMap1)
%CREATEMAP Summary of this function goes here
%   Detailed explanation goes here
mapVectorPoints = [];
for i=2:length(globalMap1)        
    mapVectorPoints(i-1,1:2)=get(globalMap1{1,i},'x');
end
end
