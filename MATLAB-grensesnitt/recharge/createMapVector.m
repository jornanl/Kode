 function mapVector = createMapVector(globalMap)
%CREATEMAP Summary of this function goes here
%   Detailed explanation goes here
mapVector = [];
for i=2:length(globalMap)        
    startnstopp=get(globalMap{1,i},'ss');
    for j=1:4:size(startnstopp,2)    
        mapVector = [mapVector startnstopp];     
   end;
end;
end

