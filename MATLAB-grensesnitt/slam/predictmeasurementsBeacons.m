function [Gpred,Hpred] = predictmeasurementsBeacons(Gin,oldGpred,oldHpred)
%PREDICTMEASUREMENTS Transforms map features into the local frame.
%   [GPRED,HPRED] = PREDICTMEASUREMENTS(G) applies the observation 
%   function to all features in the map by transforming them from
%   the global coordinate frame into the local frame given the
%   current robot pose. Returns the predicted measurements in the
%   map GPRED and the structure HPRED with the individual measure-
%   ment Jacobians:
%      HPRED(i).HR: measurement Jacobian of the i-th feature with
%                   respect to the robot
%      HPRED(i).HM: measurement Jacobian of the i-th feature with
%                   respect to the map feature
%
%   The function iterates over the map features in G and calls
%   the PREDICT method of the feature objects.
%
%   See also POINTFEATURE/PREDICT, ARLINEFEATURE/PREDICT.

% v.1.0, Kai Arras, Nov. 2003, CAS-KTH


% Get data from Gin
X  = get(Gin,'x');  
C  = get(Gin,'c');
n  = length(X);
% Get robot and its pose
r  = getrobot(Gin); %same as "get(X{1})"
xr = get(r,'x');
Cr = get(r,'C');

for i = 2:n,
    xyFeature(1:2,i-1)=get(X{i},'x'); % xy=[2x1]
end

m=0;
if n>1
distVector=sqrt((xyFeature(1,1:n-1)-xr(1)).^2 + (xyFeature(2,1:n-1)-xr(2)).^2);
tmpIndex1 = find(distVector<=0.6);
tmpIndex2 = find(distVector>0.6); 
%cT_lastRow=cT(cTsz,:);
%cT(cTsz,:)=[]; % delete last row
%for j=1:length(tmpIndex),
%   if ~any(tmpIndex(j)==cT(:,1)), % if point not in cT, append it
%       cT(size(cT,1)+1,:)=[tmpIndex(j) ones(1,NOP)];
%   end
%end
%cT(size(cT,1)+1,:) = cT_lastRow(:);
%cTsz=size(cT,1);
m=length(tmpIndex1);

end


if n > 1,
  % init new map
  %Gpred = map('measurement prediction map',0);
    Gpred=oldGpred;
    Xpred  = get(Gpred,'x');
  % traverse Gin, fill in Gpred and Hpred
  
  for i = 1:m,      
    
      
    Crp = C(1,tmpIndex1(i)+1).C;
    [pr,Hr,Hm] = predict(X{tmpIndex1(i)+1},xr,Cr,Crp);
    % store measurement Jacobians at index i-1
    Hpred(tmpIndex1(i)+1-1).Hr = Hr;
    Hpred(tmpIndex1(i)+1-1).Hm = Hm;
    % add to map: makes it consistent with C
    % and fills in zeros in the off-diagonal
    %Gpred = addentity(Gpred, pr);
    Xpred{tmpIndex1(i)}=pr;
    Gpred=set(Gpred,'x',Xpred);
  end;
  if ~isempty(tmpIndex2),
  %X=get(oldGpred,'x');
  for q=1:length(tmpIndex2),
     Hpred(1,tmpIndex2(1,q)).Hr = oldHpred(1,tmpIndex2(1:q)).Hr; 
     Hpred(1,tmpIndex2(1,q)).Hm = oldHpred(1,tmpIndex2(1:q)).Hm;
     %oldGpred;
     %Gpred;
     
     %feature=get(X{tmpIndex2(q)},'x');
     
     %Gpred=addentity(Gpred,X{1:length(tmpIndex2)});
  end
  end
  
  
  
else
  Gpred = []; Hpred = [];
end;