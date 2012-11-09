%ADDENTITY Add map entity object to map.
%   M = ADDENTITY(M,E) appends map entity E to map M and
%   extends the map covariance matrix by filling in zeros
%   in the new row and column belonging to E.
%
%   See also ENTITY, MAP.

% v.1.0, Nov. 2003, Kai Arras, CAS-KTH

% v.2.0, Nov. 2007, Trond Magnussen
% extending function to add multiple entities at the same time
function m = addentity(m, e)

if isa(m,'map') && isa(e,'entity')
%if isa(m,'map'), 
%    ne=length(es);
%    for n=1:ne,
%        if ~isa(es(n),'entity'),
%            disp('addentity: Wrong input. Check your arguments');
%            break;
%        end
%    end
  
%    C=get(m,'c');
%    Xnew=horzcat(m.X,es);
%    
%    Ce=get(es,'c');
%    dimc = length(Ce);
%    totalL=size(C,2)+dimc;
%    Cnew=C;
%    %if size(Ce,2)>0,
%        % add new covarianse
%        Cnew(size(C,2)+1:totalL,size(C,2)+1:totalL) = Ce;
%        
%        % fill up new block matrices with zeros
%        for i=size(C,2)+1:totalL,
%            Cnew(i,1).C=eye(2,3)*zeros(3,3);
%            Cnew(1,i).C=zeros(3,3)*eye(3,2);
%            for j=2:size(C,2),
%                Cnew(j,i).C=zeros(2,2);
%                Cnew(i,j).C=zeros(2,2);
%            end 
%        end
%    %end
    % put all features back to map
%    m=set(m,'x',Xnew,'c',Cnew);
  
  % extend state vector and matrix
  n = length(m.X);
  m.X{n+1} = e;
  Ce = get(e,'c');
  m.C(n+1,n+1).C = Ce;
  
  % fill up new block matrices with zeros
  dimc = length(Ce);
  for i = 1:n,
    dimr = length(get(m.X{i},'x'));
    m.C(i,n+1).C = zeros(dimr, dimc);
    m.C(n+1,i).C = m.C(i,n+1).C';
  end;
  
else
  disp('addentity: Wrong input. Check your arguments');
end;