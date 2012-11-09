function G=PtoGconv(Pin,Gin)
% extract a perception map P from global map G based on robotposition and
% range of sensors
% 

% Get data from input
Xp = get(Pin,'x');  
Xg = get(Gin,'x');  
%C  = get(Gin,'c');
np  = length(Xp);
ng  = length(Xg);
% Get robot and its pose
%r  = getrobot(Pin); 
%xr = get(r,'x');
%Cr = get(r,'C');


% overwrite robotobject in Gin with robotobject from Pin
Xg{1}=Xp{1};
Gin=set(Gin,'x',Xg);

% if there is features in Pin, add them to Gin
if np>1
    
    % update feature id's
    ag=zeros(1,1000);
    ap=zeros(1,1000);
    for i=2:length(Xg),
        ag(i-1)=get(Xg{i},'id');    
    end
    ag=ag(1:length(Xg)-1);
    for i=2:length(Xp),
        ap(i-1)=get(Xp{i},'id');    
    end
    ap=ap(1:length(Xp)-1);
    a=horzcat(ag,ap);
    
    b = sort(a(:));
    % c=find(b((1:end-1)')==b((2:end)'));
    c=find(b((1:end-1)')==b((2:end)'), 1);
    if ~isempty(c),
        maxElement=max(b);
        d=length(a)-maxElement;
        for e=1:d
            Xp{maxElement+e-length(ag)+1}=set(Xp{maxElement+e-length(ag)+1},'id',maxElement+e); 
        end
    end

    
    % add all features in Pin to Gin
    for i=2:np
        Gin=addentity(Gin,Xp{i});
    end

    G=Gin;
else
    G=Gin;

end