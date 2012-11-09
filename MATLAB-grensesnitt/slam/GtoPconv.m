function [P, G]=GtoPconv(Gin,handles)
% extract a perception map P from global map G based on robotposition and
% range of sensors
% 
PERCRAD=str2double(get(handles.maxPerceptRadius,'String'));
%angularRes=str2double(get(handles.angularRes,'String'));

%PERCRAD=0.5;

% Get data from Gin
Xin  = get(Gin,'x');  
Cin  = get(Gin,'c');

% Get robot and its pose
r  = getrobot(Gin); 
xr = get(r,'x');
%Cr = get(r,'C');

% init new map P
P = map('perception map',0);

X=Xin;
C=Cin;

% add robot object to P and remove it from Gin
P=addentity(P,r);
%X(1)=[];
%C(1,:)=[];
%C(:,1)=[];

n  = length(X); % number of features in Gin

% if features in Gin, extract features located inside radius and store them in P. Also
% remove features inside radius from Gin
if n>1,
    for i = 2:n,
        xyFeature(1:2,i-1)=get(X{i},'x'); % xyFeature=[2x1] contains (x,y) values of all beacons in G 
    end
    
    % finding beacons inside perception radius
    distVector=sqrt((xyFeature(1,1:n-1)-xr(1)).^2 + (xyFeature(2,1:n-1)-xr(2)).^2);
    insideIndex = find(distVector<=PERCRAD);
    outsideIndex = find(distVector>PERCRAD);
    
    

    % delete features outside radius
    Xinside=X;
    Xinside(outsideIndex(1:end)+1)=[];

    % add features inside radius to P
    for i=2:length(Xinside)
        P=addentity(P,Xinside{i});
    end


    % delete features inside radius
    Xoutside=X;
    Xoutside(insideIndex(1:end)+1)=[];

    %update covarians matrix 
    C(insideIndex(1:end)+1,:)=[];
    C(:,insideIndex(1:end)+1)=[];
    
    % update Gin and produce output G
    Gin=set(Gin,'x',Xoutside,'c',C);
    G=Gin;


else
    G=Gin;
end