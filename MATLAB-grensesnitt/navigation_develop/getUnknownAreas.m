function cT=getUnknownAreas(G,myHandles,robotPath,edges)
%CT=GETUNKNOWNAREAS  calculate unknown robot areas
%  CT=GETUNKNOWNAREAS(G, myHandles, robotPath, edges) extract linesegments
%  from G and merge them with edges. The function uses this information
%  along with robotPath to produce a check table cT which is returned. cT
%  is [n x m+1] matrix where n is the number robotposes which has a border
%  to an unknown area, m is angleresolution of the range sensors. 
%
%  E.g. 
%   cT=
%  [ 2 1 1 1 1 0 0;
%    4 1 1 0 0 1 1;
%    5 1 0 1 1 1 1;
%   13 0 1 1 1 1 1] 
%               
%   meaning n=4 robotpositions is representing the total unknown areas where
%   the robot has travelled so far. Angleresolution is 360 deg/m=360
%   deg/6=60 deg, ie. sensors are measuring every 60th degree. At robotpose
%   2, there is an unknown area between angle 300 degrees (or 5*2*pi/6) and
%   angle 360 degrees (or 2*pi). At pose 4, unknown area is from angle
%   3*2*pi/6 and angle 4*2*pi/6, and so on. Generally in cT, except the first
%   column, a 0 is indicating an unknown area for a spesical angle at a
%   robotpose, a 1 is indicating an angle covered either by landmarks
%   (features) or perception circles of other robotposes. A robotpose is
%   deleted from cT when all its anglevalues in cT are zeros. All 1's are
%   plotted as yellow marks in the global map.
%
% See also LINEBEACONSLAM, ISCIRLE

% v. 1.0, Trond Magnussen, Nov 2007, NTNU


% split up segments and points
    Gs=G;
    Gp=G;
    X = get(G,'x');
    C = get(G,'c');
    
    Xs=X;
    Cs=C;
    Xp=X;
    Cp=C;
    
    f=length(X);
    while f>1,
        type=get(X{f},'Type');
        if ~strcmp(type,'alpha,r line feature'),
            Xs(f)=[];
            %update covarians matrix 
            Cs(f,:)=[];
            Cs(:,f)=[];
            
        else
            Xp(f)=[];
            %update covarians matrix
            Cp(f,:)=[];
            Cp(:,f)=[];
        end
        f=f-1;
    end
    Gs=set(Gs,'x',Xs,'c',Cs);
    Gp=set(Gp,'x',Xp,'c',Cp);


% fetch segments
Xs = get(Gs,'x');
segs=[];
for i=2:length(Xs),
    ss=get(Xs{1,i},'ss');
    tmp=[];
    for j=1:4:size(ss,2),
        %if ~(ss(j:j+3)==[0 0 0 0]),
        tmp=vertcat(tmp,ss(j:j+3));
        
    end
    
    for t=1:size(tmp,1),
    if tmp(t,1)> tmp(t,3), % sort with respect to x
        tmp(t,1:4)=[tmp(t,3) tmp(t,4) tmp(t,1) tmp(t,2)];
    end
    end
    
    segs=vertcat(tmp,segs);
   
end

% merge edges and segments
segs=vertcat(segs,edges);


% fetch points
Xp = get(Gp,'x');
pt=zeros(2,length(Xp)-1);
for i=2:length(Xp)
    pt(:,i-1)=get(Xp{1,i},'x');
end    



rad=myHandles.maxPerceptRadius; 
%rad=0.6;
res=myHandles.angularRes; %
%res=3;
NOP=360/res;


lenRobotPath=size(robotPath,1); % # of robotpositions
cT=myHandles.checkTable;

cTsz=size(cT,1);
if cTsz==0, % initialisation of checkTable
    cT=[1,zeros(1,NOP)];
else
    cT(cTsz+1,1)=lenRobotPath; % append last pose to checkTable
end
cTsz=size(cT,1); % number of interesting positions fetched from robotPath



X=robotPath(lenRobotPath,1); % X= x-coordinate to last/newest position in robotPath  
Y=robotPath(lenRobotPath,2); % Y= x-coordinate to last/newest position in robotPath


THETA=linspace(0,2*pi,NOP); % producing row vector with angles which we consider..



%% checking for segments inside circles

% for all positions in cT check if new segments are in perception radius of
% the current position and update cT with new data
c=cTsz;

while c>0 && size(segs,1)>0,
% extracting data from segs 
x1=segs(:,1);
y1=segs(:,2);
x2=segs(:,3);
y2=segs(:,4);
dx=x2-x1;
dy=y2-y1;
dr = sqrt( dx.^2 + dy.^2 ); % col vector with length of all segs    
    
X=robotPath(cT(c,1),1);
Y=robotPath(cT(c,1),2);
    
D = (x1-X).*(y2-Y) - (x2-X).*(y1-Y); % determinant = |x1 x2|
                                     %               |y1 y2|
if D(:)==0,
    disp( 'navigation_develop/UnknownAreaFinder: divide by zero');
end
delta = rad^2*dr.^2 - D.^2;
warning( 'OFF');%,msgID);
u=( (X-x1).*dx + (Y-y1).*dy ) ./ dr.^2;
warning('ON');%,msgID);
u(u<0)=0;
u(u>1)=1;

closest=[x1 y1] + [u u].*[dx dy];
dist=sqrt(  (X*ones(size(closest,1),1) - closest(:,1)).^2 ...
          + (Y*ones(size(closest,1),1) - closest(:,2)).^2 );

indexSegs= dist<=rad;
indexSegsNum=find(indexSegs); % indices of segments intersecting the circle around (X,Y) position
segsInt=segs(indexSegsNum,:); % update segs with the new segments of interest

% produce three cartesian points which define the circle around (X,Y)
% will be used in the function "iscircle"
THETA_3=linspace(0,pi,3); % THETA_3=[0, pi/2, pi]
RHO_3=ones(1,3)*rad;    % RHO_3 = [0.6 0.6 0.6]
[X_3,Y_3] = pol2cart(THETA_3,RHO_3); % X_3=[0.6 0 -0.6] Y_3=[0 0.6 0]
X_3=X_3 + X; % locate the points with referance to global frame 
Y_3=Y_3 + Y;

% use (X_3,Y_3) to check the properties of intersections
inside1=iscircle(X_3,Y_3,segsInt(:,1),segsInt(:,2)); % checking....
pose1=(inside1==1); 
inside2=iscircle(X_3,Y_3,segsInt(:,3),segsInt(:,4)); % checking....
pose2=(inside2==1); 
pose=horzcat(pose1,pose2);

% for all segments check intersection properties with circle around (X,Y)
% position
for k=1:size(segsInt,1),
if all(pose(k,1:2)), % both endpoints of segment is inside circle
    phiSegs=cart2pol(segsInt(k,1:2:3)-X ,segsInt(k,2:2:4)-Y); % phiSegs in radians [-pi,pi]
    
    if phiSegs(1)<0,
        phiSegs(1)=phiSegs(1)+2*pi;
    end
    if phiSegs(2)<0,
        phiSegs(2)=phiSegs(2)+2*pi;
    end
    
    phiSegs=sort(phiSegs);
    if phiSegs(2)-phiSegs(1)<=pi,
        cross=find(phiSegs(1)<THETA & THETA<phiSegs(2) );
    elseif phiSegs(2)-phiSegs(1)>pi,
        cross=find(phiSegs(2)<THETA | THETA<phiSegs(1) );
    else
        disp('navigation_develop/UnknownAreaLines: program error.....')
    end
    cT(c,cross+1)=1; % update cT   
    
elseif any(pose(k,1:2)), % one endpoint of segment is inside circle
    if pose(k,1),
        phiSeg1=cart2pol(segsInt(k,1)-X,segsInt(k,2)-Y);
    elseif pose(k,2),
        phiSeg1=cart2pol(segsInt(k,3)-X,segsInt(k,4)-Y);
    else
        disp('navigation_develop/UnknownAreaLines: program error.....')
    end
    
    if dy(indexSegsNum(k))~=0,
        sign_dy=sign(dy(indexSegsNum(k)));
    end  
    if dy(indexSegsNum(k))==0,
        sign_dy=1;
        x_1=(-sign_dy*dx(indexSegsNum(k))*sqrt(delta(indexSegsNum(k)))) / (dr(indexSegsNum(k)))^2 + X;
        x_2=(sign_dy*dx(indexSegsNum(k))*sqrt(delta(indexSegsNum(k)))) / (dr(indexSegsNum(k)))^2 + X;
    
        y_1=(-D(indexSegsNum(k))*dx(indexSegsNum(k))) / (dr(indexSegsNum(k)))^2 + Y;
        y_2=y_1;
        
        if ( segsInt(k,1) <= x_1 && x_1 <= segsInt(k,3))
            sol=[x_1, y_1];
        else
            sol=[x_2, y_2];
        end
    
    elseif dx(indexSegsNum(k))==0,
        x_1=( D(indexSegsNum(k))*dy(indexSegsNum(k)) ) / (dr(indexSegsNum(k)))^2 + X;
        x_2=x_1;
        y_1=(-abs(dy(indexSegsNum(k)))*sqrt(delta(indexSegsNum(k))) ) / (dr(indexSegsNum(k)))^2 + Y;
        y_2=(abs(dy(indexSegsNum(k)))*sqrt(delta(indexSegsNum(k))) ) / (dr(indexSegsNum(k)))^2 + Y;
        
        if ( segsInt(k,2) < segsInt(k,4) ),
            if ( segsInt(k,2) <= y_1 && y_1 <= segsInt(k,4))
                sol=[x_1, y_1];
            else
                sol=[x_2, y_2];
            end
        else
            if ( segsInt(k,2) <= y_1 && y_1 <= segsInt(k,4))
                sol=[x_2, y_2];
            else
                sol=[x_1, y_1];
            end
        end
    else
        x_1=(D(indexSegsNum(k))*dy(indexSegsNum(k)) - sign_dy*dx(indexSegsNum(k))*sqrt(delta(indexSegsNum(k)))) / (dr(indexSegsNum(k)))^2 + X;
        x_2=(D(indexSegsNum(k))*dy(indexSegsNum(k)) + sign_dy*dx(indexSegsNum(k))*sqrt(delta(indexSegsNum(k)))) / (dr(indexSegsNum(k)))^2 + X;
    
        y_1=(-D(indexSegsNum(k))*dx(indexSegsNum(k)) - abs(dy(indexSegsNum(k)))*sqrt(delta(indexSegsNum(k))) ) / (dr(indexSegsNum(k)))^2 + Y;
        y_2=(-D(indexSegsNum(k))*dx(indexSegsNum(k)) + abs(dy(indexSegsNum(k)))*sqrt(delta(indexSegsNum(k))) ) / (dr(indexSegsNum(k)))^2 + Y;
        
        if ( segsInt(k,1) <= x_1 && x_1 <= segsInt(k,3))
            sol=[x_1, y_1];
        else
            sol=[x_2, y_2];
        end
    end
    
    phiSeg2=cart2pol(sol(1)-X,sol(2)-Y);
    phiSegs=[phiSeg1,phiSeg2];
    
    if phiSegs(1)<0,
        phiSegs(1)=phiSegs(1)+2*pi;
    end
    if phiSegs(2)<0,
        phiSegs(2)=phiSegs(2)+2*pi;
    end
    
    phiSegs=sort(phiSegs);
    if phiSegs(2)-phiSegs(1)<=pi,
        cross=find(phiSegs(1)<THETA & THETA<phiSegs(2) );
    elseif phiSegs(2)-phiSegs(1)>pi,
        cross=find(phiSegs(2)<THETA | THETA<phiSegs(1) );
    else
        disp('navigation_develop/UnknownAreaLines: program error.....')
    end
    cT(c,cross+1)=1; % setting angles to crossing, i.e. they will not be considered in the future

else % none of the endpoints are inside circle
    if dy(indexSegsNum(k))~=0,
        sign_dy=sign(dy(indexSegsNum(k)));
    end
       
    if dy(indexSegsNum(k))==0,
        sign_dy=1;
        x_1=(-sign_dy*dx(indexSegsNum(k))*sqrt(delta(indexSegsNum(k)))) / (dr(indexSegsNum(k)))^2 + X;
        x_2=(sign_dy*dx(indexSegsNum(k))*sqrt(delta(indexSegsNum(k)))) / (dr(indexSegsNum(k)))^2 + X;
    
        y_1=(-D(indexSegsNum(k))*dx(indexSegsNum(k))) / (dr(indexSegsNum(k)))^2 + Y;
        y_2=y_1;
    
    elseif dx(indexSegsNum(k))==0,
        x_1=(D(indexSegsNum(k))*dy(indexSegsNum(k))) / (dr(indexSegsNum(k)))^2 + X;
        x_2=x_1;
        y_1=(-abs(dy(indexSegsNum(k)))*sqrt(delta(indexSegsNum(k))) ) / (dr(indexSegsNum(k)))^2 + Y;
        y_2=(abs(dy(indexSegsNum(k)))*sqrt(delta(indexSegsNum(k))) ) / (dr(indexSegsNum(k)))^2 + Y;
    else
        x_1=(D(indexSegsNum(k))*dy(indexSegsNum(k)) - sign_dy*dx(indexSegsNum(k))*sqrt(delta(indexSegsNum(k)))) / (dr(indexSegsNum(k)))^2 + X;
        x_2=(D(indexSegsNum(k))*dy(indexSegsNum(k)) + sign_dy*dx(indexSegsNum(k))*sqrt(delta(indexSegsNum(k)))) / (dr(indexSegsNum(k)))^2 + X;
    
        y_1=(-D(indexSegsNum(k))*dx(indexSegsNum(k)) - abs(dy(indexSegsNum(k)))*sqrt(delta(indexSegsNum(k))) ) / (dr(indexSegsNum(k)))^2 + Y;
        y_2=(-D(indexSegsNum(k))*dx(indexSegsNum(k)) + abs(dy(indexSegsNum(k)))*sqrt(delta(indexSegsNum(k))) ) / (dr(indexSegsNum(k)))^2 + Y;
    end

    phiSegs=cart2pol([x_1-X;x_2-X],[y_1-Y;y_2-Y]);
    if phiSegs(1)<0,
        phiSegs(1)=phiSegs(1)+2*pi;
    end
    if phiSegs(2)<0,
        phiSegs(2)=phiSegs(2)+2*pi;
    end
    
    phiSegs=sort(phiSegs);
    if phiSegs(2)-phiSegs(1)<=pi,
        cross=find(phiSegs(1)<THETA & THETA<phiSegs(2) );
    elseif phiSegs(2)-phiSegs(1)>pi,
        cross=find(phiSegs(2)<THETA | THETA<phiSegs(1) );
    else
        disp('navigation_develop/UnknownAreaLines: program error.....')
    end
        
    cT(c,cross+1)=1; % setting angles to crossing, i.e. they will not be considered in the future
   
end
end
c=c-1;

end

%% Checking for points inside circle

%---- NOT ACTIVE part----%
pt=[];
%--- Trond M. November 2007


c=cTsz;

if c>0 && size(pt,2)>0, 
    
X=robotPath(cT(c,1),1);
Y=robotPath(cT(c,1),2);

% produce three cartesian points which define the circle around (X,Y)
% will be used in the function "iscircle"
THETA_3=linspace(0,pi,3); % THETA_3=[0, pi/2, pi]
RHO_3=ones(1,3)*rad;    % RHO_3 = [0.6 0.6 0.6]
[X_3,Y_3] = pol2cart(THETA_3,RHO_3); % X_3=[0.6 0 -0.6] Y_3=[0 0.6 0]
X_3=X_3 + X; % locate the points with referance to global frame 
Y_3=Y_3 + Y;

inside=iscircle(X_3,Y_3,pt(1,:),pt(2,:)); % checking....
    
ptInside=pt(:,inside==1)';
npti=size(ptInside,1);

[phiPoints,distToPoints]=cart2pol(ptInside(1:npti,1)-X,ptInside(1:npti,2)-Y);

ANGLEPARAMETER=2;
deltaPt=(ANGLEPARAMETER./distToPoints)*(pi/180);
phiPoints=[phiPoints-deltaPt  phiPoints+deltaPt];
    
phiPoints=phiPoints + 2*pi*(phiPoints<0);

phiPoints=sort(phiPoints,2);

for in=1:npti, 
    if phiPoints(in,2)-phiPoints(in,1)<=pi,
        cross=find(phiPoints(in,1)<THETA & THETA<phiPoints(in,2) );
    elseif phiPoints(in,2)-phiPoints(in,1)>pi,
        cross=find(phiPoints(in,2)<THETA | THETA<phiPoints(in,1) );
    else
        disp('navigation_develop/UnknownAreaLines: program error.....')
    end
    cT(c,cross+1)=1; % update cT  
end

end

%% Checking for circle inside circle
% 
% $$e^{\pi i} + 1 = 0$$
% 

X=robotPath(lenRobotPath,1); % X= x-coordinate to last/newest position in robotPath  
Y=robotPath(lenRobotPath,2); % Y= x-coordinate to last/newest position in robotPath

% create three points, (X3,Y3), defining a circle with center in [X, Y] and
% radius twice the length of perception radius
THETA3=linspace(0,pi,3); % THETA3=[0, pi/2, pi]
RHO3=ones(1,3)*2*rad;    % RHO3 = [1x3]
[X3,Y3] = pol2cart(THETA3,RHO3); % X3=[1x3] Y3=[1x3]
X3=X3 + X; % locate center of circle with referance to global frame 
Y3=Y3 + Y;

% checking old postions...
X=robotPath(lenRobotPath,1); % X= x-coordinate to last/newest position in robotPath  
Y=robotPath(lenRobotPath,2); % Y= x-coordinate to last/newest position in robotPath
cTsz=size(cT,1);

% distance from all robotpoints to newest position in robotPath
distVector=sqrt((robotPath(1:lenRobotPath-1,1)-X).^2 + (robotPath(1:lenRobotPath-1,2)-Y).^2);

tmpIndex = find(distVector<=2*rad);
cT_lastRow=cT(cTsz,:);
cT(cTsz,:)=[]; % delete last row
for j=1:length(tmpIndex),
   if ~any(tmpIndex(j)==cT(:,1)), % if point not in cT, append it
       cT(size(cT,1)+1,:)=[tmpIndex(j) ones(1,NOP)];
   end
end
cT(size(cT,1)+1,:) = cT_lastRow(:);
cTsz=size(cT,1);


% find elements in cT that are inside (X3,Y3)
x_all=robotPath(cT(1:cTsz-1,1),1); % col vector with x-coordinates to all elements in cT except last pose
y_all=robotPath(cT(1:cTsz-1,1),2); % col vector with y-coordinates to all elements in cT except last pose
inside=iscircle(X3,Y3,x_all,y_all); % checking....
pose=find(inside==1); % extracting elements inside (X3,Y3)
x=robotPath(cT(pose,1),1); % col vector with x-coordinates to elements of cT inside (X3,Y3)
y=robotPath(cT(pose,1),2); % col vector with y-coordinates to elements of cT inside (X3,Y3)

% for all elements inside (X3,Y3), update cT
len=length(pose);
i=len;
while(i>0),
    [phi,ds]=cart2pol(X-x(i),Y-y(i)); % phi in radians [-pi,pi]
    if ds==0, % overlapping circles, new circle takes overrides the old one
        notCrossing=cT(cTsz,2:NOP+1)==0;
        num=1:NOP;
        num=num(notCrossing);
        cT(pose(i),num+1)=1;
        
        i=i-1;
        continue
    end
        
    % calculation of angles for use in cT
    theta=acos((ds^2)/(2*ds*rad)); % theta in radians [0,pi]
    
    
    if phi<0,phi=phi+2*pi;end
    theta1=phi-theta;
    theta2=phi+theta;
    if theta1<0,
        theta1=theta1+2*pi;
    elseif theta1>2*pi
        theta1=theta1-2*pi;
    end
    if theta2<0,
        theta2=theta2+2*pi;
    elseif theta2>2*pi
        theta2=theta2-2*pi;
    end
    if theta1<theta2,
        cross=find(theta1<THETA & THETA<theta2 );
    else 
        cross=find(theta1<THETA | THETA<theta2 );
    end
    
    cT(pose(i),cross+1)=1; % setting angles to crossing, i.e. they will not be considered in the future
    

    % phi2 = angle between (X,Y) and (x(i),y(i)) seen from (X,Y)
    phi2=cart2pol(x(i)-X,y(i)-Y);
    if phi2<0,phi2=phi2+2*pi;end
    theta1=phi2-theta;
    theta2=phi2+theta;
    if theta1<0,
        theta1=theta1+2*pi;
    elseif theta1>2*pi
        theta1=theta1-2*pi;
    end
    if theta2<0,
        theta2=theta2+2*pi;
    elseif theta2>2*pi
        theta2=theta2-2*pi;
    end
    
    if theta1<theta2,
        cross=find(theta1<THETA & THETA<theta2 );
    else 
        cross=find(theta1<THETA | THETA<theta2 );
    end
    cT(cTsz,cross+1)=1; % setting angles to crossing, i.e. they will not be considered in the future   

    i=i-1;
end



%% Plotting resultes
not=cT(cTsz,2:size(cT,2)) ==0;
val=THETA(not);
plot( X(size(X,1))+cos(val)*rad , Y(size(Y,1))+sin(val)*rad ,'bx');

%  delete rows with all 1's, ie. positions which are totally in a known area
%lastDeleted=0;
%if all(cT(cTsz,2:size(cT,2)),2),
%    lastDeleted=1;
%end
del=all(cT(1:cTsz,2:size(cT,2)),2);
cT(del,:) = [];

% delete positions which previusly has been recorded and later deleted
del = sort(del(:)); 
del(find(del((1:end-1)')==del((2:end)'))) = []; % inlining the unique function 



%cTsz=size(cT,1);
%for k=1:cTsz-1+lastDeleted, % draw all cT elements except the last
%    free=cT(k,2:size(cT,2))==0;
%    val=THETA(free);
    
%    plot(cos(val)*rad+robotPath(cT(k,1),1),sin(val)*rad+robotPath(cT(k,1),2),'yx','LineWidth',2);
%end
