function myIr=simMap(mySimParams)
%SIMMAP 
%   This function returns artificial IR-measurements according to chosen
%   maze, startposition, current position and radarangle


global REALPOSE;

% extracting simulation primitives
if mySimParams.poseError,
    
    x=REALPOSE(1);
    y=REALPOSE(2);
    theta=mySimParams.theta;
else
    x=mySimParams.x;
    y=mySimParams.y;
    theta=mySimParams.theta;
end

s=mySimParams.radar;
maze=mySimParams.maze;
startX=mySimParams.startSimX;
startY=mySimParams.startSimY;
b=mySimParams.widthBetweenWalls;
perceptRad=mySimParams.maxPerceptRadius;




radarOffset=6; % [cm]



u=(0.7)*pi/180; % error when measuring parallel walls using ir sensor [in radians]
if mySimParams.irError, 
    a=(90+theta-s);
    radA=pi*a/180;
    if ~(radA==u || radA-pi-u==0 || radA-pi/2-u==0 || radA+pi/2-u==0)
        R = [(b-x)*sin(u)*tan(radA)/(sin(radA-pi/2-u)),...
        y*sin(u)*tan(3*pi/2-radA)/(sin(radA-pi-u)),...
        x*sin(u)*tan(pi-radA)/(sin(radA+pi/2-u)),...
        (b-y)*sin(u)*tan(pi/2-radA)/(sin(radA-u))];
    else
        R=[0,0,0,0];
    end
else
    R=[0,0,0,0];    
end
    
% ----setting startpoint for robot---- %
x=x+startX;
y=y+startY;
% ------------------------------------ %

% setting in which maze to do simulation


if maze==1;
    ir=maze1(x,y,theta,s,b,R); 
elseif maze==2,
    ir=maze2(x,y,theta,s,b,R);
elseif maze==3,
    ir=maze3(x,y,theta,s,b,R);
elseif maze==4,
    ir=maze4(x,y,theta,s,b,R);
elseif maze==5,
    ir=maze5(x,y,theta,s,b,R);
elseif maze==6,
    ir=maze6(x,y,theta,s,b,R);
elseif maze==9,
    ir=maze0(x,y,theta,s,b,R);
elseif maze==8,
    ir=maze8(x,y,theta,s,b,R);
else
    ir=maze1(x,y,theta,s,b,R); % maze 1 is default
end

% including random errors to signal
if mySimParams.irError,
    Ir=abs(ir)-radarOffset+0.6*randn(1);
else
    Ir=abs(ir)-radarOffset;
end

% filtering high signals
if Ir>perceptRad*100,
    myIr=255;
else
    myIr=Ir;
end
