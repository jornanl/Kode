%SIMROBOT simulation of LEGO-robot 
% 
%  This modul handles communication with the SLAM algorithms 
%  and processing of data and variables within the robot.
function data=simRobot9(cmd,handles)

global ROBOTPOSE;
global REALPOSE;
NSENSORS=4;

handles.simParams.x=ROBOTPOSE(1); % [cm]
handles.simParams.y=ROBOTPOSE(2); % [cm]
handles.simParams.theta=ROBOTPOSE(3); % [degree]

%if handles.poseError
%    handles.simParams.x=REALPOSE(1); % [cm]
%    handles.simParams.y=REALPOSE(2); % [cm]
%    handles.simParams.theta=REALPOSE(3); % [degree]
%end
x=handles.simParams.x; % [cm]
y=handles.simParams.y; % [cm]
theta=handles.simParams.theta; % [degree]

rad=handles.maxPerceptRadius;
res=handles.angularRes;

handles.simParams.poseError=handles.poseError;
handles.simParams.irError=handles.irError;

    

data=0;

% ---------- start-communication-modul ----------- %

% receiving data from SLAM %
switch cmd(1),
    case 'o',
        data='k'; % responding to ping...
    case 'f',    % performing fullScan
        tmp=theta;
        c=0;
        % preallocating data
        ndata=(length(360/NSENSORS:-res:1))*4;
        data=zeros(ndata,3);
        for angle=360/NSENSORS:-res:1,
            handles.simParams.radar=angle;
            for s=1:NSENSORS,
                dir=360/NSENSORS*(s-1) ;
                handles.simParams.theta=dir;
                ir=round(simMap(handles.simParams)); % fetching IR measurement
                if ir>rad*100,
                    [x,y] = pol2cart( (90+dir-tmp-angle)*pi/180,(-4/100)+0.06);
                else
                    [x,y] = pol2cart( (90+dir-tmp-angle)*pi/180,(ir/100)+0.06);
                end
                data( NSENSORS*c+s)=s;
                
                data( NSENSORS*c+s ,2:3)=[x,y];
            end
            c=c+1;
        end
        %figure(31);clf;hold on;axis equal;
        %plot(data(:,2), data(:,3),'mx');

        % rotate the local x,y points found to global frame
        rotmat=[cos(tmp*pi/180) -sin(tmp*pi/180);sin(tmp*pi/180) cos(tmp*pi/180) ];
        rot=(rotmat*[data(:,2) data(:,3)]')';
        rot= rot + [repmat(ROBOTPOSE(1)/100,size(rot,1),1) repmat(ROBOTPOSE(2)/100,size(rot,1),1)];
        
        data(:,2)=rot(:,1);
        data(:,3)=rot(:,2); % from cm -> meters
        %plot(data(:,2), data(:,3),'yx');
        
        handles.simParams.theta=tmp;
        
    case 'h',    % setting robot heading in range [0, 359]
        if (double(cmd(2))>=0 && double(cmd(2))<=180)
            
            %data=double(cmd(2))*2;
            
            if handles.poseError,
                dh=double(cmd(2))*2-ROBOTPOSE(3);
                %pr=ROBOTPOSE(3)+180;
                %pl=ROBOTPOSE(3)-180;
                if dh>180,
                    dh=dh-360;
                elseif dh<=-180,
                    dh=dh+360;
                end
                REALPOSE(3)=REALPOSE(3) + dh + 0.0875*randn(1); % including errors to pose
                if REALPOSE(3)<0,
                    REALPOSE(3)=REALPOSE(3)+360;
                elseif REALPOSE(3)>359,
                    REALPOSE(3)=REALPOSE(3)-360;
                end
                
            end
            ROBOTPOSE(3)=double(cmd(2))*2;
        else                    
            disp('simulation/simRobot9:  heading is invalid')
        end
    case 'd',
        if (double(cmd(2))>=0 && double(cmd(2))<=256) % desired distance from SLAM in cm!
            [X,Y,data]=set_distance(double(cmd(2)),theta,x,y,handles);  % setting distance i cm
            ROBOTPOSE(1)=X; % position coordinates is stored as cm!
            ROBOTPOSE(2)=Y; % position coordinates is stored as cm!
            %data(3)=theta;
        else
            disp('simulation/simRobot9: distance is invalid')
        end
    case 'g',
        switch cmd(2),
            case 'i',
                data=round(simMap(handles.simParams)); % fetching IR measurement in cm!   
            case 'p',
                % Send coordinates as mm and theta as radians!
                data(1)=10*x; data(2)=10*y; data(3)=theta*pi/180; 
            case 'l'
                %for testing
            case 'r'
                %for testing
        end
    case 'm',
        switch cmd(2), % motor controlling (not implemented)
            case 'f',
                %robot_forward()
            case 's',
                %robot_stop()
            case 'a',
                %robot_left()
            case 'b',
                %robot_right()
        end
    case 'c',
        reset_pose(); % reset robot position
    otherwise
        disp('simRobot: command not valid')
end

%--------end-communication-modul--------------------%


function reset_pose()
global ROBOTPOSE;


%handles.simParams.y=0;
%handles.simParams.theta=0;
ROBOTPOSE=[0 0 0];
 
% --not implemented... -- %
function [robotX,robotY,robotTheta]=calcPos(oldTicksLeft,newTicksLeft,oldTicksRight,newTicksRight)
global dthetaR;
global dxR;
global sl;
global sr;
global wheelBase;
global X_fACTOR;

sl=0;
sr=0;
dxR=0;
dthetaR=0;
WHEELBASE_MM=189.4938;
X_FACTOR=0.695;

sl  = (newTicksLeft  - oldTicksLeft )*X_FACTOR;
sr  = (newTicksRight - oldTicksRight)*X_FACTOR;
dxR = (sr+sl)/2 ;
dthetaR = (sr - sl) / WHEELBASE_MM ;

robotX = robotX + ( dxR * cos(robotTheta + dthetaR/2) );
robotY = robotY + ( dxR * sin(robotTheta + dthetaR/2) );
robotTheta = robotTheta + dthetaR;
    
if( robotTheta >= (2*pi) )
  	robotTheta = robotTheta - (2*pi);
elseif( robotTheta < 0 )
   	robotTheta = robotTheta + (2*pi);
end

% ----setting distance -------%
function [newX,newY,scanData]=set_distance(d,theta,x,y,handles)
global ROBOTPOSE;
global REALPOSE;


xStart=x;yStart=y;

Xset=x+d*cos(theta*pi/180);
Yset=y+d*sin(theta*pi/180);

% robot is driving!
reached=0;
v=2; % [cm/s]
sampletime=1; %[s]
delta=v*sampletime;
t=0;
scanData=[];
if d~=0,
%robotposisjoner=[];

while ~reached,
    if handles.poseError, % including error to position estimate
        REALPOSE(1)=(REALPOSE(1)+delta*cos(theta*pi/180)) + 0.2*randn(1); 
        REALPOSE(2)=(REALPOSE(2)+delta*sin(theta*pi/180)) + 0.2*randn(1);
        
        X=x+delta*cos(theta*pi/180);
        Y=y+delta*sin(theta*pi/180);
    else
        X=x+delta*cos(theta*pi/180);
        Y=y+delta*sin(theta*pi/180);
    end


    % collect data
    
    ROBOTPOSE(1)=X;
    ROBOTPOSE(2)=Y;
    rad=handles.maxPerceptRadius;
    
    rotmat=[cos(theta*pi/180) -sin(theta*pi/180);sin(theta*pi/180) cos(theta*pi/180) ];
    % scan
    for sc=1:2
        handles.simParams.radar=180*(sc-1);
        ir=simRobot9('gi',handles)+6;
        
        if ir>rad*100,
            [x,y] = pol2cart( (90-180*(sc-1))*pi/180,(0));
            
        else
            [x,y] = pol2cart( (90-180*(sc-1))*pi/180,(ir));
            
        end
        % rotate the local x,y points found to global frame
        rot=(rotmat*[x,y]')'-[xStart-X,yStart-Y];
        x=rot(1)/100;y=rot(2)/100; % from cm -> meters
        scanData( size(scanData,1)+1,1:3)=[2*sc x y];
    end
    %robotposisjoner(size(robotposisjoner,1)+1,1:2)=[X Y];
    
    if ((X-Xset)^2+(Y-Yset)^2)<=1.5,
        reached=1;
    end
    x=X;
    y=Y;
    t=t+1;
end
newX=X;
newY=Y;
%figure(33);clf;hold on;
%plot()
else
    newX=x;
    newY=y;
end





