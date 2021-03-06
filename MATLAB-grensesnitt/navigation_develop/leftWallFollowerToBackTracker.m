function [error,navData,handles,scndata]=leftWallFollowerToBackTracker(G,myCon,navData,gaps,handles)
global RUNNING;

% Get robot and its pose
r  = getrobot(G); 
myNewpose = get(r,'x')';

% simulation parameter
scndata=[];

distToStartBackTrack = 0.15; % in meter
error=0;   

% Define rectangurlar region. Robot ref-frame.
rectLength=0.30; % in meter
rectWidth=0.15; % in meter

navData.b=rectWidth;
navData.l=rectLength;



cT=handles.checkTable;
szcT=size(cT,1);

numCol=size(navData.allBackTrackPoints,2);

    
%% initial state
if (navData.state==0),
%    tmpAngle=navData.lastHeading + 120;
%    robAngle = char( tmpAngle/2 );
%    [error,scndata]=setRobotTarget(myCon,robAngle,0,handles);
%    
%    navData.lastHeading=2*double(robAngle);
%    if tmpAngle==240,
%        navData.state=1;
%    end
    setRobotTarget(myCon,char(180 /2),15,handles);
    navData.state=1;

%% normal navigation state
elseif ( navData.state == 1)
    if navData.totalDist>1 && handles.goBack
        % && navData.state~=91        handles.goBack nonexistent...
        % enter go back to startpoint
        navData.state=90;
        
    end
    
    %state check/change. Start to backtrack?
    numPoses=size(navData.robotPath,1);
    if( numPoses > 5 )
        distVal = (navData.robotPath(1:1:(numPoses-4),1)-myNewpose(1)).^2+(navData.robotPath(1:1:(numPoses-4),2)-myNewpose(2)).^2 ;            
        %Current pose close to starting pose??
        tmp= find(distVal < distToStartBackTrack*distToStartBackTrack); % Gives out a matrix with the index of which position in robotPath the current position is closer to than 15 cm
        if(size(tmp,1)>0)
            navData.state = 2;
            navData.STARTbackTrackPoint = numPoses ;
            navData.backTrackPoint = numPoses;
            disp('Starting to backtrack');
            navData.teller=0;
        else 
            navData.backTrackPoint = -1 ;
            navData.STARTbackTrackPoint = -1 ;                
        end;
    end;         
        

    if( navData.state == 1) % did not change state
        % just try to follow the left wall        
        %turnRads1=quasiFollowLeftWallNavigation( G );
        %turnRads2=quasiFollowLeftWallNavigation2( P );
        %turnRads2=-90;
        turnRads=quasiFollowLeftWall(G,rectWidth,rectLength); % in meter
        
        %if turnRads==turnRads2,
        %    disp('her brukes turnRads2..')
        %end
        
        %90*pi because 0->180=0->360 on robot
        if(turnRads == 0 || abs(turnRads) < 0.01 )
            tmp = floor( myNewpose(3)*90/pi );
            robAngle = char( tmp );
            [error,scndata]=setRobotTarget(myCon,robAngle,17.5,handles);
            navData.lastHeading=2*double(robAngle);
        else
            relVinkel = myNewpose(3)+turnRads;
            
            relVinkel = relVinkel + 2*pi*(relVinkel<0);
            relVinkel = relVinkel - 2*pi*(relVinkel>2*pi);
            
%            if(relVinkel < 0)
%                relVinkel = relVinkel+2*pi;
%            elseif(relVinkel>2*pi)
%                relVinkel = relVinkel-2*pi;               
%            end;
            tmp = floor( relVinkel/2*180/pi );
            robAngle = char( tmp ) ;
            [error,scndata]=setRobotTarget(myCon,robAngle,15,handles);
            navData.lastHeading=2*double(robAngle);
        end;
        if error,
            disp('navigation_develop: error setRobotTarget')
        end
    end;       

%% Backtracking state
elseif( navData.state == 2 )        
  
        backTrackPnts=numCol;
        %finds backtrackpoint close to an unvisted gap in the wall
        % added by Trond M
        if navData.teller==0,
        [newPath,dummyX,dummyY] = findNextInterestingGapPathIndex2...
            ( navData.robotPath(1:1:navData.STARTbackTrackPoint,:),...
            navData.robotPath,gaps);
        navData.teller=1;
        if numCol==1, % First time allBackTrackPoints is set
            rows=size(navData.allBackTrackPoints,1);
            if (rows==1 && navData.allBackTrackPoints==0 && newPath~=-1)
                navData.allBackTrackPoints(1,numCol)=newPath;
                navData.allBackTrackPoints(2,1)=0;            
            end
        end
        if (navData.allBackTrackPoints(1,numCol)~=newPath && newPath~=-1), % Set the next index of allBackTrackPoints each time it is called
            navData.allBackTrackPoints(1,numCol+1)=newPath;    
        end
        if (newPath==-1), 
            if navData.backTrackPoint+1 == navData.STARTbackTrackPoint &&... % If you have backtracked one step and the start point for the backtracking is the same place as you started the navigation
                    navData.robotPath(navData.STARTbackTrackPoint,1)==0 &&...
                    navData.robotPath(navData.STARTbackTrackPoint,2)==0,
            
                navData.backTrackPointAtGap=3;
                navData.allBackTrackPoints(1,numCol)=1; % Then this is the first back track point?          
                    
            elseif navData.backTrackPoint+1 == navData.STARTbackTrackPoint,
                [newPath,gapX,gapY]=findNextInterestingGapPathIndex4...
                    ( navData.robotPath(1:1:navData.STARTbackTrackPoint-1,:),...
                    navData.robotPath(navData.allBackTrackPoints(2,numCol):1:navData.STARTbackTrackPoint-1,:),gaps);
                newPath=newPath+navData.allBackTrackPoints(2,numCol)-1;
                navData.backTrackPointAtGap=newPath;
                if (navData.allBackTrackPoints(1,numCol)~=newPath),
                    navData.allBackTrackPoints(1,numCol+1)=newPath-1;
                    navData.allBackTrackPoints(2,numCol+1)=navData.allBackTrackPoints(2,numCol);
                end
                                
            elseif numCol==1,
                navData.backTrackPointAtGap= navData.allBackTrackPoints(1,1);
            else
                navData.backTrackPointAtGap = navData.allBackTrackPoints(1,numCol);
            end
        else
            navData.backTrackPointAtGap = newPath;
        end    
        % end added
        end
        gotoPose =[];
        
        if( navData.backTrackPointAtGap == 0)
            %no gaps
            disp('No unvisited gaps to backtrack to');
            navData.state = 9;            
        else       
            %backtrack to a gap in the wall or something
            gotoPose=navData.robotPath(navData.backTrackPoint,:);
            while( (gotoPose(1)-myNewpose(1))^2 + (gotoPose(2)-myNewpose(2))^2 < 0.1*0.1 )
                navData.backTrackPoint = navData.backTrackPoint -1;
                gotoPose = navData.robotPath(navData.backTrackPoint,:);
                        
                if( navData.backTrackPoint == 1 ) %stop backtracking when you're back at the startingpoint..
                    disp('Stopped backtracking...robot is back at starting position');
                    disp('Enter last unvisited gap')
                    navData.state = 5;
                    navData.teller=0;
                    break;
                elseif( navData.backTrackPoint == navData.backTrackPointAtGap -1) %stop backtracking when reaching a gap
                    disp('Backtracking to gap done');
                    % added by Trond M                    
                    navData.allBackTrackPoints(2,numCol)=size(navData.robotPath,1);
                    % end added
                    navData.state = 5;
                    navData.teller=0;
                    break;
                elseif numCol>1 && ( navData.backTrackPoint == navData.allBackTrackPoints(2,numCol-1) ),
                    disp('backtracked to an already visited junction');
                    navData.state = 3;
                    navData.teller=0;
                    break;
                elseif ( navData.backTrackPoint < navData.backTrackPointAtGap-2 ),
                    disp('not able to reach desired backtrack point!! Backtracking to an earlier gap');
                    navData.state = 21;
                    navData.teller=0;
                end;
            end;    
        end;
        
        %currgotoPose = gotoPose;
     
        relturn= -1; % Angle relative the x-axis
        if( navData.state == 2 )
            directionVectX = gotoPose(1)-myNewpose(1);
            directionVectY = gotoPose(2)-myNewpose(2);         
            gotoDist=sqrt(directionVectX^2+directionVectY^2); % Given in [m]
        
            if( directionVectX == 0 )          
                if(directionVectY == 0)
                %disp('Navigation error..2 ');
                elseif(directionVectY > 0)
                %disp('Navigation case..1 ');                    
                    relturn = 90;
                elseif(directionVectY < 0)
                    %disp('Navigation case..2 ');                                        
                    relturn = 270;
                end;
            elseif(directionVectX > 0)
                if(directionVectY == 0)
                    relturn = 0;
                    %disp('Navigation case..3 ');                                        
                elseif(directionVectY > 0)
                    relturn = floor( acos( directionVectX/gotoDist )*180/pi);
                    %disp('Navigation case..4 ');                                        
                elseif(directionVectY < 0)
                    relturn = 360-floor( acos( directionVectX/gotoDist )*180/pi);
                    %disp('Navigation case..5 ');                                        
                end;                
            elseif(directionVectX < 0)
                if(directionVectY == 0)
                    %disp('Navigation case..6 ');                                        
                    relturn = 180;
                elseif(directionVectY > 0)
                    relturn = floor( acos( directionVectX/gotoDist )*180/pi);
                    %disp('Navigation case..7 ');                                        
                elseif(directionVectY < 0)
                    relturn = 360-floor( acos( directionVectX/gotoDist )*180/pi);                    
                    %disp('Navigation case..8 ');                                        
                end;                
            end;
            
        
            
            if(relturn == -1)
                disp('Navigation Error 2!!');
            else
                %relAngle = myNewpose(3)*180/pi + relturn
                relAngle =relturn;
                if(relAngle < 0)
                    relAngle = relAngle+360;
                elseif(relAngle>=360)
                    relAngle = relAngle-360;
                end;
             
                tmp = floor( relAngle/2 );
                robAngle = char( tmp );
                [error,scndata]=setRobotTarget(myCon,robAngle, round(min((gotoDist*100)*0.5,20)),handles); % Travles half the distanse between current position and the position it is backtracking to
                navData.lastHeading=2*double(robAngle);
            end
        end

%% Moving through gaps state
elseif( navData.state == 5 )
    disp('%move through the gap.....');
    
    [gapX,gapY,myGapAnglePerp] = findNearestGap(navData.robotPath(1:1:navData.STARTbackTrackPoint,:),navData.backTrackPointAtGap,gaps); % gapX and gapY are the coordinates of the midpoint to the nearest gap. the angle is prependicular to the gap
                
    relturn= -1;
    directionVectX =gapX-myNewpose(1);
    directionVectY = gapY-myNewpose(2);            
    gotoDist=sqrt(directionVectX^2+directionVectY^2); % Distance between current pose and to the midpoint of the nearest gap
         
    if( directionVectX == 0 )          
        if(directionVectY == 0)
            disp('Navigation Error 1!!');
        elseif(directionVectY > 0)
            relturn = 90;   % Angle relative to the x-axis
        elseif(directionVectY < 0)
            relturn = 270;  % Angle relative to the x-axis
        end;
    elseif(directionVectX > 0)
        if(directionVectY == 0)
            relturn = 0;    % Angle relative to the x-axis
        elseif(directionVectY > 0)
            relturn = floor( acos( directionVectX/gotoDist )*180/pi); % Angle relative to the x-axis
        elseif(directionVectY < 0)
            relturn = 360-floor( acos( directionVectX/gotoDist )*180/pi);   % Angle relative to the x-axis
        end;                
    elseif(directionVectX < 0)
        if(directionVectY == 0)
            relturn = 180;  % Angle relative to the x-axis
        elseif(directionVectY > 0)
            relturn = floor( acos( directionVectX/gotoDist )*180/pi);   % Angle relative to the x-axis
        elseif(directionVectY < 0)
            relturn = 360-floor( acos( directionVectX/gotoDist )*180/pi);  % Angle relative to the x-axis                  
        end;                
    end;
            
    if(relturn == -1)
        disp('Navigation Error 2!!');
    else
        relAngle =relturn;
        if(relAngle < 0)
         	relAngle = relAngle+360;
        elseif(relAngle>360)
            relAngle = relAngle-360;
        end;             
        
        tmp = floor( relAngle/2 );
        robAngle = char( tmp );
        [error,scndata]=setRobotTarget(myCon,robAngle, round(gotoDist*100) +5,handles); % Moves the robot 5 cm through the gap, and the angle is set to be towards the midpoint of the gap from the current position
        navData.lastHeading=2*double(robAngle);
        
        % rotating robot to achieve correct heading after move,
        % prependicular to the gap
        if (myGapAnglePerp>=0 && myGapAnglePerp < 90), % If the gap is laying in the second or fourth quadrant relative the fixed frame
            if (relAngle>=myGapAnglePerp+270 && relAngle<360) || (relAngle>=0 && relAngle<myGapAnglePerp+90), % Depending on what side of the gap the current position was
                myAngle=myGapAnglePerp;
            else
                myAngle=myGapAnglePerp+180;
            end
        elseif (myGapAnglePerp>=90 && myGapAnglePerp <= 180), % If the gap is laying in the first or third quadrant relative the fixed frame
            if (relAngle>=myGapAnglePerp-90 && relAngle<myGapAnglePerp+90), % Depending on what side of the gap the current position was
                myAngle=myGapAnglePerp;
            else
                myAngle=myGapAnglePerp+180;
            end     
        end
        tmp = floor( myAngle/2 );
        myRobAngle = char( tmp );
        [error]=setRobotTarget(myCon,myRobAngle, 0, handles); % Rotate the robot such that the heading is prependicular to the gap
        navData.lastHeading=2*double(myRobAngle);
    end;
        
    navData.state = 1;
%% Finish state
elseif( navData.state == 9 )
    disp('Nowhere to backtrack. Finished.');
    RUNNING = 0;
    %pause;

%% Backtracking: earlier steps found state 
elseif( navData.state == 3 ),
    %stop when earlier path found during backtracking
    %added by Trond M
    navData.backTrackPoint=navData.allBackTrackPoints(1,numCol-1);
    if numCol>2,
        v=navData.allBackTrackPoints(1:2,1:numCol-2);
        v(1,size(v,2)+1)=navData.allBackTrackPoints(1,numCol);
        v(2,size(v,2))=0;
        navData.allBackTrackPoints=v;
    elseif numCol==2,
        v(1,1)=navData.allBackTrackPoints(1,2);
        v(2,1)=0;
        navData.allBackTrackPoints=v;
    end
    % continue to backtrack..
    navData.state=2;

%% Backtracking: not able to reach desired backtrackpoint..
elseif (navData.state == 21),
    [newPath,dummyX,dummyY] = findNextInterestingGapPathIndex3...
        ( navData.robotPath(1:1:navData.STARTbackTrackPoint,:),...
        navData.robotPath,gaps);
        
    navData.backTrackPointAtGap=newPath;
    if numCol==1,
        rows=size(navData.allBackTrackPoints,1);
        if (rows==1 && navData.allBackTrackPoints==0 && newPath~=-1)
            navData.allBackTrackPoints(1,numCol)=newPath;
            navData.allBackTrackPoints(2,1)=0;
        end
    end
    if (navData.allBackTrackPoints(1,numCol)~=newPath && newPath~=-1),
        navData.allBackTrackPoints(1,numCol)=newPath;    
    end
        
    gotoPose =[];
       
    if( navData.backTrackPointAtGap == 0)
        %no gaps
        disp('No unvisited gaps to backtrack to');
        navData.state = 9;            
    else       
        %backtrack to a gap in the wall or something
        gotoPose=navData.robotPath(navData.backTrackPoint,:);
        while( (gotoPose(1)-myNewpose(1))^2 + (gotoPose(2)-myNewpose(2))^2 < 0.1*0.1 )
            navData.backTrackPoint = navData.backTrackPoint -1;
            gotoPose = navData.robotPath(navData.backTrackPoint,:);
                        
            if( navData.backTrackPoint == 1 ) %stop backtracking when you're back at the startingpoint..
                disp('Stopped backtracking...');
                disp('Enter last unvisited gap')
                navData.state = 5;
                break;
            elseif( navData.backTrackPoint == navData.backTrackPointAtGap-1 ) %stop backtracking when reaching a gap
                disp('Backtracking to gap done');
                % added by Trond M                    
                navData.allBackTrackPoints(2,numCol)=size(navData.robotPath,1);
                % end added
                navData.state = 5;
                break;
            elseif numCol>1 && ( navData.backTrackPoint == navData.allBackTrackPoints(2,numCol-1) ),
                disp('backtracked to an already visited junction');
                navData.state = 3;
                break;
            end;
        end;    
    end;
        
    %currgotoPose = gotoPose;
        
    relturn= -1;
    if( navData.state == 21 )
        directionVectX = gotoPose(1)-myNewpose(1);
        directionVectY = gotoPose(2)-myNewpose(2);         
        gotoDist=sqrt(directionVectX^2+directionVectY^2);
        
        if( directionVectX == 0 )          
            if(directionVectY == 0)
                %disp('Navigation error..2 ');
            elseif(directionVectY > 0)
                %disp('Navigation case..1 ');                    
                relturn = 90;
            elseif(directionVectY < 0)
                %disp('Navigation case..2 ');                                        
                relturn = 270;
            end;
        elseif(directionVectX > 0)
            if(directionVectY == 0)
                relturn = 0;
                %disp('Navigation case..3 ');                                        
            elseif(directionVectY > 0)
                relturn = floor( acos( directionVectX/gotoDist )*180/pi);% corrected from cos to acos
                %disp('Navigation case..4 ');                                        
            elseif(directionVectY < 0)
                relturn = 360-floor( acos( directionVectX/gotoDist )*180/pi);% corrected from cos to acos
                %disp('Navigation case..5 ');                                        
            end;                
        elseif(directionVectX < 0)
            if(directionVectY == 0)
                %disp('Navigation case..6 ');                                        
                relturn = 180;
            elseif(directionVectY > 0)
                relturn = floor( acos( directionVectX/gotoDist )*180/pi);% corrected from cos to acos
                %disp('Navigation case..7 ');                                        
            elseif(directionVectY < 0)
                relturn = 360-floor( acos( directionVectX/gotoDist )*180/pi);% corrected from cos to acos                    
                %disp('Navigation case..8 ');                                        
            end;                
        end;
       
        %currrelturn = relturn
            
        if(relturn == -1)
            disp('Navigation Error 2!!');
        else
            %relAng = myNewpose(3)*180/pi + relturn
            relAngle =relturn;
            if(relAngle < 0)
            	relAngle = relAngle+360;
            elseif(relAngle>=360)
                relAngle = relAngle-360;
            end;
             
            tmp = floor( relAngle/2 );
            robAngle = char( tmp );
            [error,scndata]=setRobotTarget(myCon,robAngle, round(min((gotoDist*100)*0.5,14)),handles);
            navData.lastHeading=2*double(robAngle);                
        end;
    end;
    
end
