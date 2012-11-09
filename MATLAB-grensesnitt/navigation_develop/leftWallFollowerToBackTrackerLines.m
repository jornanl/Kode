function [error,navData,handles,scndata]=leftWallFollowerToBackTrackerLines(G,myCon,navData,gaps,handles)
global RUNNING;

% Get robot and its pose
r  = getrobot(G); 
myNewpose = get(r,'x')';

% simulation parameter
scndata=0;

distToStartBackTrack = 0.15; % in meter
motorParam=0; % zero in simulation
error=0;   
 
cT=handles.checkTable;
szcT=size(cT,1);   

numCol=size(navData.allBackTrackPoints,2);
    
%% initial state
%if (navData.state==0),
%    tmpAngle=navData.lastHeading + 120;
%    robAngle = char( tmpAngle/2 );
%    [error,scndata]=setRobotTarget(myCon,robAngle,0,navData.lastHeading,handles);
    
%    navData.lastHeading=2*double(robAngle);
%    if tmpAngle==240,
%        navData.state=1;
%    end


%% normal navigation state
if ( navData.state == 1)
    %state check/change. Start to backtrack?
    numPoses=size(navData.robotPath,1);
    if( numPoses > 5 )
        distVal = (navData.robotPath(1:1:(numPoses-4),1)-myNewpose(1)).^2+(navData.robotPath(1:1:(numPoses-4),2)-myNewpose(2)).^2 ;            
        %Current pose close to starting pose??
        tmp= find(distVal < distToStartBackTrack*distToStartBackTrack);
        if(size(tmp,1)>0)
            navData.state = 2;
            navData.STARTbackTrackPoint = numPoses ;
            navData.backTrackPoint = numPoses;
            disp('Starting to backtrack');
        else 
            navData.backTrackPoint = -1 ;
            navData.STARTbackTrackPoint = -1 ;                
        end;
    end;         
        
    if( navData.state == 1) % did not change state
        % just try to follow the left wall        
        turnRads=quasiFollowLeftWallNavigation( G );
        
        %90*pi because 0->180=0->360 on robot
        if(turnRads == 0 || abs(turnRads) < 0.01 )
            tmp = floor( myNewpose(3)*90/pi );
            robAngle = char( tmp );
            [error,scndata]=setRobotTarget(myCon,robAngle,15,handles);
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
            [error,scndata]=setRobotTarget(myCon,robAngle,10,handles);
            navData.lastHeading=2*double(robAngle);
        end;   
    end;       

%% Backtracking state
elseif( navData.state == 2 )        
    %finds backtrackpoint close to an unvisted gap in the wall
    % added by Trond M
    [newPath,dummyX,dummyY] = findNextInterestingGapPathIndex2...
        ( navData.robotPath(1:1:navData.STARTbackTrackPoint,:),...
        navData.robotPath,gaps);
    if numCol==1,
        rows=size(navData.allBackTrackPoints,1);
        if (rows==1 && navData.allBackTrackPoints==0 && newPath~=-1)
            navData.allBackTrackPoints(1,numCol)=newPath;
            navData.allBackTrackPoints(2,1)=0;            
        end
    end
    if (navData.allBackTrackPoints(1,numCol)~=newPath && newPath~=-1),
        navData.allBackTrackPoints(1,numCol+1)=newPath;    
    end
    if (newPath==-1), 
        if navData.backTrackPoint+1 == navData.STARTbackTrackPoint &&...
                navData.robotPath(navData.STARTbackTrackPoint,1)==0 &&...
                navData.robotPath(navData.STARTbackTrackPoint,2)==0,
            
            navData.backTrackPointAtGap=3;
            navData.allBackTrackPoints(1,numCol)=1;
                    
                    
                    
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
                break;
            elseif( navData.backTrackPoint == navData.backTrackPointAtGap -1) %stop backtracking when reaching a gap
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
            elseif ( navData.backTrackPoint < navData.backTrackPointAtGap-2 ),
                disp('not able to reach desired backtrack point!! Backtracking to an earlier gap');
                navData.state = 21;
            end;
        end;    
    end;
        
    %currgotoPose = gotoPose;
     
    relturn= -1;
    if( navData.state == 2 )
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
            [error,scndata]=setRobotTarget(myCon,robAngle, round(min((gotoDist*100)*0.5,20)),...
                handles);
            navData.lastHeading=2*double(robAngle);
            
        end;
    end;

%% Moving through gaps state
elseif( navData.state == 5 )
    disp('%move through the gap.....');
    
    [gapX,gapY,myGapAnglePerp] = findNearestGap(navData.robotPath(1:1:navData.STARTbackTrackPoint,:),navData.backTrackPointAtGap,gaps); %added by Trond M
                
    relturn= -1;
    directionVectX =gapX-myNewpose(1);
    directionVectY = gapY-myNewpose(2);            
    gotoDist=sqrt(directionVectX^2+directionVectY^2);
         
    if( directionVectX == 0 )          
        if(directionVectY == 0)
            disp('Navigation Error 1!!');
        elseif(directionVectY > 0)
            relturn = 90;
        elseif(directionVectY < 0)
            relturn = 270;
        end;
    elseif(directionVectX > 0)
        if(directionVectY == 0)
            relturn = 0;
        elseif(directionVectY > 0)
            relturn = floor( acos( directionVectX/gotoDist )*180/pi);% corrected from cos to acos
        elseif(directionVectY < 0)
            relturn = 360-floor( acos( directionVectX/gotoDist )*180/pi);% corrected from cos to acos
        end;                
    elseif(directionVectX < 0)
        if(directionVectY == 0)
            relturn = 180;
        elseif(directionVectY > 0)
            relturn = floor( acos( directionVectX/gotoDist )*180/pi);% corrected from cos to acos
        elseif(directionVectY < 0)
            relturn = 360-floor( acos( directionVectX/gotoDist )*180/pi);% corrected from cos to acos                    
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
        
        tmp = floor( relAngle/2 + motorParam );
        robAngle = char( tmp );
        [error,scndata]=setRobotTarget(myCon,robAngle, round(gotoDist*100) +5,...
            handles);
        navData.lastHeading=2*double(robAngle);
        
        % rotating robot to achieve correct heading after move
        if (myGapAnglePerp>=0 && myGapAnglePerp < 90),
            if (relAngle>=myGapAnglePerp+270 && relAngle<360) || (relAngle>=0 && relAngle<myGapAnglePerp+90),
                myAngle=myGapAnglePerp;
            else
                myAngle=myGapAnglePerp+180;
            end
        elseif (myGapAnglePerp>=90 && myGapAnglePerp <= 180),
            if (relAngle>=myGapAnglePerp-90 && relAngle<myGapAnglePerp+90),
                myAngle=myGapAnglePerp;
            else
                myAngle=myGapAnglePerp+180;
            end     
        end
        tmp = floor( myAngle/2 );
        myRobAngle = char( tmp );
        [error]=setRobotTarget(myCon,myRobAngle, 0, handles);
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

%% 
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
            [error,scndata]=setRobotTarget(myCon,robAngle, round(min((gotoDist*100)*0.5,14))...
                ,handles);
            navData.lastHeading=2*double(robAngle);                
        end;
    end;
    % end added
        
end;

