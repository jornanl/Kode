function turnRad=quasiFollowLeftWallNavigation2( Gmap )
% This function finds the left most direction the robot can move in
% whitout hitting a wall. It searches from -45 deg to +90 deg for a
% obstacle-free region. (priority to the left)

globalTemp=get(Gmap,'x');
roboPose = get(globalTemp{1,1},'x');

%get(globalTemp{1,2},'X') == [x;y]
npoints=length(globalTemp)-1;
for i=2:length(globalTemp)
    nums = size(get(globalTemp{1,i},'X'),2);
    beacons(i-1,1:2)=get(globalTemp{1,i},'X')';
end;

%define rectangular region. Robot ref-frame.
rectLength=0.25; % in meter
rectWidth=0.125; % in meter

%figure(8);clf;hold on;
%plot([-rectWidth +rectWidth],[0 0],'b',[-rectWidth +rectWidth],[rectLength rectLength],'b',[-rectWidth -rectWidth],[0 rectLength],'b',[rectWidth rectWidth],[0 rectLength],'b');

%rotate all beacons from global frame to robot frame
beaconsRobotFrame=-1;
rotmat=[cos(roboPose(3)-pi/2) sin(roboPose(3)-pi/2) ; -sin(roboPose(3)-pi/2) cos(roboPose(3)-pi/2)];
angleTurn = 0;
counter=0;
angleCount=0;

if( npoints == 0 )

else
    for j=1:1:size(beacons,1)
        
        if ~beacons(j,:)==0,
            beacons(j,1)  = beacons(j,1) - roboPose(1);
            beacons(j,2)= beacons(j,2) - roboPose(2);
            beaconsRobotFrame(j,1:2)=(rotmat*(beacons(j,1:2)'))';
                
                %debug
                %plot all beacons from the robots  point of view
                
                %plot( beaconsRobotFrame(j,1),beaconsRobotFrame(j,2),'kx','LineWidth',2,'MarkerSize',9);
                
        end
        
    end

    %wall inside region check. checking form -45 to 90 deg. eg, we
    %try to follow the left wall.

    obstruction=1;       
    j=1;
    i=1;
    color=1;

    angleTurn = pi/4;
    rotmat=[cos(angleTurn) sin(angleTurn) ; -sin(angleTurn) cos(angleTurn)];
    %rotmat=[rotmat [0 0;0 0];[0 0;0 0] rotmat];    
    %figure(7);clf;hold on;
    %plot([-rectWidth +rectWidth],[0 0],'b',[-rectWidth +rectWidth],[rectLength rectLength],'b',[-rectWidth -rectWidth],[0 rectLength],'b',[rectWidth rectWidth],[0 rectLength],'b');
    
    for q=1:1:size(beacons,1)      
                  
        beaconsRobotFrame(q,1:2)=(rotmat*(beaconsRobotFrame(q,1:2)'))';
        %plot( beaconsRobotFrame(q,1),beaconsRobotFrame(q,2),'rx','LineWidth',2,'MarkerSize',9);

    end;
    %figure(7);clf;hold on;
    %plot([-rectWidth +rectWidth],[0 0],'b',[-rectWidth +rectWidth],[rectLength rectLength],'b',[-rectWidth -rectWidth],[0 rectLength],'b',[rectWidth rectWidth],[0 rectLength],'b');
    
    while( obstruction && angleCount <= 9  )

        obstruction=0;
        color=color*0.8;
        counter = counter+1;
        
        %disp(counter);
        if(counter > 15)
            disp('navigation error');
            return;            
        end;    
        
        
        for j=1:1:size(beacons,1)
            
            if( beaconsRobotFrame(j,1)==0 && beaconsRobotFrame(j,2) == 0)
                continue;
            end;
            
            %debug
            %plot(beaconsRobotFrame(j,1), beaconsRobotFrame(j,2),'Linewidth',2,'Color',[1 (1-color) 0]);
            %plot( beaconsRobotFrame(j,1),beaconsRobotFrame(j,2),'kx','LineWidth',2,'MarkerSize',9);    
            %plot( beaconsRobotFrame(j,1),beaconsRobotFrame(j,2),'x','Color',[1 (1-color) 0]);
            %plot( beaconsRobotFrame(j,1),beaconsRobotFrame(j,2),'kx','LineWidth',2,'MarkerSize',9);
               
            if~((rectLength < beaconsRobotFrame(j,2)) || (beaconsRobotFrame(j,2) <0 ) ||...
                   ( beaconsRobotFrame(j,1) < -rectWidth) || ( beaconsRobotFrame(j,1) > rectWidth) ),
            
                angleTurn = -pi/12;
                        
                rotmat=[cos(angleTurn) sin(angleTurn) ; -sin(angleTurn) cos(angleTurn)];
                

                obstruction = 1;
                angleCount = angleCount+1;
                   
                for q=1:1:size(beacons,1)            
                    beaconsRobotFrame(q,1:2)=(rotmat*([beaconsRobotFrame(q,1:2)]'))';
                end;          
                       
                
                if(obstruction)
                    break;
                end;
            end
            if(obstruction)
                j=1;
                i=1;               
                break;
            end;                
        end;
    end;
end;


%disp('turn robot this much:')
%disp( (pi/4 + angleTurn*angleCount)*180/pi );
%pause;

turnRad = pi/4 + angleTurn*angleCount;
if(turnRad < 0)
   turnRad = turnRad + 2*pi;
elseif(turnRad > 2*pi)
    turnRad = turnRad - 2*pi;
end;



