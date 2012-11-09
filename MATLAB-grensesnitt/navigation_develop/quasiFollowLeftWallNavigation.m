function turnRad=quasiFollowLeftWallNavigation( Gmap )
% This function finds the left most direction the robot can move in
% whitout hitting a wall. It searches from -45 deg to +90 deg for a
% obstacle-free region. (priority to the left)

X=get(Gmap,'x');
roboPose = get(X{1,1},'x');
startnstopp = -1;

% split up segments and points
    Gs=Gmap;
    Gp=Gmap;
    X = get(Gmap,'x');
    C = get(Gmap,'c');
    
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

segs=[];
for i=2:length(Xs),
    ss=get(Xs{1,i},'ss');
    tmp=[];
    for j=1:4:size(ss,2),
        tmp=vertcat(tmp,ss(j:j+3));
    end
    
    for t=1:size(tmp,1),
    if tmp(t,1)> tmp(t,3), % sort with respect to x
        tmp(t,1:4)=[tmp(t,3) tmp(t,4) tmp(t,1) tmp(t,2)];
    end
    end
    segs(size(segs,1)+1:size(tmp,1)-1+size(segs,1)+1,:)=tmp;
end

% fetch points
%Xp = get(Gp,'x');
%pt=zeros(2,length(Xp)-1);
%for i=2:length(Xp)
%    pt(:,i-1)=get(Xp{1,i},'x');
%end    
%beacons=pt;

startnstopp=segs;

%define rectangurlar region. Robot ref-frame.
rectLength=0.25; % in meter
rectWidth=0.125; % in meter

%figure(8);clf;hold on;
%plot([-rectWidth +rectWidth],[0 0],'b',[-rectWidth +rectWidth],[rectLength rectLength],'b',[-rectWidth -rectWidth],[0 rectLength],'b',[rectWidth rectWidth],[0 rectLength],'b');

%rotate all segments from global frame to robot frame
startnstoppRobotFrame=-1;
rotmat=[cos(roboPose(3)-pi/2) sin(roboPose(3)-pi/2) ; -sin(roboPose(3)-pi/2) cos(roboPose(3)-pi/2)];
rotmat=[rotmat [0 0;0 0];[0 0;0 0] rotmat];
angleTurn = 0;
counter=0;
angleCount=0;

%rotate all beacons from global frame to robot frame
%beaconsRobotFrame=-1;
%rotmatBeacons=[cos(roboPose(3)-pi/2) sin(roboPose(3)-pi/2) ; -sin(roboPose(3)-pi/2) cos(roboPose(3)-pi/2)];

if( startnstopp == -1 )

else
    %figure(77); clf; hold on;
    for j=1:1:size(startnstopp,1)
        for i=1:4:size(startnstopp,2)
            if ~startnstopp(j,i)==0,
                startnstopp(j,i:2:i+3)  = startnstopp(j,i:2:i+3) - roboPose(1);
                startnstopp(j,i+1:2:i+3)= startnstopp(j,i+1:2:i+3) - roboPose(2);
                startnstoppRobotFrame(j,i:1:i+3)=(rotmat*(startnstopp(j,i:1:i+3)'))';
                
                %debug
                %plot all line-segments from the robots  point of view
%                plot([startnstoppRobotFrame(j,i) startnstoppRobotFrame(j,i+2)],[startnstoppRobotFrame(j,i+1) startnstoppRobotFrame(j,i+3)],'LineWidth',2.7,'Color',[1 0 0]);
%                plot( startnstoppRobotFrame(j,i),startnstoppRobotFrame(j,i+1),'kx','LineWidth',2,'MarkerSize',9);
%                plot( startnstoppRobotFrame(j,i+2),startnstoppRobotFrame(j,i+3),'kx','LineWidth',2,'MarkerSize',9);
            end
        end
    end
%    for j=1:1:size(beacons,1)
%       if ~beacons(j,:)==0,
%           beacons(j,1)  = beacons(j,1) - roboPose(1);
%           beacons(j,2)= beacons(j,2) - roboPose(2);
%           beaconsRobotFrame(j,1:2)=(rotmatBeacons*(beacons(j,1:2)'))';
%                
%           %debug
%           %plot all beacons from the robots  point of view
%            
%           %plot( beaconsRobotFrame(j,1),beaconsRobotFrame(j,2),'kx','LineWidth',2,'MarkerSize',9);
%       end 
%    end
    
    
    %wall inside region check. checking form -45 to 90 deg. eg, we
    %try to follow the left wall.

    obstruction=1;       
    color=1;

    angleTurn = pi/4;
    rotmat=[cos(angleTurn) sin(angleTurn) ; -sin(angleTurn) cos(angleTurn)];
%   rotmatBeacons=rotmat;
    rotmat=[rotmat [0 0;0 0];[0 0;0 0] rotmat];    
    for q=1:1:size(startnstopp,1)      
        for r=1:4:size(startnstopp,2),
            if ~startnstopp(q,r)==0,
                startnstoppRobotFrame(q,r:1:r+3)=(rotmat*(startnstoppRobotFrame(q,r:1:r+3)'))';
            end
        end;
    end;
    
%    for q=1:1:size(beacons,1)              
%        beaconsRobotFrame(q,1:2)=(rotmatBeacons*(beaconsRobotFrame(q,1:2)'))';
%        %plot( beaconsRobotFrame(q,1),beaconsRobotFrame(q,2),'rx','LineWidth',2,'MarkerSize',9);
%    end;
  
    while( obstruction && angleCount <= 9  )

        obstruction=0;
        color=color*0.90;
        counter = counter+1;
        
        %disp(counter);
        if(counter > 15)
            disp('navigation error');
            return;            
        end;    
        
        for j=1:1:size(startnstopp,1)
            
            for i=1:4:size(startnstopp,2)
                if ~startnstopp(j,i)==0,
                if( startnstoppRobotFrame(j,i)==0 && startnstoppRobotFrame(j,i+1) == 0 && startnstoppRobotFrame(j,i+2)==0 && startnstoppRobotFrame(j,i+3) ==0)
                    continue;
                end;

                %debug
%                plot([startnstoppRobotFrame(j,i) startnstoppRobotFrame(j,i+2)],[startnstoppRobotFrame(j,i+1) startnstoppRobotFrame(j,i+3)],'LineWidth',2.7,'Color',[color (1-color) 0]);
%                plot( startnstoppRobotFrame(j,i),startnstoppRobotFrame(j,i+1),'kx','LineWidth',2,'MarkerSize',9);
%                plot( startnstoppRobotFrame(j,i+2),startnstoppRobotFrame(j,i+3),'kx','LineWidth',2,'MarkerSize',9);

                if( startnstoppRobotFrame(j,i+1) > rectLength && startnstoppRobotFrame(j,i+3) > rectLength)
                    %disp('here:1');
                    %no prob
                elseif( startnstoppRobotFrame(j,i+1) < 0 && startnstoppRobotFrame(j,i+3) < 0)
                    %disp('here:2');                    
                    %no prob
                elseif( startnstoppRobotFrame(j,i) < -rectWidth && startnstoppRobotFrame(j,i+2) < -rectWidth )
                    %disp('here:3');                    
                    %no prob
                elseif( startnstoppRobotFrame(j,i) > rectWidth && startnstoppRobotFrame(j,i+2) > rectWidth )
                    %disp('here:4');                    
                    %no prob
                else
                    %disp('here:5');                    
                    
                    
                    deltaX = startnstoppRobotFrame(j,i)  -startnstoppRobotFrame(j,i+2);
                    deltaY = startnstoppRobotFrame(j,i+1)-startnstoppRobotFrame(j,i+3);
                                        
                    if( abs(deltaX) < 0.0001 )
                        %disp('here:6');                        
                        funkYOne = startnstoppRobotFrame(j,i+1);
                        funkYTwo = startnstoppRobotFrame(j,i+3);
                    else
                        %disp('here:7');
                        segAlpha= deltaY/deltaX;                
                        funkYOne = segAlpha*(-rectWidth-startnstoppRobotFrame(j,i))+startnstoppRobotFrame(j,i+1);
                        funkYTwo = segAlpha*( rectWidth-startnstoppRobotFrame(j,i))+startnstoppRobotFrame(j,i+1);
                    end;
            
                    if( funkYOne < 0 && funkYTwo < 0 )
                         %no prob
                         %disp('no prob1');
                    elseif( funkYOne > rectLength && funkYTwo > rectLength )
                         %no prob
                         %disp('no prob2');                     
                    else
                        %prob
                        angleTurn = -pi/12; %-15 degrees
                        
                        %disp('obstruction at degs:');
                        %disp((pi/4+angleCount*angleTurn)*180/pi);
                        %disp(counter);                                                           
                        
                        rotmat=[cos(angleTurn) sin(angleTurn) ; -sin(angleTurn) cos(angleTurn)];
                        rotmat=[rotmat [0 0;0 0];[0 0;0 0] rotmat];

                        obstruction = 1;
                        angleCount = angleCount+1;
                   
                        for q=1:1:size(startnstopp,1)      
                            for r=1:4:size(startnstopp,2)
                                if ~startnstopp(q,r)==0,
                                startnstoppRobotFrame(q,r:1:r+3)=(rotmat*(startnstoppRobotFrame(q,r:1:r+3)'))';
                                end
                            end;
                        end;          
                        
                        %pause(1);
                    end;
                    if(obstruction)
                        break;
                    end;
                end;
                if(obstruction)
                    j=1;
                    i=1;               
                    break;
                end;                
                end;           
            end
        end
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



