function turnRad=quasiFollowLeftWall(G,rectWidth,rectLength)

% split up segments and points
    Gs=G;
    Gp=G;
    X = get(G,'x');
    C = get(G,'c');
    roboPose = get(X{1,1},'x');
    
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
Xp = get(Gp,'x');
beacons=zeros(length(Xp)-1,2);
for i=2:length(Xp)
    beacons(i-1,:)=get(Xp{1,i},'x');
end    

startnstopp=segs;

%figure(8);clf;hold on;
%plot([-rectWidth +rectWidth],[0 0],'b',[-rectWidth +rectWidth],[rectLength rectLength],'b',[-rectWidth -rectWidth],[0 rectLength],'b',[rectWidth rectWidth],[0 rectLength],'b');

%rotate all segments from global frame to robot frame
rotmat1=[cos(roboPose(3)-pi/2) sin(roboPose(3)-pi/2) ; -sin(roboPose(3)-pi/2) cos(roboPose(3)-pi/2)];
rotmat2=[rotmat1 [0 0;0 0];[0 0;0 0] rotmat1];
%angleTurn1 = 0;
angleTurn2 = 0;
angleCount1=0;
angleCount2=0;

% rotate rectangular region to global frame and draw
%figure(7);clf;hold on;
rectReg=[-rectWidth 0 +rectWidth 0;...
        -rectWidth rectLength rectWidth rectLength;...
        -rectWidth 0 -rectWidth rectLength;...
        rectWidth 0 rectWidth rectLength ];
rotmat3=[cos(-roboPose(3)+pi/2) sin(-roboPose(3)+pi/2) ; -sin(-roboPose(3)+pi/2) cos(-roboPose(3)+pi/2)];
rotmat3=[rotmat3 [0 0;0 0];[0 0;0 0] rotmat3];
rectRegGlobalFrame = ( rotmat3 * rectReg' )';
rectRegGlobalFrame(1:4,1:2:3) = rectRegGlobalFrame(1:4,1:2:3) + roboPose(1);
rectRegGlobalFrame(1:4,2:2:4) = rectRegGlobalFrame(1:4,2:2:4) + roboPose(2);
for r=1:4,
    plot([rectRegGlobalFrame(r,1) rectRegGlobalFrame(r,3)],[rectRegGlobalFrame(r,2) rectRegGlobalFrame(r,4)],'b-')
end


    %figure(77); clf; hold on;
if size(startnstopp,1)>0,        
        startnstopp(:,1:2:4)  = startnstopp(:,1:2:4) - roboPose(1);
        startnstopp(:,2:2:5)= startnstopp(:,2:2:5) - roboPose(2);
        startnstoppRobotFrame = ( rotmat2 * startnstopp' )';
    for j=1:1:size(startnstopp,1),           
            %debug
            %plot all line-segments from the robots  point of view
%            plot([startnstoppRobotFrame(j,1) startnstoppRobotFrame(j,3)],[startnstoppRobotFrame(j,2) startnstoppRobotFrame(j,4)],'LineWidth',2.7,'Color',[1 0 0]);
%            plot( startnstoppRobotFrame(j,1),startnstoppRobotFrame(j,2),'kx','LineWidth',2,'MarkerSize',9);
%            plot( startnstoppRobotFrame(j,3),startnstoppRobotFrame(j,4),'kx','LineWidth',2,'MarkerSize',9); 
    end
    
end
if size(beacons,1)>0,
       beacons(:,1)  = beacons(:,1) - roboPose(1);
       beacons(:,2)= beacons(:,2) - roboPose(2);
       beaconsRobotFrame(:,1:2)=(rotmat1*(beacons(:,1:2)'))';
                
           %debug
           %plot all beacons from the robots  point of view
            
           %plot( beaconsRobotFrame(j,1),beaconsRobotFrame(j,2),'kx','LineWidth',2,'MarkerSize',9);
end

    
    
obstruction=1;       
color=1;

angleTurn1 = pi/4;
rotmat1=[cos(angleTurn1) sin(angleTurn1) ; -sin(angleTurn1) cos(angleTurn1)];
rotmat2=[rotmat1 [0 0;0 0];[0 0;0 0] rotmat1];    
if size(startnstopp,1)>0,      
    startnstoppRobotFrame(:,1:1:4)=(rotmat2*(startnstoppRobotFrame(:,1:1:4)'))'; 
end
    
if size(beacons,1)>0,              
    beaconsRobotFrame(:,1:2)=(rotmat1*(beaconsRobotFrame(:,1:2)'))';
end
%angleTurn2=angleTurn1;

counter=0;
while( obstruction && angleCount1 <= 9  )

    obstruction=0;
    color=color*0.90;
    counter = counter+1;
        
    if(counter > 15)
        disp('navigation error');
        return;            
    end;    
        
    for j=1:1:size(startnstopp,1)
            
        if( all(startnstoppRobotFrame(j,:)==0))
            continue;
        end;

        %debug
%       plot([startnstoppRobotFrame(j,i) startnstoppRobotFrame(j,i+2)],[startnstoppRobotFrame(j,i+1) startnstoppRobotFrame(j,i+3)],'LineWidth',2.7,'Color',[color (1-color) 0]);
%       plot( startnstoppRobotFrame(j,i),startnstoppRobotFrame(j,i+1),'kx','LineWidth',2,'MarkerSize',9);
%       plot( startnstoppRobotFrame(j,i+2),startnstoppRobotFrame(j,i+3),'kx','LineWidth',2,'MarkerSize',9);

        if( startnstoppRobotFrame(j,2) > rectLength && startnstoppRobotFrame(j,4) > rectLength)
                    %disp('here:1');
                    %no prob
        elseif( startnstoppRobotFrame(j,2) < 0 && startnstoppRobotFrame(j,4) < 0)
                    %disp('here:2');                    
                    %no prob
        elseif( startnstoppRobotFrame(j,1) < -rectWidth && startnstoppRobotFrame(j,3) < -rectWidth )
                    %disp('here:3');                    
                    %no prob
        elseif( startnstoppRobotFrame(j,1) > rectWidth && startnstoppRobotFrame(j,3) > rectWidth )
                    %disp('here:4');                    
                    %no prob
        else
                 
            deltaX = startnstoppRobotFrame(j,1)  -startnstoppRobotFrame(j,3);
            deltaY = startnstoppRobotFrame(j,2)-startnstoppRobotFrame(j,4);
                                        
            if( abs(deltaX) < 0.0001 )
                        %disp('here:6');                        
                funkYOne = startnstoppRobotFrame(j,2);
                funkYTwo = startnstoppRobotFrame(j,4);
            else
                    %disp('here:7');
                segAlpha= deltaY/deltaX;                
                funkYOne = segAlpha*(-rectWidth-startnstoppRobotFrame(j,1))+startnstoppRobotFrame(j,2);
                funkYTwo = segAlpha*( rectWidth-startnstoppRobotFrame(j,1))+startnstoppRobotFrame(j,2);
            end;
            
            if( funkYOne < 0 && funkYTwo < 0 )
                     %no prob
                     %disp('no prob1');
            elseif( funkYOne > rectLength && funkYTwo > rectLength )
                         %no prob
                         %disp('no prob2');                     
            else
                    %prob
                angleTurn1 = -pi/12; %-15 degrees
                        
                    %disp('obstruction at degs:');
                    %disp((pi/4+angleCount*angleTurn)*180/pi);
                    %disp(counter);                                                           
                        
                rotmat1=[cos(angleTurn1) sin(angleTurn1) ; -sin(angleTurn1) cos(angleTurn1)];
                rotmat1=[rotmat1 [0 0;0 0];[0 0;0 0] rotmat1];

                obstruction = 1;
                angleCount1 = angleCount1+1;
               
                for q=1:1:size(startnstopp,1)      
                    startnstoppRobotFrame(q,1:1:4)=(rotmat1*(startnstoppRobotFrame(q,1:1:4)'))';
                end
                          
                        
                
            end
                
                
            if(obstruction)
                j=1;               
                break;
            end;                
                           
        end
    end
end
counter=0;
obstruction=1;       
color=1;
while( obstruction && angleCount2 <= 9  )

    obstruction=0;
    color=color*0.90;
    counter = counter+1;
        
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
            
            angleTurn2 = -pi/12;
                        
            rotmat2=[cos(angleTurn2) sin(angleTurn2) ; -sin(angleTurn2) cos(angleTurn2)];
                

            obstruction = 1;
            angleCount2 = angleCount2+1;
                   
            for q=1:1:size(beacons,1)            
                beaconsRobotFrame(q,1:2)=(rotmat2*(beaconsRobotFrame(q,1:2)'))';
            end;          
                       
                
        end
        if(obstruction)
            j=1;
                              
            break;
        end;                
    end;
end;
        
angleTurn=min(angleTurn1,angleTurn2);
angleCount=max(angleCount1, angleCount2);

turnRad = pi/4 + angleTurn*angleCount;
if(turnRad < 0)
   turnRad = turnRad + 2*pi;
elseif(turnRad > 2*pi)
    turnRad = turnRad - 2*pi;
end;

