function c=lineNavigationTest

disp('---------STARTING------')

%startnstopp=[1 1 2 2;-0.5 1 -2 -2;0 -1 0 -0.5;-0.35 0.1 0.5 1]
%startnstopp=[-0.35 0.1 0.5 1;-0.6 0.1 0.5 0.7;0 0.6 0.33 0]
startnstopp=[1 1 2 2 -0.5 0.25 1 0.25]
roboPose=[0 0 0]';

rectLength=0.3;
rectWidth=0.15;

%rotate all segments from global frame to robot frame
startnstoppRobotFrame=-1;
%rotmat = [cos(roboPose(3)-pi/2) sin(roboPose(3)-pi/2) ; -sin(roboPose(3)-pi/2) cos(roboPose(3)-pi/2)];
%rotmat = [rotmat [0 0;0 0];[0 0;0 0] rotmat];

figure(8);
clf;
hold on;

rect = [[-rectWidth +rectWidth;-rectWidth +rectWidth;-rectWidth -rectWidth;rectWidth rectWidth],[0 0;rectLength rectLength;0 rectLength;0 rectLength]];
plot([-rectWidth +rectWidth],[0 0],'b',[-rectWidth +rectWidth],[rectLength rectLength],'b',[-rectWidth -rectWidth],[0 rectLength],'b',[rectWidth rectWidth],[0 rectLength],'b');

if( startnstopp == -1 )

else   
    
%     for j=1:1:size(startnstopp,1)
%         for i=1:4:size(startnstopp,2)
%           
%             startnstopp(j,i:2:i+3)  = startnstopp(j,i:2:i+3)  - roboPose(1);
%             startnstopp(j,i+1:2:i+3)= startnstopp(j,i+1:2:i+3)- roboPose(2);
%             startnstoppRobotFrame(j,i:1:i+3)=(rotmat*([startnstopp(j,i:1:i+3)]'))';
%             %plot all line-segments from the robots  point of view
%         
%             plot([startnstoppRobotFrame(j,i) startnstoppRobotFrame(j,i+2)],[startnstoppRobotFrame(j,i+1) startnstoppRobotFrame(j,i+3)],'LineWidth',2.7,'Color',[0 0 0]);
%             plot( startnstoppRobotFrame(j,i),startnstoppRobotFrame(j,i+1),'kx','LineWidth',2,'MarkerSize',9);
%             plot( startnstoppRobotFrame(j,i+2),startnstoppRobotFrame(j,i+3),'kx','LineWidth',2,'MarkerSize',9);
%         end;
%     end;

    %wall inside region check. checking form -45 to 90 deg. eg, we
    %try to follow the left wall.

    obstruction=1;       
    angleTurn = 0;
    j=1;
    i=1;
    color=1;
    counter = 0;

    
    for q=1:1:size(startnstopp,1)      
        for r=1:4:size(startnstopp,2)          
            startnstoppRobotFrame(q,r:1:r+3)=startnstopp(q,r:1:r+3);
        end;
    end;
    
    %utJA=    startnstoppRobotFrame
    %utTUT = startnstopp    
    
    while( obstruction && counter < 9 )
        
        color=color*0.8;
        obstruction=0;
        
        for j=1:1:size(startnstopp,1) 
            
            for i=1:4:size(startnstopp,2)

                if( startnstoppRobotFrame(j,i)==0 && startnstoppRobotFrame(j,i+1) == 0 && startnstoppRobotFrame(j,i+2)==0 && startnstoppRobotFrame(j,i+3) ==0)
                    continue;
                end;
                
                plot([startnstoppRobotFrame(j,i) startnstoppRobotFrame(j,i+2)],[startnstoppRobotFrame(j,i+1) startnstoppRobotFrame(j,i+3)],'LineWidth',2.7,'Color',[color color*0.5 1-color]);
                plot( startnstoppRobotFrame(j,i),startnstoppRobotFrame(j,i+1),'kx','LineWidth',2,'MarkerSize',9);
                plot( startnstoppRobotFrame(j,i+2),startnstoppRobotFrame(j,i+3),'kx','LineWidth',2,'MarkerSize',9);

                if( startnstoppRobotFrame(j,i+1) > rectLength && startnstoppRobotFrame(j,i+3) > rectLength)
                    %no prob
                elseif( startnstoppRobotFrame(j,i+1) < 0 && startnstoppRobotFrame(j,i+3) < 0)
                    %no prob
                elseif( startnstoppRobotFrame(j,i) < -rectWidth && startnstoppRobotFrame(j,i+2) < -rectWidth )
                    %no prob
                elseif( startnstoppRobotFrame(j,i) > rectWidth && startnstoppRobotFrame(j,i+2) > rectWidth )
                    %no prob
                else
                    deltaX = startnstoppRobotFrame(j,i)  -startnstoppRobotFrame(j,i+2);
                    deltaY = startnstoppRobotFrame(j,i+1)-startnstoppRobotFrame(j,i+3);
    segAlpha = pi;
                    if( abs(deltaX) < 0.0001 )
                        funkYOne = startnstoppRobotFrame(j,i+1);
                        funkYTwo = startnstoppRobotFrame(j,i+3);
                        disp('hohoh');
                        disp(deltaX);
                    else
                        segAlpha= deltaY/deltaX;
                
                        funkYOne = segAlpha*(-rectWidth-startnstoppRobotFrame(j,i))+startnstoppRobotFrame(j,i+1);
                        funkYTwo = segAlpha*( rectWidth-startnstoppRobotFrame(j,i))+startnstoppRobotFrame(j,i+1);
                    end;
            
                    if( funkYOne < 0 && funkYTwo < 0 )
                         %no prob
                         disp('no prob');
                    elseif( funkYOne > rectLength && funkYTwo > rectLength )
                         %no prob
                         disp('no prob');                     
                    else
                        %prob
                        
                        disp('obstruction at degs:');
                        disp(angleTurn*180/pi);
                        disp(counter);
                        ut1=funkYOne
                        ut2=funkYTwo                       
                        ut3 =  segAlpha
                        
                        angleTurn = angleTurn + pi/12;
                        rotmat=[cos(angleTurn) sin(angleTurn) ; -sin(angleTurn) cos(angleTurn)];
                        rotmat=[rotmat [0 0;0 0];[0 0;0 0] rotmat];

                        obstruction = 1;                    
                        counter = counter +1;                  
                    
                        for q=1:1:size(startnstopp,1)      
                            for r=1:4:size(startnstopp,2)          
                                startnstoppRobotFrame(q,r:1:r+3)=(rotmat*([startnstopp(q,r:1:r+3)]'))';
                            end;
                        end;          
                        
                        pause;
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
        end;        
    end;
end;

disp('turn this much to avoid wall:')
disp(angleTurn*180/pi);


