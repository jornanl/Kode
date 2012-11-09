function [poses,myError]=getRobotPose(myCon,myHandles)

myError = 0;
poses=0;

inte=1;

if strcmp(myCon,'simulator'),
    poses=simRobot9('gp',myHandles); % [1 x 3]
    myError=0;
    
elseif strcmp(myCon, 'NXT_kamera'),
    myXY=mapping_get_robot_pos(myHandles.kamerarobot); 
    poses(1)=myXY(1)*10;  %convert from cm to mm
    poses(2)=myXY(2)*10;  %convert from cm to mm
    poses(3)=mapping_get_robot_theta(myHandles.kamerarobot);  %positive radians 0-2pi
    myError=0;
    
else
    if( serialAsyncWrite(myCon,'p') )
        set(myHandles.statusText,'String','getRobotPose: Unable to send serial data' );
        return;
    else
        poseRead=0;
        while ~poseRead,
        [myError,data]=serialSyncRead(myCon,1);
        if(myError)
            disp('getRobotPose: no data received..')
            return;
        elseif data=='p'
            % fetch position data
            disp('pose header received')
            
            [myError,pose]=serialSyncRead(myCon,6);
            if(myError)
                disp('navigation_develop/getRobotPose: Error receiving postion from robot')
                return;
            else
                %converting from signed int,
                xposi = pose(1)*256+pose(2);
                if( xposi > 32767) % negative value
                    myX= -(65536-xposi);
                else
                    myX=xposi;
                end
          
                yposi = pose(3)*256+pose(4);
                if(yposi > 32767)% negative value
                    myY= -(65536-yposi);              
                else
                    myY=yposi;
                end
        
                % theta always positive 0-2*pi
                myTheta = (pose(5)*256+pose(6))/1000;
       
            end
            poseRead=1;
            
            poses(1)=myX;
            poses(2)=myY;
            poses(3)=myTheta;
        elseif data=='1' || data=='2' || data=='3' || data=='4' || data=='5' || data=='6' || data=='7' || data=='8',      
            disp('navigation_develop/getRobotPose: IR data header received')
            
        elseif data=='#'
            c=clock;
            str=sprintf('%d:\t%d:\t%d\t\t ##### DEBUG MARKER 1 #####',c(4),c(5),c(6));
            disp(str);        
            %disp(sprintf('%d', c(6)));
        elseif data=='$'
            c=clock;
            inte = inte+1;
            str=sprintf('%d - %d:\t%d:\t%d\t\t $$$$$ DEBUG MARKER 2 $$$$$--$',inte,c(4),c(5),c(6));
            disp(str);
            
        elseif data=='%'
            c=clock;
            str=sprintf('%d:\t%d:\t%d\t\t %%%%%%%%%% DEBUG MARKER 3 %%%%%%%%%%----%%',c(4),c(5),c(6));
            disp(str);
            
        % DEBUGGING - RECEIVE TOTAL TICKS
%         elseif data==char(1)
%             [myError,ticks]=serialSyncRead(myCon,2);
%             if(myError)
%                 disp('navigation_develop/getRobotPose: Error receiving left wheel ticks from robot')
%                 return;
%             else
%                 ticks_temp = ticks(1)*256+ticks(2);
%                 if( ticks_temp > 32767) % negative value
%                     ticks_left= -(65536-xposi);
%                 else
%                     ticks_left=ticks_temp;
%                 end
%                 disp('left wheel ticks: ')
%                 ticks_left
%             end
%         
        elseif data=='w', % collision warning maybe < 10 cm
            disp('collision warning received')
            set(myHandles.statusText,'String',num2str('collision warning received') );
        elseif data=='e', % collision error < 5 cm (Robot stopped!)
            disp('collision error received, robot stopped')
            myError=1;
            break        
            
        else
            disp('navigation_develop/getRobotPose: Wrong header received ')
            disp(data)
            pause(0.1);
        end
        end
    end
end


