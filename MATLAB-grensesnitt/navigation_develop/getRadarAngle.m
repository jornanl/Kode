function [myError,angle]=getRadarAngle(myCon,myHandles)
%GETRADARANGLE
% returns radar angle in radians
%
%
myError = 1;
angle=[];

if strcmp(myCon,'simulator'),
    disp('getRadarAngle called during simulation..')
    myError=0;
    
else
    if( serialAsyncWrite(myCon,'t') )
        set(myHandles.statusText,'String','Unable to send serial data' );
        return;
    else
        [myError,data]=serialSyncRead(myCon,1);
        if(myError)
            return;
        elseif data=='r'
            % fetch radar angle
            [myError,data]=serialSyncRead(myCon,2);
            if(myError)
                disp('navigation_develop/getRobotPose: Error receiving radar angle from robot')
                return;
            else
                % data given in deg (0-180)
                 % myTheta 0-2*pi
                 myTheta = (data(1)*256+data(2)) * 2 * pi/180;
       
            end
            
            angle=myTheta;
        else
            disp('navigation_develop/getRobotPose: Wrong header received ')
        end
    end
end


