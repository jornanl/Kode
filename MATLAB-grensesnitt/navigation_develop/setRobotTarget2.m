function [error,outData]=setRobotTarget2(SerialLink,rAngle,rDist,handles)

error= 1;
outData=[];

if strcmp(SerialLink,'simulator'),
    simRobot9(horzcat('h',rAngle),handles);
    % update simulation
%    handles.simParams.theta=newHeading;
    outData=simRobot9(horzcat('d',rDist),handles);
%    data=[pose(1) pose(2) newHeading];
    error=0;
        
else
    % 
    
    if(rAngle~=-1),
        error = serialAsyncWrite( SerialLink,'h');
        
        error = error+serialAsyncWrite( SerialLink,rAngle);
    end

    error = error+serialAsyncWrite( SerialLink,'j');
    error = error+serialAsyncWrite( SerialLink,char(rDist));
    
    if error,
        disp('error sending heading and distance to robot')
    else
        % robot driving, waiting to reach target destination
        % scanning and driving...receive sensordata here...
        t=0;
        [myNewpose,error]=getRobotPose(SerialLink,handles);
        
        while t<5,
            myLastpose = myNewpose;
            [myNewpose,error]=getRobotPose(SerialLink,handles);
            if(error)
                disp('error fetching robot pose');
            else
                if myLastpose == myNewpose,
                    t = t+1;
                end              
            end
            
            pause(0.5)
        end
        bytes_availableSetRobotTarget=SerialLink.bytesAvailable;
        if bytes_availableSetRobotTarget,
            data = fread(SerialLink);
            disp('leftover data on serialconnection after setRobotTarget: ')
            disp(double(data))
        end
        
    end
end
    

