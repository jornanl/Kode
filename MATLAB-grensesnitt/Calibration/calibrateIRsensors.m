function [myError,outData] = calibrateIRsensors(SerialLink,myHandles,sensornr)

myError = 1;
outData = 0;
analogData = [];


if strcmp(SerialLink,'simulator'),
    disp('Cannot calibrate IR sensors in simulation mode');

else
    
    myError = serialAsyncWrite( SerialLink,'i');
        
    myError = myError+serialAsyncWrite( SerialLink,sensornr); % Sends the number of the sensor that is going to be calibrated, 0,1,2 or 3.
    
    if(myError)
        disp('Error sending calibration and sensor number to robot');
    else
        t = 0;
        [myError, data]=serialSyncRead(SerialLink,1);
        while data~='!' && t < 1, % While the robot has not sent the command that it is finished with sending the analog values 
            if ~myError
                analogData = [analogData; data]; % Put the analog values from the sensor in an column vector
            else
                disp(' No bytes received from robot')
                t=t+1;
            end 
            if ~myError
                [myError, data]=serialSyncRead(SerialLink,1);
            end
        end
        
        if ~myError && length(analogData)>= 3,
            analogData = sort(analogData); % Sorts the analog values in ascending order
        
            value1 = analogData(floor(length(analogData)/2),:); 	
            value2 = analogData(floor(length(analogData)/2)+1,:); 	
            value3 = analogData(floor(length(analogData)/2)-1,:); 	
        
            outData = (value1 + value2 + value3)/3; % The mean value of the three analog values in the middle of analogData
        
        else
            outData = 0;
        end
    end
end
    
