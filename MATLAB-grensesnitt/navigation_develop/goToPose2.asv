function [error,outData]=goToPose2(SerialLink,x,y,handles)
% x,y in [mm]
global ROBOTPOSE



if strcmp(SerialLink,'simulator'),
    %ROBOTPOSE(1)=double(x)*10;
    %ROBOTPOSE(2)=double(y)*10;
    [phi,rDist]=cart2pol(x/10-ROBOTPOSE(1),y/10-ROBOTPOSE(2));
    if phi<0,
        phi=phi+2*pi;
    end
    tmp = floor( phi*180/pi /2 );
    rAngle = char( tmp );
    
    setRobotTarget(SerialLink,rAngle,rDist,handles)
    error=0;
        
else
   
    
    if x<0,
        
        %xx=binvec2dec(not(dec2binvec(-1*x)));
        %x=xx+1;
        x=2^16 +x;
    end
       
    x1=fix(x/256);
    x2=x - x1*256;
    
    if y<0,
        %y=y*-1
        %yy=dec2binvec(y);
        %yy=not(yy);
        %yy=binvec2dec(yy);
        %y=yy+1;
        y=2^16 +y;
    end
       
    y1=fix(y/256);
    y2=y - y1*256;
    
        
    error=0;
    error = error+serialAsyncWrite( SerialLink,'g');
    error = error+serialAsyncWrite( SerialLink,char(x1) );
    error = error+serialAsyncWrite( SerialLink,char(x2) );
    
    
    error = error+serialAsyncWrite( SerialLink,char(y1) );
    error = error+serialAsyncWrite( SerialLink,char(y2) );
    
    if error,
        disp('error sending target position to robot')
    else
%        [error, data]=serialSyncRead(SerialLink,1);
%        if error || data~='n',
%            disp('target position not acknowledged from robot')
%        else
%        
%        end
%    end
        % robot driving, waiting to reach target destination
        % scanning and driving...receive sensordata here...
        t=0;
        [myNewpose,error]=getRobotPose(SerialLink,handles);
        
        while t<3,
            disp('Kjører sløyfe')
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
    

