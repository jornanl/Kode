function [error,outData]=setRobotTarget(SerialLink,rAngle,rDist,handles)

error= 1;
outData=[];

if strcmp(SerialLink,'simulator'),
    simRobot9(horzcat('h',rAngle),handles);
    % update simulation
%    handles.simParams.theta=newHeading;
    outData=simRobot9(horzcat('d',rDist),handles);
%    data=[pose(1) pose(2) newHeading];
    error=0;
    
elseif strcmp(SerialLink,'NXT_kamera'),
    
    
else
    % 
    
    if(rAngle~=-1),
        error = serialAsyncWrite( SerialLink,'h');
        
        error = error+serialAsyncWrite( SerialLink,rAngle);
        
    end

    error = error+serialAsyncWrite( SerialLink,'j');
    error = error+serialAsyncWrite( SerialLink,char(rDist));
    
    if error,
        disp('error sending distance to robot')
    else
        % robot driving, waiting to reach target destination
        % scanning and driving...receive sensordata here...
        reachedTarget=0;
        i=0;
        t=0;
        while ~reachedTarget && t<1,
            [error, data1]=serialSyncRead(SerialLink,1);
            if ~error,
                switch data1,
                    case 'q', % robot arrived
                        reachedTarget=1;
                        if SerialLink.bytesAvailable
                            disp('setRobotTarget: bytes in buffer after receiving q, robot arrived')
                        end
                    case 'n',
                        % NACK
                        disp('setRobotTarget: received NACK')
                    case 'a',
                        % ACK
                        disp('setRobotTarget: received ACK for j')
                    case 'w', % collision warning maybe <15 cm
                        disp('collision warning received')
                        set(handles.statusText,'String',num2str('collision warning received') );
                    case 'e', % collision error <10 cm (Robot stopped!)
                        disp('collision error received, robot stopped')
                        reachedTarget=1;    
                    case {'5', '6', '7', '8'},
                        disp('no data: sensor  ')
                        disp(data1-52)
                        
                    case {'1', '2' , '3', '4'},
                        % read IR data
                        i=1+i;
                        disp('got sensornummer (49-52) waiting for xxyy')
                        %data(1)
                        outData(i,1)=data1-48; % sensor number #
                    
                        [myError,data2]=serialSyncRead(SerialLink,4);
                        if (myError),
                            return
                        else
                            %converting from signed int,
                            xpos = data2(1)*256+data2(2);
                            if( xpos > 32767) % negative value
                                myX= -(65536-xpos);
                            else
                                myX=xpos;
                            end

                            ypos = data2(3)*256+data2(4);
                            if(ypos > 32767)% negative value
                                myY= -(65536-ypos);
                            else
                                myY=ypos;
                            end
                    
                        end
                        outData(i,2)=myX;
                        outData(i,3)=myY;
                        
                    
                    otherwise,
                        disp('navigation_develop\setRobotTarget: Error: No action taken for the received byte..')
                        disp(double(data1))
                end
                    
            else
                disp(' no bytes received from robot')
                t=t+1;   
            end        
        end
        
        bytes_availableSetRobotTarget=SerialLink.bytesAvailable
        if bytes_availableSetRobotTarget,
            data = fread(SerialLink);
            disp('leftover data on serialconnection after setRobotTarget: ')
            disp(double(data))
        end
        
    end
end
    

