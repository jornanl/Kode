function [error]=docking(SerialLink,handles,lineParam) % Andreas H 2008
    
    chargerDetected = 0;

    lineParam.windowsize = 5;
    lineParam.threshfidel= 2;
    lineParam.fusealpha= 0.7;
    lineParam.minlength= 0.10;
    lineParam.compensa= 0.017453;        
    lineParam.compensr= 0.01;        
    lineParam.cyclic= 1;        
    lineParam.alpha= 0.85;
    lineParam.sensor.stdrho = 0.02;
    lineParam.robot.formtype = get(handles.robotType,'Value')-1;
    
    lineParam.robot.xsensor = [0 0 0];
    IRdata.params.xs=lineParam.robot.xsensor;
    IRdata.params.stdrho = lineParam.sensor.stdrho;
    
    alfaThreshold = pi/10; % Defining threshold for "parallellness"
    wallDist = 0.46;
    rThreshold = 0.05;
    
    %Returning to an approximate angle of zero degrees
    error=setRobotTarget2(SerialLink,0,0,handles);

    if( error )  
        set(handles.statusText,'String','Unable to send turn command.' );    
    else
        set(handles.statusText,'String',strcat('Turning...') );
    end 
    
    [myNewpose,error]=getRobotPose(SerialLink,handles);
    if(error)
        disp('error fetching robot pose');
    end;
    
    % display current position in Gui
    set(handles.xPosText,'String',num2str(myNewpose(1)) );
    set(handles.yPosText,'String',num2str(myNewpose(2)) );
    set(handles.tPosText,'String',num2str(myNewpose(3)*180/pi) );
    
    myNewpose=[myNewpose(1)/1000 myNewpose(2)/1000 myNewpose(3)];
    [error,data]=fullScan(SerialLink,handles);
    data(:,2:3)=data(:,2:3)/1000; % converting from mm to meter

    % rotate the global x,y points received from robot to local x,y points
    % in order to use them in extractlines
    % Muligens feil rotasjonsmatrise
    % rotmat=[cos(myNewpose(3)) -sin(myNewpose(3)); sin(myNewpose(3)) cos(myNewpose(3)) ];
    rotmat=[cos(myNewpose(3)) sin(myNewpose(3)); -sin(myNewpose(3)) cos(myNewpose(3)) ];

    data(:,2)=data(:,2)-myNewpose(1);
    data(:,3)=data(:,3)-myNewpose(2);

    data(:,2:3)=(rotmat*(data(:,2:3))')';

    %data(find(-0.1<data(:,3) & data(:,3)<0.1),:)=[];

    IRdata.steps.data=[];
    NS=4;
    for sens=1:1:NS,
        tmpData=data(data(:,1)==sens,:);

        IRdata.steps.data=vertcat(IRdata.steps.data,tmpData);
    end
    
    IRdata.steps.data1=[];
    IRdata.steps.data2=[];
    IRdata.steps.data3=[];
    if size(IRdata.steps.data,1)>0,
        IRdata.steps.data1=IRdata.steps.data(:,1);
        IRdata.steps.data2=IRdata.steps.data(:,2);
        IRdata.steps.data3=IRdata.steps.data(:,3);
    end
    
    %Extracting lines
    [L,rawSegs,lines] = extractlines( IRdata ,lineParam,myNewpose,1);
    
     for i=1:1:size(lines,2)
         if lines(5,i)>2*pi,
             lines(5,i) = lines(5,i)- 2*pi;
         elseif lines(5,i)<0,
             lines(5,i) = lines(5,i)+2*pi;
         end
     end
     
     if size(lines,2)>1 % If more than one line exists
         for i=1:1:(size(lines,2)-1) %Compare all angles in search of parallell lines
             for j=i+1:1:size(lines,2)
                if (abs(lines(5,i)-lines(5,j)) > pi-alfaThreshold && abs(lines(5,i)-lines(5,j)) < pi+alfaThreshold) % Detecting parallell lines
                    if ((lines(6,i)+lines(6,j)) > (wallDist-rThreshold) && (lines(6,i)+lines(6,j)) < (wallDist+rThreshold)) % Distance between detected walls within threshold
                        chargerDetected = 1;
                        if lines(5,i)<lines(5,j)%Calculate wich direction to turn
                            heading = (lines(5,i)-pi/2)*180/pi;
                        else
                            heading = (lines(5,j)-pi/2)*180/pi;
                        end

                        disp('adding current position');
                        tmpHead = floor((heading/1.85)+(myNewpose(3)*180/pi)/2 ); % 1.85 is compensating scaling factor...
                                                
                        if tmpHead < 0
                            disp('Scaling to 180')
                            tmpHead = tmpHead + 180;
                        elseif tmpHead >= 180
                            disp('Scaling to 180')
                            tmpHead = tmpHead - 180;
                        end
                
                        error=setRobotTarget2(SerialLink,tmpHead,0,handles);

                        if( error )  
                            set(handles.statusText,'String','Unable to send turn command.' );    
                        else
                            set(handles.statusText,'String',strcat('Turning...') );
                        end 
                    end
                end
             end
         end
     end
     if chargerDetected,
        error = serialAsyncWrite( SerialLink,'x'); %Backing up towards charger plates
        if( error )  
            set(handles.statusText,'String','Unable to send serial data.' );    
        else
            set(handles.statusText,'String',strcat('Backing up') );
        end

        pause(5)

        for i=1:1:5 %Backing up several times because of collision warning
            error = serialAsyncWrite( SerialLink,'x');
            if( error )  
                set(handles.statusText,'String','Unable to send serial data.' );    
            end
            pause(2)
        end
        %Clearing serial connection
        bytes_availableSetRobotTarget=SerialLink.bytesAvailable;
        if bytes_availableSetRobotTarget,
            data = fread(SerialLink);
            disp('leftover data on serialconnection after setRobotTarget: ')
            disp(double(data))
        end
        %Resetting position estimate according to current position
        error=finalResetPose(SerialLink,handles);
     else
         set(handles.statusText,'String','No entrance detected...' );
     end
         
    bytes_availableSetRobotTarget=SerialLink.bytesAvailable;
    if bytes_availableSetRobotTarget,
        data = fread(SerialLink);
        disp('leftover data on serialconnection after setRobotTarget: ')
        disp(double(data))
    end
end

    

