function [error]=dockingApproach(SerialLink,handles,lineParam) %Experimantal function. Not implemented... Andreas H
    
    chargerDetected = 0;

    lineParam.windowsize = 5;
    lineParam.threshfidel= 1.5;
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
    rotmat=[cos(myNewpose(3)) sin(myNewpose(3)); -sin(myNewpose(3)) cos(myNewpose(3)) ];

    data(:,2)=data(:,2)-myNewpose(1);
    data(:,3)=data(:,3)-myNewpose(2);
    data(:,2:3)=(rotmat*(data(:,2:3))')';

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
    
    bestFit = inf;
    bestFitIndex = 1;
    
    %Wall detected?
     if size(rawSegs,2),
    %Searching for line closest to 180 degrees
         for i=1:1:size(rawSegs,2)
             temp = (rawSegs(5,i)+myNewpose(3))*180/pi;
             if temp > 180,
                 temp = temp-360;
             elseif temp < -180,
                 temp = temp+360;
             end

             if abs(abs(temp)-180) < bestFit,
                 bestFit = abs(abs(temp)-180);
                 bestFitIndex = i;
             end
         end
         disp(rawSegs(:,bestFitIndex))
         
         headingToWall = (rawSegs(5,bestFitIndex)+myNewpose(3))*180/pi;
         
         distanseToWall = rawSegs(6,bestFitIndex);
         %Position robot 20-25 cm from wall
         if distanseToWall > 0.25,
            error = setRobotTarget2(SerialLink,char(floor(headingToWall/2)),char(floor(distanseToWall*100-20)),handles);
            if error,
                set(handles.statusText,'String','Error going to pose' );
            else
                set(handles.statusText,'String',strcat('Running' ));
            end
         elseif distanseToWall < 0.2,
             disp('Close to wall')
         end
     else
         set(handles.statusText,'String','No wall detected' );
     end
     
%     disp(rawSegs(5,:)*180/pi)
%     disp(bestFit)
%     disp(rawSegs(:,bestFitIndex))
         
    bytes_availableSetRobotTarget=SerialLink.bytesAvailable;
    if bytes_availableSetRobotTarget,
        data = fread(SerialLink);
        disp('leftover data on serialconnection after setRobotTarget: ')
        disp(double(data))
    end
end

    

