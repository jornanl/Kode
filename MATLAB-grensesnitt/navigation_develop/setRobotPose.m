function [error]=setRobotPose(SerialLink,handles,pose)
global ROBOTPOSE;

error= 0;

if strcmp(SerialLink,'simulator'),
    ROBOTPOSE(1)=pose(1)/10;
    ROBOTPOSE(2)=pose(2)/10;
    ROBOTPOSE(3)=pose(3);
    error=0;
        
else
    x=pose(1);y=pose(2);t=pose(3)*pi/180*1000;
    
    t1 = fix(t/256 );
    t2 = t - t1/256;
    
    if x<0,
        x=2^16 +x;
    end   
    x1=fix(x/256);
    x2=x - x1*256;
    
    if y<0,
        y=2^16 +y;
    end
       
    y1=fix(y/256);
    y2=y - y1*256;
    
    error = error+serialAsyncWrite( SerialLink,'q');
    error = error+serialAsyncWrite( SerialLink,char(x1));
    error = error+serialAsyncWrite( SerialLink,char(x2));
    error = error+serialAsyncWrite( SerialLink,char(y1));
    error = error+serialAsyncWrite( SerialLink,char(y2));
    error = error+serialAsyncWrite( SerialLink,char(t1));
    error = error+serialAsyncWrite( SerialLink,char(t2));
    
    if (error),
        disp('error setting position in robot')
    else
        % receive ACK if ok
        
        [error, data]=serialSyncRead(SerialLink,1);
        if error,
            disp('setRobotPose: No ACK setting robot position received')    
        elseif data=='a'
            % intern pose set
            set(handles.statusText,'String','Robot intern position set' );
            
            % also display result in Gui
            set(handles.xPosText,'String',num2str(pose(1)) );
            set(handles.yPosText,'String',num2str(pose(2)) );
            set(handles.tPosText,'String',num2str(pose(3)) );   
            
            disp('set robot position to:')
            disp(pose);
        else
           disp('Error in setRobotPose: wrong ACK from robot during set intern pose') 
        end
        
    end
           
end