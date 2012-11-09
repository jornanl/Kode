function [error]=finalResetPose(SerialLink,handles) % Andreas H 2008
    error=0;
    clearPose(SerialLink,handles);
    
    myY = zeros(2,2);
    for i=1:1:2
        error = error+serialAsyncWrite( SerialLink,num2str(i*2));
        if error,
            disp('error sending target position to robot')
        else
            [error,data]=serialSyncRead(SerialLink,1);
            if (error),
                set(handles.statusText,'String','Feil');
            elseif (data(1) == '6' || data(1) == '8')
                set(handles.statusText,'String', 'No data');
            elseif (data(1) == '2' || data(1) == '4') 
               [error,data3]=serialSyncRead(SerialLink,4);
               if (error),
                    return
               else
                    %converting from signed int,

                    ypos = data3(3)*256+data3(4);
                    if(ypos > 32767)% negative value
                        myY(i)= -(65536-ypos);
                    else
                        myY(i)=ypos;
                    end
                end
            end
        end
    end
    if (myY(1)- myY(2))> 510   %One sensor too close to the wall. Using the other one to set position
        if (abs(myY(1))> abs(myY(2)))
            robotY = -myY(1)+250;
        else
            robotY = -myY(2)-250;
        end
    else
        robotY = -(myY(1)+myY(2))/2;
        %set(handles.statusText,'String',strcat('Left: ',num2str(myY(1)),'   Høyre:', num2str(myY(2)), '   Posisjon:',num2str(robotY)));
    end
    myPose=[0 robotY 0]; 
    error=setRobotPose(SerialLink,handles, myPose);
    
    if( error )
        set(handles.statusText,'String','Unable to send serial data' );
    end  
end

    

