function clearPose(myCon,myHandles)


if strcmp(myCon,'simulator'),
    simRobot9('c',myHandles);
        
else
    if( serialAsyncWrite(myCon,'c') )
        set(myHandles.statusText,'String','ClearPose: Unable to send serial data' );
    end;    
end