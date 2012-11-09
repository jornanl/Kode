function [myCon]=connectTo(handles)
    contents = get(handles.COMValg,'String'); %returns COMValg contents as cell array
    port = contents{get(handles.COMValg,'Value')}; %returns selected item from COMValg
    if strcmp(port,'simulator')
        myCon=port;
        
        % setting default simulation properties
        handles.simParams.x=0;
        handles.simParams.y=0;
        handles.simParams.theta=0;
        
        set(handles.statusText,'String',strcat('Connected to:  ',port ))
    elseif strcmp(port,'NXT')
        myCon=port;
        nxt_init();
        set(handles.statusText,'String',strcat('Connected to:  ',port ))
    elseif strcmp(port, 'NXT_kamera')
        myCon=port;
        nxt_cam_init(handles);
        set(handles.statusText,'String',strcat('Connected to:  ',port ))
    else
        myCon = serial(port);
        set(myCon,'BaudRate',57600);
        set(myCon,'TimeOut',10); %10sec timeout
        try
            fopen(myCon);
            serialAsyncWrite(myCon,'m');    %Set robot to machine mode
            set(handles.statusText,'String',strcat(port,' is:', myCon.status, ' at baudrate:', int2str(myCon.BaudRate) ));        
        catch
            set(handles.statusText,'String',strcat(port,' is not available' ));
            stopasync(myCon);
            fclose(myCon);
            delete(myCon);
            myCon = 0;        
        end;
    end