function varargout = LegoGui(varargin)
% LEGOGUI M-file for LegoGui.fig
%      LEGOGUI, by itself, creates a new LEGOGUI or raises the existing
%      singleton*.
%
%      H = LEGOGUI returns the handle to a new LEGOGUI or the handle to
%      the existing singleton*.
%
%      LEGOGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LEGOGUI.M with the given input arguments.
%
%      LEGOGUI('Property','Value',...) creates a new LEGOGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LegoGui_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LegoGui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Copyright 2002-2003 The MathWorks, Inc.
% Edit the above text to modify the response to help LegoGui

% Last Modified by GUIDE v2.5 03-Jul-2011 22:30:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LegoGui_OpeningFcn, ...
                   'gui_OutputFcn',  @LegoGui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before LegoGui is made visible.
function LegoGui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LegoGui (see VARARGIN)

%a=get(LegoGui)
% Choose default command line output for LegoGui
handles.output = hObject;
handles.lineParam.changed='false';
handles.beaconParam.changed='false';

%handles.simParams.changed='false';

handles.checkTable=[];

%need to define robot-object and vertex-list for NXT camera robot

handles.kamerarobot=[];
handles.vertex_list_left = [];
handles.vertex_list_right = [];

% Update handles structure
guidata(hObject, handles);
% UIWAIT makes LegoGui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


%%%%% defining globals %%%%%%
global SERIALLINK;
global RUNNING;
global table;

SERIALLINK = 0;
RUNNING = 0;
table = [];
%mappingChoise = 1;

%add all folders under/below the current to path
path(path,genpath(pwd));


% --- Outputs from this function are returned to the command line.
function varargout = LegoGui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

%% BASIC - define callbacks


% --- Executes on selection change in COMValg.
function COMValg_Callback(hObject, eventdata, handles)
% hObject    handle to COMValg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns COMValg contents as cell array
%        contents{get(hObject,'Value')} returns selected item from COMValg


% --- Executes during object creation, after setting all properties.
function COMValg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to COMValg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in connectButton.
function connectButton_Callback(hObject, eventdata, handles)
% hObject    handle to connectButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;
    
if(SERIALLINK==0)
    SERIALLINK = connectTo(handles);
elseif strcmp(SERIALLINK,'simulator'),
    set(handles.statusText,'String',strcat('Already connected to simulator.' ));
else
    set(handles.statusText,'String',strcat(SERIALLINK.port,' is already:', SERIALLINK.status,'. If you want to connect to another port, please close the current connection first' ));
end



% --- Executes on button press in disconnectButton.
function disconnectButton_Callback(hObject, eventdata, handles)
% hObject    handle to disconnectButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;

if( SERIALLINK == 0 )
    set(handles.statusText,'String','Already disconnected');
elseif strcmp(SERIALLINK,'simulator'),
    SERIALLINK=0;
    set(handles.statusText,'String','Disconnected from simulator');
elseif strcmp(SERIALLINK,'NXT')
    SERIALLINK=0;
    set(handles.statusText,'String','Disconnected from NXT');
elseif strcmp(SERIALLINK,'NXT_kamera')
    SERIALLINK=0;
    set(handles.statusText,'String','Disconnected from NXT_kamera robot');
else
    stopasync(SERIALLINK); % Stop asynchronous read and write
    fclose(SERIALLINK); % Close file
    set(handles.statusText,'String',strcat(SERIALLINK.port,' is:', SERIALLINK.status ));
    delete(SERIALLINK);    
    SERIALLINK = 0;
end;




% --- Executes on button press in pingButton.
function pingButton_Callback(hObject, eventdata, handles)
% hObject    handle to pingButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;
if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

set(handles.statusText,'String','Ping!........' );
pause(0.1); % Pauses for 0.1 second

if( serialAsyncWrite(SERIALLINK,'o') )   % p = ping kommand
    set(handles.statusText,'String','Unable to send serial data' );    
else
    [error,data] = serialSyncRead(SERIALLINK,1); % 1 byte response expected
    if( error )
        set(handles.statusText,'String','No response from robot. Timeout.' );
    else

        if( data == 'k') % k = ping response
            set(handles.statusText,'String','Robot responding' );
        else
            disp(data);
            set(handles.statusText,'String','Robot responding. But wrong response' );
        end;
    end;
end;


% --- Executes when user attempts to close_pushbutton figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;    

if(SERIALLINK==0)
    clear SERIALLINK;
else
    try
        stopasync(SERIALLINK);
        fclose(SERIALLINK);
        delete SERIALLINK;
        clear SERIALLINK;        
    catch
        
    end;
end;

delete(hObject);

% --- Executes on slider movement.
function angleSlider_Callback(hObject, eventdata, handles)
% hObject    handle to AngleSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

sliderVal = floor( get(hObject,'Value') );
set(handles.angleText,'String', sliderVal ); % angleText is the textbox next to the SetRadarAngle push button

% --- Executes during object creation, after setting all properties.
function angleSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AngleSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in radarButton.
function radarButton_Callback(hObject, eventdata, handles)
% hObject    handle to radarButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

tmpAngle = floor( str2double( get(handles.angleText,'String') ) );
rAngle = char( (90+tmpAngle) );
if( serialAsyncWrite( SERIALLINK,strcat('r',rAngle) ) )   % rx = set radar, x=angle
     set(handles.statusText,'String','Unable to send serial data' );    
else
    [error,data]=serialSyncRead( SERIALLINK,1);
    if error || data~='a'
        set(handles.statusText,'String','Radar angle not set' );
    else
        set(handles.statusText,'String','Radar set' );
    end
end;


% --- Executes during object creation, after setting all properties.
function angleText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angleText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in resetRobtPoseButton.
function resetRobtPoseButton_Callback(hObject, eventdata, handles)
% hObject    handle to resetRobtPoseButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

clearPose(SERIALLINK,handles);

roboposeButton_Callback(handles.roboposeButton, eventdata, handles);


% --- Executes on button press in startButton.
function startButton_Callback(hObject, eventdata, handles)
% hObject    handle to startButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global RUNNING;
global SERIALLINK;
global ROBOTPOSE;


if(RUNNING==1)
    set(handles.statusText,'String','Allready started');
    return;
end
if RUNNING==2, % user have pushed pausebutton, resume navigation..
    RUNNING=1; 
    set(handles.statusText,'String','Started');
    return
end
    
if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
    
    set(handles.statusText,'String','Started');
end;
RUNNING=1;

if strcmp(SERIALLINK,'simulator'),
    % setting default simulation properties
    ROBOTPOSE=[0 0 0];
    
    handles.simParams.maze=SimMapMenu_Callback(handles.SimMapMenu, eventdata, handles);
    handles.poseError=checkbox_posErr_Callback(handles.checkbox_posErr, eventdata, handles);
    handles.irError=checkbox_irErr_Callback(handles.checkbox_posErr, eventdata, handles);
    
    set(handles.statusText,'String','Simulation running..');
end

% sensor properties
handles.maxPerceptRadius=str2double(get(handles.maxPerceptRadius,'String'));
handles.angularRes=str2double(get(handles.angularRes,'String'));

handles.goHometext = str2double(get(handles.goHomeText,'String'));

% scan while driving
handles.driveScan=get(handles.checkbox_driveScan,'Value');
%if( get(handles.checkbox_driveScan,'Value')==1  )
%    handles.driveScan=1;
%end

% Start mapping from 0,0,0
resetRobtPoseButton_Callback(handles.resetRobtPoseButton, eventdata, handles);

% testing...
if strcmp(SERIALLINK,'simulator'), 
    setRobotPose(SERIALLINK,handles,[0 0 0]);
end

%make figure to plot image in
if strcmp(SERIALLINK,'NXT_kamera'), 
    handles.kamerafigur=figure();
end


%% Beacon navigation
if( get(handles.togglebutton_beaconNav,'Value')==1  )
    if strcmp(handles.beaconParam.changed,'false'), % parameters not manipulated, set them to standard values 
        beaconParam.beaconthreshdist = 0.02;    % Jump distance for segmentation in [m]
        beaconParam.alpha = 0.75;   % Sannsynligheten for at to punkter er ett punkt
    elseif strcmp(handles.beaconParam.changed,'true'),
        beaconParam=handles.beaconParam;
    end
    %robot parameters
    %beaconParamParam.robot.x = [str2double(get(handles.edit20,'String'));str2double(get(handles.edit19,'String'));str2double(get(handles.edit21,'String'))];
    %beaconParam.robot.xsensor = [str2double(get(handles.edit24,'String'));str2double(get(handles.edit22,'String'));str2double(get(handles.edit23,'String'))];    
    beaconParam.robot.formtype = get(handles.robotType,'Value')-1;

    %plotting parameters
    beaconParam.displayGlobal = get(handles.checkbox_globalMap,'Value');
    beaconParam.displayLocal = get(handles.checkbox_localMap,'Value');
    beaconParam.displayMatch = get(handles.checkbox_matching,'Value');    
    
    if strcmp(SERIALLINK,'simulator'),
        beaconParam.simulation = 1;
        %simParams.init=1;
        handles.simParams.startSimX=str2double(get(handles.startSimX,'String'));
        handles.simParams.startSimY=str2double(get(handles.startSimY,'String'));
        handles.simParams.widthBetweenWalls=str2double(get(handles.widthOfWalls,'String'));
        %handles.simParams.irError=-2;
        handles.simParams.maxPerceptRadius=handles.maxPerceptRadius;
        handles.simParams.angularRes=handles.angularRes;
        
    else
        beaconParam.simulation = 0;
        handles.simParams=[];
    end
    
    beaconSLAM(SERIALLINK,handles,beaconParam);

%% lineSLAM
elseif( get(handles.togglebutton_lineNav,'Value')==1 ) % linenavigation
    if strcmp(handles.lineParam.changed,'false'), % parameters not manipulated, set them to standard values 
        lineParam.windowsize = 9;
        lineParam.threshfidel= 0.05;
        lineParam.fusealpha= 0.95;
        lineParam.minlength= 0.15;
        lineParam.compensa= 0.017453;        
        lineParam.compensr= 0.01;        
        lineParam.cyclic= 1;        
        lineParam.alpha= 0.85;
        lineParam.sensor.stdrho = 0.02;
    elseif strcmp(handles.lineParam.changed,'true'),
        lineParam=handles.lineParam;
    end
    %lineParam.robot.x = [str2double(get(handles.edit20,'String'));str2double(get(handles.edit19,'String'));str2double(get(handles.edit21,'String'))];
    %lineParam.robot.xsensor = [str2double(get(handles.edit24,'String'));str2double(get(handles.edit22,'String'));str2double(get(handles.edit23,'String'))];    
    lineParam.robot.formtype = get(handles.robotType,'Value')-1;
    
    lineParam.displayGlobal = get(handles.checkbox_globalMap,'Value');
    lineParam.displayLocal = get(handles.checkbox_localMap,'Value');
    lineParam.displayMatch = get(handles.checkbox_matching,'Value');
    
    if strcmp(SERIALLINK,'simulator'),
        % simulation properties
        lineParam.simulation = 1;
        handles.simParams.startSimX=str2double(get(handles.startSimX,'String'));
        handles.simParams.startSimY=str2double(get(handles.startSimY,'String'));
        handles.simParams.widthBetweenWalls=str2double(get(handles.widthOfWalls,'String'));
%        handles.simParams.irError=-2;
        handles.simParams.maxPerceptRadius=handles.maxPerceptRadius;
        handles.simParams.angularRes=handles.angularRes;
    else
        lineParam.simulation = 0;
        handles.simParams=[];
    end
    
    %navigation type
    if( get(handles.lineNavButton1,'Value')==1)
        lineParam.navigation=1;
    end
    lineSLAM(SERIALLINK,handles,lineParam); 
    
    
%% lineBeaconSLAM
elseif( get(handles.togglebutton_lineBeaconNav,'Value')==1 ) % line/beacon navigation
    if strcmp(handles.lineParam.changed,'false'), % parameters not manipulated, set them to standard values 
        lineParam.windowsize = 9; % Size of sliding window. N første punktene i datasettet.11
        lineParam.threshfidel= 0.05; % Hvert punkt må være næremere linja enn denne verdien.
        lineParam.fusealpha= 0.95;  % Sammensmeltning av linjer gjøres når de med denne sikkerheten er én linje
        lineParam.minlength= 0.15; % minimum length a segment must have to be accepted
        lineParam.compensa= 0.017453;  % pi/180      
        lineParam.compensr= 0.01;        
        lineParam.cyclic= 1;        % 0: non-cyclic or 1: cyclic
        lineParam.alpha= 0.85;  % Sannsynligheten for at to linjer er den samme
        lineParam.sensor.stdrho = 0.02; % Beskriver en konstant radiell usikkerhet i avstandsmålingene
    elseif strcmp(handles.lineParam.changed,'true'),
        lineParam=handles.lineParam;
    end
    %lineParam.robot.x = [str2double(get(handles.edit20,'String'));str2double(get(handles.edit19,'String'));str2double(get(handles.edit21,'String'))];
    %lineParam.robot.xsensor = [str2double(get(handles.edit24,'String'));str2double(get(handles.edit22,'String'));str2double(get(handles.edit23,'String'))];    
    lineParam.robot.formtype = get(handles.robotType,'Value')-1;
    
    lineParam.displayGlobal = get(handles.checkbox_globalMap,'Value');
    lineParam.displayLocal = get(handles.checkbox_localMap,'Value');
    lineParam.displayMatch = get(handles.checkbox_matching,'Value');
    
    lineParam.goHome = get(handles.checkbox_goHome,'Value');
    
    if strcmp(SERIALLINK,'simulator'),
        % simulation properties
        lineParam.simulation = 1;
        handles.simParams.startSimX=str2double(get(handles.startSimX,'String'));
        handles.simParams.startSimY=str2double(get(handles.startSimY,'String'));
        handles.simParams.widthBetweenWalls=str2double(get(handles.widthOfWalls,'String'));
%        handles.simParams.irError=-2;
        handles.simParams.maxPerceptRadius=handles.maxPerceptRadius;
        handles.simParams.angularRes=handles.angularRes;
    else
        lineParam.simulation = 0;
        handles.simParams=[];
    end
    
    %navigation type
    if( get(handles.lineNavButton1,'Value')==1)
        lineParam.navigation=1;
    end
    

%%%-- integration of lineSLAM and beaconSLAM --%%%
    
    if strcmp(handles.beaconParam.changed,'false'), % parameters not manipulated, set them to standard values 
        beaconParam.beaconthreshdist = 0.02;
        beaconParam.alpha = 0.75;
    elseif strcmp(handles.beaconParam.changed,'true'),
        beaconParam=handles.beaconParam;
    end
    %robot parameters
    %beaconParamParam.robot.x = [str2double(get(handles.edit20,'String'));str2double(get(handles.edit19,'String'));str2double(get(handles.edit21,'String'))];
    %beaconParam.robot.xsensor = [str2double(get(handles.edit24,'String'));str2double(get(handles.edit22,'String'));str2double(get(handles.edit23,'String'))];    
    beaconParam.robot.formtype = get(handles.robotType,'Value')-1;

    %plotting parameters
    beaconParam.displayGlobal = get(handles.checkbox_globalMap,'Value');
    beaconParam.displayLocal = get(handles.checkbox_localMap,'Value');
    beaconParam.displayMatch = get(handles.checkbox_matching,'Value');   
    
    if ~strcmp(SERIALLINK,'simulator'),
        beaconParam.simulation = 0;
        handles.simParams=[];
    end
    
    
    lineBeaconSLAM(SERIALLINK,handles,lineParam,beaconParam); 
        
else
    disp('no navigation method chosen...LegoGui.m')
    
end;


set(handles.statusText,'String','Stopped');

% --- Executes on button press in stopButton.
function stopButton_Callback(hObject, eventdata, handles)
% hObject    handle to stopButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global RUNNING;
global SERIALLINK;

if ((RUNNING == 1) || (RUNNING ==2)) && ~strcmp(SERIALLINK,'simulator'),
    RUNNING = 0;
    set(handles.statusText,'String','Stopping.....Please wait');

elseif ((RUNNING == 1) || (RUNNING ==2)) && strcmp(SERIALLINK,'simulator'),
    RUNNING = 0;
    set(handles.statusText,'String','Simulation stopped by user');
else
    set(handles.statusText,'String','Not started...');
end;




% --- Executes on button press in roboposeButton.
function roboposeButton_Callback(hObject, eventdata, handles)
% hObject    handle to roboposeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

[poses,error]=getRobotPose( SERIALLINK,handles);
if( error )
    set(handles.statusText,'String','Error when prosessing getRobotPose' );
else   
    set(handles.xPosText,'String',num2str(poses(1)) );
    set(handles.yPosText,'String',num2str(poses(2)) );
    set(handles.tPosText,'String',num2str(poses(3)*180/pi) );    
end;


function xPosText_Callback(hObject, eventdata, handles)
% hObject    handle to xPosText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xPosText as text
%        str2double(get(hObject,'String')) returns contents of xPosText as a double


% --- Executes during object creation, after setting all properties.
function xPosText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xPosText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function tPosText_Callback(hObject, eventdata, handles)
% hObject    handle to tPosText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tPosText as text
%        str2double(get(hObject,'String')) returns contents of tPosText as a double


% --- Executes during object creation, after setting all properties.
function tPosText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tPosText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
%if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
%    set(hObject,'BackgroundColor','white');
%end
% Hint: slider controls usually have a light gray background.



function yPosText_Callback(hObject, eventdata, handles)
% hObject    handle to yPosText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yPosText as text
%        str2double(get(hObject,'String')) returns contents of yPosText as a double

% --- Executes during object creation, after setting all properties.
function yPosText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yPosText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
%if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
%    set(hObject,'BackgroundColor','white');
%end



% --- Executes on button press in robotForwardButton.
function robotForwardButton_Callback(hObject, eventdata, handles)
% hObject    handle to robotForwardButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

if( serialAsyncWrite(SERIALLINK,'w') )
    set(handles.statusText,'String','Unable to send serial data' );

    
end;    


% --- Executes on button press in robotStopButton.
function robotStopButton_Callback(hObject, eventdata, handles)
% hObject    handle to robotStopButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

if( serialAsyncWrite(SERIALLINK,'s') )
    set(handles.statusText,'String','Unable to send serial data' );
else
    if strcmp(SERIALLINK,'NXT')
        [myError,data]=serialSyncRead2(SERIALLINK);
    elseif SERIALLINK.bytesAvailable,
        [myError,data]=serialSyncRead2(SERIALLINK) % fetch all data received from robot
    end
end;    


% --- Executes on button press in robotLeftButton.
function robotLeftButton_Callback(hObject, eventdata, handles)
% hObject    handle to robotLeftButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

if( serialAsyncWrite(SERIALLINK,'a') )
    set(handles.statusText,'String','Unable to send serial data' );
end;    


% --- Executes on button press in robotRightButton.
function robotRightButton_Callback(hObject, eventdata, handles)
% hObject    handle to robotRightButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

if( serialAsyncWrite(SERIALLINK,'d') )
    set(handles.statusText,'String','Unable to send serial data' );
end;    


% --- Executes on button press in pushbutton_robotBackward.
function pushbutton_robotBackward_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_robotBackward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

if( serialAsyncWrite(SERIALLINK,'x') )
    set(handles.statusText,'String','Unable to send serial data' );
end;    




function statusText_Callback(hObject, eventdata, handles)
% hObject    handle to statusText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of statusText as text
%        str2double(get(hObject,'String')) returns contents of statusText as a double


% --- Executes during object creation, after setting all properties.
function statusText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to statusText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --------------------------------------------------------------------
function menuFile_Callback(hObject, eventdata, handles)
% hObject    handle to menuFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in robotType.
function robotType_Callback(hObject, eventdata, handles)
% hObject    handle to robotType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns robotType contents as cell array
%        contents{get(hObject,'Value')} returns selected item from robotType


% --- Executes during object creation, after setting all properties.
function robotType_CreateFcn(hObject, eventdata, handles)
% hObject    handle to robotType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function linesWindowsize_Callback(hObject, eventdata, handles)
% hObject    handle to linesWindowsize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linesWindowsize as text
%        str2double(get(hObject,'String')) returns contents of linesWindowsize as a double


% --- Executes during object creation, after setting all properties.
function linesWindowsize_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linesWindowsize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function linesCompactness_Callback(hObject, eventdata, handles)
% hObject    handle to linesCompactness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linesCompactness as text
%        str2double(get(hObject,'String')) returns contents of linesCompactness as a double


% --- Executes during object creation, after setting all properties.
function linesCompactness_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linesCompactness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function linesFusion_Callback(hObject, eventdata, handles)
% hObject    handle to linesFusion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linesFusion as text
%        str2double(get(hObject,'String')) returns contents of linesFusion as a double


% --- Executes during object creation, after setting all properties.
function linesFusion_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linesFusion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function linesMinLenght_Callback(hObject, eventdata, handles)
% hObject    handle to linesMinLenght (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linesMinLenght as text
%        str2double(get(hObject,'String')) returns contents of linesMinLenght as a double


% --- Executes during object creation, after setting all properties.
function linesMinLenght_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linesMinLenght (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function linesHeura_Callback(hObject, eventdata, handles)
% hObject    handle to linesHeura (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linesHeura as text
%        str2double(get(hObject,'String')) returns contents of linesHeura as a double


% --- Executes during object creation, after setting all properties.
function linesHeura_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linesHeura (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function linesHeurr_Callback(hObject, eventdata, handles)
% hObject    handle to linesHeurr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linesHeurr as text
%        str2double(get(hObject,'String')) returns contents of linesHeurr as a double


% --- Executes during object creation, after setting all properties.
function linesHeurr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linesHeurr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function linesCyclic_Callback(hObject, eventdata, handles)
% hObject    handle to linesCyclic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linesCyclic as text
%        str2double(get(hObject,'String')) returns contents of linesCyclic as a double


% --- Executes during object creation, after setting all properties.
function linesCyclic_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linesCyclic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function linesAlpha_Callback(hObject, eventdata, handles)
% hObject    handle to linesAlpha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linesAlpha as text
%        str2double(get(hObject,'String')) returns contents of linesAlpha as a double


% --- Executes during object creation, after setting all properties.
function linesAlpha_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linesAlpha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in checkbox_globalMap.
function checkbox_globalMap_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_globalMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_globalMap


% --- Executes on button press in checkbox2.
function checkbox_localMap_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2


% --- Executes on button press in checkbox_matching.
function checkbox_matching_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_matching (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_matching

% --- Executes on button press in togglebutton_lineNav.
function togglebutton_lineNav_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton_lineNav (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton_lineNav

% --- Executes on button press in togglebutton_beaconNav.
function togglebutton_beaconNav_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton_beaconNav (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton_beaconNav

% --- Executes on button press in togglebutton_lineNav.
function togglebutton_lineBeaconNav_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton_lineNav (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton_lineNav


function edit_heading_Callback(hObject, eventdata, handles)
% hObject    handle to edit_heading (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_heading as text
%        str2double(get(hObject,'String')) returns contents of edit_heading as a double


% --- Executes during object creation, after setting all properties.
function edit_heading_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_heading (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_distance_Callback(hObject, eventdata, handles)
% hObject    handle to edit_distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_distance as text
%        str2double(get(hObject,'String')) returns contents of edit_distance as a double

% --- Executes during object creation, after setting all properties.
function edit_distance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton_goto_target.
function pushbutton_goto_target_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_goto_target (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

goToX=str2double( get(handles.edit_goToX,'String') );
goToY=str2double( get(handles.edit_goToY,'String') );

heading=str2double( get(handles.edit_heading,'String') );
dist=str2double( get(handles.edit_distance,'String') );


if goToX==0 && goToY==0,
    % use heading + distance to go to target destination
    tmpHead = floor( heading/2 ); % Floor rounds the element to nearest integer towards minus infinity
    rAngle = char( tmpHead ); 

    tmpDist = floor( dist );
    rDist = char( tmpDist );
    set(handles.edit_distance,'String',0);%% clear the box
%    set(handles.slider_target_distance,'Value',0);%% clear the slider

    error=setRobotTarget2(SERIALLINK,rAngle,rDist,handles);

    if( error )  
        set(handles.statusText,'String','Unable to send serial data. Target not sat' );    
    else
        set(handles.statusText,'String',strcat('Target heading sat to: ', num2str(tmpHead*2),'. Distance to:',num2str(tmpDist) ) );
    end
    
else
    % use cartesian coordinates to go to target
    x=goToX*10; % x in [mm]
    y=goToY*10; % y in [mm]
    
    error=goToPose2(SERIALLINK,x,y,handles);
    if error,
        set(handles.statusText,'String','Unable to send serial data. Target not sat' );
    else
        set(handles.statusText,'String',strcat('Target position sat: X=',num2str(goToX),' cm, Y=',num2str(goToY),' cm.') );
    end
end


% --- Executes on slider movement.
function slider_target_heading_Callback(hObject, eventdata, handles)
% hObject    handle to slider_target_heading (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
sliderVal = floor( get(hObject,'Value') );
set(handles.edit_heading,'String', sliderVal );


% --- Executes during object creation, after setting all properties.
function slider_target_heading_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_target_heading (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_target_distance_Callback(hObject, eventdata, handles)
% hObject    handle to slider_target_distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
sliderVal = floor( get(hObject,'Value') );
set(handles.edit_distance,'String', sliderVal );


% --- Executes during object creation, after setting all properties.
function slider_target_distance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_target_distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);

end

% --- Executes on button press in lineNavButton1.
function lineNavButton1_Callback(hObject, eventdata, handles)
% hObject    handle to lineNavButton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of lineNavButton1




% --- Executes on button press in checkbox_posErr.
function poseError=checkbox_posErr_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_posErr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_posErr
global REALPOSE;

if (get(hObject,'Value')),
    poseError=1;
    REALPOSE(1)=0;
    REALPOSE(2)=0;
    disp('position error ON')
else
    poseError=0;
    disp('position error OFF')

end


% --- Executes on button press in checkbox_irErr.
function irError=checkbox_irErr_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_irErr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_irErr
if (get(hObject,'Value')),
    irError=1;
    disp('IR sensor error ON');
else
    irError=0;
    disp('IR sensor error OFF');

end

% --- Executes on selection change in SimMapMenu.
function maze=SimMapMenu_Callback(hObject, eventdata, handles)
% hObject    handle to SimMapMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns SimMapMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from SimMapMenu


contents = get(hObject,'String');
chosenMap=contents{get(hObject,'Value')};

handles.simParams.changed='true';
switch chosenMap
    case 'Map 1'
        handles.simParams.maze=1;
    case 'Map 2'
        handles.simParams.maze=2;
    case 'Map 3'
        handles.simParams.maze=3;
    case 'Map 4'
        handles.simParams.maze=4;
    case 'Map 5'
        handles.simParams.maze=5;
    case 'Map 6'
        handles.simParams.maze=9;
    case '(Empty square)'
        handles.simParams.maze=6;
    
end
maze=handles.simParams.maze;


% --- Executes during object creation, after setting all properties.
function SimMapMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SimMapMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pauseButton.
function pauseButton_Callback(hObject, eventdata, handles)
% hObject    handle to stopButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global RUNNING;

if(RUNNING == 1)
    RUNNING = 2;
    set(handles.statusText,'String','Pause.....');
elseif (RUNNING ==2),
    RUNNING = 1;
    set(handles.statusText,'String','Started');

else
    set(handles.statusText,'String','Not started ...');
end;

function widthOfWalls_Callback(hObject, eventdata, handles)
% hObject    handle to linesWindowsize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linesWindowsize as text
%        str2double(get(hObject,'String')) returns contents of linesWindowsize as a double


% --- Executes during object creation, after setting all properties.
function widthOfWalls_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linesWindowsize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function startSimX_Callback(hObject, eventdata, handles)
% hObject    handle to linesWindowsize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linesWindowsize as text
%        str2double(get(hObject,'String')) returns contents of linesWindowsize as a double



% --- Executes during object creation, after setting all properties.
function startSimX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linesWindowsize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function startSimY_Callback(hObject, eventdata, handles)
% hObject    handle to linesWindowsize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linesWindowsize as text
%        str2double(get(hObject,'String')) returns contents of linesWindowsize as a double


% --- Executes during object creation, after setting all properties.
function startSimY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linesWindowsize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --------------------------------------------------------------------
function set_breakpoint_Callback(hObject, eventdata, handles)
% hObject    handle to set_breakpoint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function parameters_Callback(hObject, eventdata, handles)
% hObject    handle to parameters (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%pos_size = get(handles.figure1,'Position');

if strcmp(handles.lineParam.changed,'false') && strcmp(handles.beaconParam.changed,'false'),
    OutputParam2 = params2('Title','Navigation parameters');
    if length(OutputParam2)>1,
        handles.lineParam=OutputParam2{1};
        handles.beaconParam=OutputParam2{2};
    end
else
    allParam=handles.lineParam;
    allParam.beaconThresh=handles.beaconParam.beaconthreshdist;
    allParam.beaconAlpha=handles.beaconParam.alpha;
    OutputParam2 = params2('Title','Navigation parameters','current_data',allParam);
    if length(OutputParam2)>1,
        handles.lineParam=OutputParam2{1};
        handles.beaconParam=OutputParam2{2};
    end
end

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in close_pushbutton.
function close_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to close_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get the current position of the GUI from the handles structure
% to pass to the modal dialog.
pos_size = get(handles.figure1,'Position');
% Call modaldlg with the argument 'Position'.
user_response = Close_Gui('Title','Confirm Close');
switch user_response
case {'No'}
	% take no action
case 'Yes'
	% Prepare to close GUI application window
	%
	delete(handles.figure1)
end


% --------------------------------------------------------------------
function close_Callback(hObject, eventdata, handles)
% hObject    handle to close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close_pushbutton_Callback(hObject, eventdata, handles);


function lineNavButton1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to lineNavButton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function maxPerceptRadius_Callback(hObject, eventdata, handles)
% hObject    handle to maxPerceptRadius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of maxPerceptRadius as text
%        str2double(get(hObject,'String')) returns contents of
%        maxPerceptRadius as a double

% --- Executes during object creation, after setting all properties.
function maxPerceptRadius_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxPerceptRadius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function angularRes_Callback(hObject, eventdata, handles)
% hObject    handle to angularRes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of angularRes as text
%        str2double(get(hObject,'String')) returns contents of angularRes as a double


% --- Executes during object creation, after setting all properties.
function angularRes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angularRes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end






% --------------------------------------------------------------------
function edit_Callback(hObject, eventdata, handles)
% hObject    handle to edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




% --- Executes on button press in checkbox_driveScan.
function checkbox_driveScan_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_driveScan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_driveScan




% --- Executes on button press in pushbutton_fullScan.
function pushbutton_fullScan_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_fullScan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

[error,data]=fullScan(SERIALLINK,handles);

figure(66);clf;hold on;axis equal;
plot(0,0,'ko')
%plot(data(:,2), data(:,3),'bx');


IRdata=[];
NS=4;
for sens=1:1:NS,

     tmpData=data(data(:,1)==sens,:); % Missing data from one sensor!
     IRdata=vertcat(IRdata,tmpData); % IRData = the data sorted by the sensors, first sensor 1, then sensor 2 and at last sensor 3 with its data
end
%for i=1:size(IRdata,1),
    plot(IRdata(:,2), IRdata(:,3),'rx');
    
%end






% --- Executes on button press in pushbutton_setRobotPose.
function pushbutton_setRobotPose_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_setRobotPose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;
global ROBOTPOSE;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;


setX = ( str2double( get(handles.edit_setPoseX,'String') ) );
%setXtoRob = ( setX ); % x in mm
setY = ( str2double( get(handles.edit_setPoseY,'String') ) );
%setYtoRob = ( setY ); % y in mm
setTheta = ( str2double( get(handles.edit_setPoseTheta,'String') ) );
%setThetatoRob = ( setTheta*pi/180 * 1/1); % theta in rad/1000 (0-2000*pi) 

if strcmp(SERIALLINK,'simulator'),
    ROBOTPOSE=[setX/10 setY/10 setTheta]; % to simulator: x,y in [cm], theta in [deg]
    % also display result in Gui
    roboposeButton_Callback(handles.roboposeButton, eventdata, handles);

else
    pose=[setX setY setTheta ]; 
    error=setRobotPose(SERIALLINK,handles, pose);
    
    if( error )   % 
        set(handles.statusText,'String','Unable to send serial data' );
    end        
    
end



function edit_setPoseX_Callback(hObject, eventdata, handles)
% hObject    handle to edit_setPoseX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_setPoseX as text
%        str2double(get(hObject,'String')) returns contents of edit_setPoseX as a double


% --- Executes during object creation, after setting all properties.
function edit_setPoseX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_setPoseX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_setPoseY_Callback(hObject, eventdata, handles)
% hObject    handle to edit_setPoseY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_setPoseY as text
%        str2double(get(hObject,'String')) returns contents of edit_setPoseY as a double


% --- Executes during object creation, after setting all properties.
function edit_setPoseY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_setPoseY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_setPoseTheta_Callback(hObject, eventdata, handles)
% hObject    handle to edit_setPoseTheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_setPoseTheta as text
%        str2double(get(hObject,'String')) returns contents of edit_setPoseTheta as a double


% --- Executes during object creation, after setting all properties.
function edit_setPoseTheta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_setPoseTheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





function edit_goToX_Callback(hObject, eventdata, handles)
% hObject    handle to edit_goToX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_goToX as text
%        str2double(get(hObject,'String')) returns contents of edit_goToX as a double


% --- Executes during object creation, after setting all properties.
function edit_goToX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_goToX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_goToY_Callback(hObject, eventdata, handles)
% hObject    handle to edit_goToY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_goToY as text
%        str2double(get(hObject,'String')) returns contents of edit_goToY as a double


% --- Executes during object creation, after setting all properties.
function edit_goToY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_goToY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit42_Callback(hObject, eventdata, handles)
% hObject    handle to tPosText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tPosText as text
%        str2double(get(hObject,'String')) returns contents of tPosText as a double


% --- Executes during object creation, after setting all properties.
function edit42_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tPosText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on button press in pushbutton_clawOpen.
function pushbutton_clawOpen_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_clawOpen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global SERIALLINK;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

if( serialAsyncWrite(SERIALLINK,'y') )
    set(handles.statusText,'String','Unable to send serial data' );
end;    



% --- Executes on button press in pushbutton_clawStop.
function pushbutton_clawStop_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_clawStop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SERIALLINK;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

if( serialAsyncWrite(SERIALLINK,'i') )
    set(handles.statusText,'String','Unable to send serial data' );
end;    




function edit_realposeX_Callback(hObject, eventdata, handles)
% hObject    handle to edit_realposeX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_realposeX as text
%        str2double(get(hObject,'String')) returns contents of edit_realposeX as a double


% --- Executes during object creation, after setting all properties.
function edit_realposeX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_realposeX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_realposeY_Callback(hObject, eventdata, handles)
% hObject    handle to edit_realposeY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_realposeY as text
%        str2double(get(hObject,'String')) returns contents of edit_realposeY as a double


% --- Executes during object creation, after setting all properties.
function edit_realposeY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_realposeY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_realposeTheta_Callback(hObject, eventdata, handles)
% hObject    handle to edit_realposeTheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_realposeTheta as text
%        str2double(get(hObject,'String')) returns contents of edit_realposeTheta as a double


% --- Executes during object creation, after setting all properties.
function edit_realposeTheta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_realposeTheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on button press in calibrate_sensors.
function calibrate_sensors_Callback(hObject, eventdata, handles)
% hObject    handle to calibrate_sensors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global SERIALLINK;
global table;

if( SERIALLINK == 0 )
    SERIALLINK = connectTo(handles);
end;

% If the calibration is finished, draw a curve for each sensor that shows
% the analog voltage vs distance in cm
if (get(handles.finishedCalibrating,'Value') == 1)
    % Remove analog values that are 0
     for j=1:1:length(table)
         if(table(j,3) == 0)
             table(j,:) = [];
         end
     end
    table1 = [];
    table2 = [];
    table3 = [];
    table4 = [];
    % Make a table for each sensor that contains the analog voltage and the
    % distance in cm
    for i=1:1:length(table)
        if table(i,1) == 0 % Data from sensor 1
            table1 = [table1; table(i,2:1:3)];
        elseif table(i,1) == 1 % Data from sensor 2
            table2 = [table2; table(i, 2:1:3)];
        elseif table(i,1) == 2 % Data from sensor 3
            table3 = [table3; table(i, 2:1:3)];
        elseif table(i,1) == 3 % Data from sensor 4
            table4 = [table4; table(i, 2:1:3)];
        end
    end
   x = 0:1:255;
   % For sensor 1, plot curve
   if length(table1) ~= 0
     figure();
     % Fit curve to table
     [P1,S1] = polyfit(table1(:,2),table1(:,1),7);
     yfit1 = polyval(P1,x);
     % Plot the curve
     plot(x,yfit1); % x-axis = voltage and y-axis = distance in cm
     title('Sensor 1');
     xlabel('Voltage');
     ylabel('cm');
   end
   
   % For sensor 2, plot curve
   if length(table2) ~= 0
     figure(); 
     % Fit curve to table
     [P2,S2] = polyfit(table2(:,2),table2(:,1),7);
     yfit2 = polyval(P2,x);
     % Plot the curve
     plot(x,yfit2); % x-axis = voltage and y-axis = distance in cm
     title('Sensor 2');
     xlabel('Voltage');
     ylabel('cm');
   end
   
   % For sensor 3, plot curve
   if length(table3) ~= 0
     figure();
     % Fit curve to table
     [P3,S3] = polyfit(table3(:,2),table3(:,1),7);
     yfit3 = polyval(P3,x);
     % Plot the curve
     plot(x,yfit3); % x-axis = voltage and y-axis = distance in cm
     title('Sensor 3');
     xlabel('Voltage');
     ylabel('cm');
   end
   
   % For sensor 4, plot curve
   if length(table4) ~= 0
     figure();
     % Fit curve to table
     [P4,S4] = polyfit(table4(:,2),table4(:,1),7);
     yfit4 = polyval(P4,x);
     % Plot the curve
     plot(x,yfit4); % x-axis = voltage and y-axis = distance in cm
     title('Sensor 4');
     xlabel('Voltage');
     ylabel('cm');
   end
   return
end

% If a new calibrationis to be made, clear the table
if (get(handles.emptyCalibrationTable,'Value') == 1)
    table = [];
end
    
contents = get(handles.sensornr,'String');
sensornr = contents{get(handles.sensornr,'Value')};

if strcmp(sensornr,'IR sensor 1')
    sensor = 0;
elseif strcmp(sensornr,'IR sensor 2')
    sensor = 1;
elseif strcmp(sensornr,'IR sensor 3')
    sensor = 2;
elseif strcmp(sensornr,'IR sensor 4')
    sensor = 3;
end

% Get an average analog voltage value at a given distance for a given
% sensor
[error,analogData] = calibrateIRsensors(SERIALLINK,handles,sensor);

if error,
    set(handles.statusText,'String','Calibrating: Unable to send serial data.' );
else
    cm = str2double(get(handles.edit_cmToCalibrate,'String'));
    
    table = [table;sensor cm analogData];
    
    disp(table);
    
end;    


% --- Executes on selection change in sensornr.
function sensornr_Callback(hObject, eventdata, handles)
% hObject    handle to sensornr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns sensornr contents as cell array
%        contents{get(hObject,'Value')} returns selected item from sensornr


% --- Executes during object creation, after setting all properties.
function sensornr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sensornr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_cmToCalibrate_Callback(hObject, eventdata, handles)
% hObject    handle to edit_cmToCalibrate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_cmToCalibrate as text
%        str2double(get(hObject,'String')) returns contents of edit_cmToCalibrate as a double


% --- Executes during object creation, after setting all properties.
function edit_cmToCalibrate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_cmToCalibrate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in emptyCalibrationTable.
function emptyCalibrationTable_Callback(hObject, eventdata, handles)
% hObject    handle to emptyCalibrationTable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of emptyCalibrationTable




% --- Executes on button press in finishedCalibrating.
function finishedCalibrating_Callback(hObject, eventdata, handles)
% hObject    handle to finishedCalibrating (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of finishedCalibrating



function angleText_Callback(hObject, eventdata, handles)
% hObject    handle to angleText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of angleText as text
%        str2double(get(hObject,'String')) returns contents of angleText as a double


% --- Executes on button press in checkbox_goHome.
function checkbox_goHome_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_goHome (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_goHome


% --- Executes on button press in Recharge_checkbox.
function Recharge_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to Recharge_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Recharge_checkbox



function goHomeText_Callback(hObject, eventdata, handles)
% hObject    handle to goHomeText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of goHomeText as text
%        str2double(get(hObject,'String')) returns contents of goHomeText as a double


% --- Executes during object creation, after setting all properties.
function goHomeText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to goHomeText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function Untitled_11_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over COMValg.
function COMValg_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to COMValg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function camIPText_Callback(hObject, eventdata, handles)
% hObject    handle to camIPText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of camIPText as text
%        str2double(get(hObject,'String')) returns contents of camIPText as a double


% --- Executes during object creation, after setting all properties.
function camIPText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to camIPText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
