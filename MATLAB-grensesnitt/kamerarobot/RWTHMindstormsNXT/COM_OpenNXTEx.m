function handle = COM_OpenNXTEx(ConnectionMode, UseThisNXTMAC, varargin)
% Opens a Bluetooth or USB connection to an NXT device and returns a handle for future use
%
% Syntax
%   handle = COM_OpenNXTEx('USB', UseThisNXTMAC)
%
%   handle = COM_OpenNXTEx('Bluetooth', UseThisNXTMAC, inifilename, 'check')
%
%   handle = COM_OpenNXTEx('Any', UseThisNXTMAC, inifilename, 'check')
%
% Description
%   This function establishes a connection to an NXT brick and returns the
%   handle structure that has to be used with NXT-functions (you can call
%   COM_SetDefaultNXT(handle) afterwards for easier use).
%
%   For a more convenient way to open an NXT handle with less parameters, the
%   function COM_OpenNXT is provided.
%
%   Different types of connection modes are supported. In all modes, you can
%   set UseThisNXTMAC to a string with the NXT's MAC address (serial number).
%   A connection will then only be estabslished to a matching NXT brick. This can be
%   useful for programs with multiple NXT devices. Set it to an empty string
%   '' to use any NXT available (usually the first one found).
%   
%
%   handle = COM_OpenNXTEx('USB', UseThisNXTMAC)
%   This will try to open a connection via USB. Device drivers (Fantom on
%   Windows, libusb on Linux) have to be installed.
%
%
%   handle = COM_OpenNXTEx('Bluetooth', UseThisNXTMAC, inifilename, 'check')
%   Uses Bluetooth as communication method. A valid inifile containing
%   parameters like the COM-Port has to be specified in inifilename. The
%   optional paramter 'check' can be omitted (it will nake sure that the
%   new Bluetooth connection is working bi-directional). Leave it out if
%   your hardware does only support sending data (depends on the Bluetooth
%   hardware, drivers and stack used). In this case you will not be able to
%   receive any data from the brick, but can still send commands (like e.g.
%   motor control).
%   To create an inifile with Bluetooth settings, the function
%   COM_MakeBTConfigFile is available.
%    
%   Note that as of right now, the parameter UseThisNXTMAC will be
%   ignored for Bluetooth connections until implemented in a future version.
%
%
%   handle = COM_OpenNXTEx('Any', UseThisNXTMAC, inifilename, 'check')
%   This syntax combines the two parameter settings from above.
%   inifilename has to be given, the optional 'check' can be omitted.
%   The function will try to locate an NXT device on the USB bus first. If
%   this fails for some reason (no USB connection to the NXT available, no
%   device drivers installed, or NXT device is busy), the function will
%   silently try to establish a connection via Bluetooth.
%
%   The advantage is that this version works with both Bluetooth and USB
%   connections _without changing_ any code. Plug or unplug the USB cable
%   to switch between connection types...
%   
%
% Limitations of COM_CloseNXT
%   If you call COM_CloseNXT('all') after a clear all command has been
%   issued, the function will not be able to close all remaining open USB
%   handles, since they have been cleared out of memory. This is a problem
%   on Linux systems. You will not be able to use the NXT device without
%   rebooting it.
%   Solution: Either use only clear in your programs, or you use the
%   COM_CloseNXT('all') statement before clear all.
%   The best way however is to track your handles carefully and close them
%   manually (COM_CloseNXT(handle)) before exiting whenever possible!%
%
%
% Example
%   myNXT = COM_OpenNXTEx('Any', '001612345678', 'bluetooth.ini', 'check');
%   % This will connect to an NXT device with the MAC/serial number 001612345678,
%   % first trying via USB. If this fails (no drivers installed or no matching USB
%   % device found), a connection via Bluetooth will be established, using
%   % the paramters found in the given config file.
% 
%
% See also: COM_OpenNXT, COM_CloseNXT, COM_MakeBTConfigFile, COM_SetDefaultNXT
%
% Signature
%   Author: Linus Atorf (see AUTHORS)
%   Date: 2008/06/11
%   Copyright: 2007-2008, RWTH Aachen University
%
%
% ***********************************************************************************************
% *  This file is part of the RWTH - Mindstorms NXT Toolbox.                                    *
% *                                                                                             *
% *  The RWTH - Mindstorms NXT Toolbox is free software: you can redistribute it and/or modify  *
% *  it under the terms of the GNU General Public License as published by the Free Software     *
% *  Foundation, either version 3 of the License, or (at your option) any later version.        *
% *                                                                                             *
% *  The RWTH - Mindstorms NXT Toolbox is distributed in the hope that it will be useful,       *
% *  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS  *
% *  FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.             *
% *                                                                                             *
% *  You should have received a copy of the GNU General Public License along with the           *
% *  RWTH - Mindstorms NXT Toolbox. If not, see <http://www.gnu.org/licenses/>.                 *
% ***********************************************************************************************
 

%% *** A few guidelines ***
% This function supports both Linux and Windows with both USB and Bluetooth
% connections. So we have 4 cases to distinguish.
% For Bluetooth, the old function BT_OpenHandle was adapted and used. It works for
% Linux and Windows (it is split internally where necessary).
% The function USB_OpenHandle is just a wrapper (the counterpart to
% BT_OpenHandle), the actual work is done by USB_OpenHandle_Windows and
% USB_OpenHandleLinux (which are very different!).
%
% The principle is always the same: We use createHandleStruct() to create an
% "empty" default handle, that is already initialized to some empty strings and
% NaNs. We then fill in the required information wherever appropriate.
%
% A handle is not considered valid until we call .Connected(true). So the top
% level wrappers can track back if creation was successful or not.
%
% The USB functions have an additional parameter "SuppressErrors". If true, they
% will "ignore" errors, so later on we can still establish a Bluetooth connection.
% This way, if USB drivers/libraries are not present, the simple COM_OpenNXT
% (which is essentially COM_OpenNXTEx('Any', ...) won't fail with USB.




%% Actual code

    % needed to track handles for later COM_CloseNXT('all')
    global NXTHANDLE_Array
    

%% Check input parameters...

    % correct connection mode string?
    if isempty(ConnectionMode) || (~strcmpi(ConnectionMode, 'any') && ~strcmpi(ConnectionMode, 'USB') && ~strcmpi(ConnectionMode, 'Bluetooth'))
        error('MATLAB:RWTHMindstormsNXT:invalidStringParameter', 'Input argument ConnectionMode has to be either ''USB'', ''Bluetooth'', or ''any''!')
    end%if

    % nxt name or mac must be string
    if ~ischar(UseThisNXTMAC)
        error('MATLAB:RWTHMindstormsNXT:invalidStringParameter', 'Input argument UseThisNXTMAC has to be a string containing the NXT''s name or MAC address, or an empty string '''' for no restrictions!')
    end%if

    if strcmpi(ConnectionMode, 'USB')
        % for usb, only 2 params needed
        if (nargin > 2)
            error('MATLAB:RWTHMindstormsNXT:tooManyInputArguments', 'Too many input arguments: For connection mode ''USB'', there is only 1 more input argument needed!')
        end%if
    else % Note: Nodes BT and any!!! So we require an inifile for any-mode as well!
        % check varargins
        if (nargin <= 2)
            error('MATLAB:RWTHMindstormsNXT:notEnoughInputArguments', 'Not enough input arguments: For connection modes ''any'' and ''Bluetooth'', argument inifilename is required!')
        else
            if ~ischar(varargin{1})
                error('MATLAB:RWTHMindstormsNXT:invalidStringParameter', 'Optional input argument Inifilename has to be a string!')
            end%if
        end%if
        if nargin > 3 
            if ~(ischar(varargin{2}) && strcmpi(varargin{2}, 'check'))
                error('MATLAB:RWTHMindstormsNXT:invalidVararginParameter','2nd optional input parameter after inifilename has to be ''check'' or nothing.');
            end%if
        end%if

    end%if

    
%% Do a quickcheck for toolbox integrity -- do the private helper functions work?
    % This is probably one of the first points where every program using
    % our toolbox comes past. Also, it's not performance critical. We use
    % this time to quickly check for wordbytes2dec. If it doesn't work, we
    % know the toolbox is corrupt -- or the OptimizeToolboxPerformance must
    % have failed. If we don't catch it here, later a try-catch-block will
    % catch it and erroneously claim it was a bluetooth error...
    try
        % Test below is copied from OptimizeToolboxPerformance
        a = [123; 132];
        test1 = wordbytes2dec(a, 2, 'signed');
        test2 = dec2wordbytes(test1, 2, 'signed');

        % compare vectors
        res = (test2 == a);
        % if not equal...
        if nnz(res) ~= 2
            % raise error for catch...
            error('dummy error')
        end%if    
    catch
        error('MATLAB:RWTHMindstormsNXT:dec2wordbytesNotWorking','Your toolbox seems corrupt. Something is wrong with the private helper functions wordbytes2dec etc. Please run the command OptimizeToolboxPerformance or reinstall this toolbox correctly.');
    end%try

    
%% Get empty handle, fill with infos

    h = createHandleStruct(); 
    h.CreationTime = clock;
    % The next steps are necessary, we need some sort of timestamp. We take the
    % current date minus 1 year, so we get a point defenitely in the past! This
    % way, COM_Send and COM_Collect will start sending / receiving at once.
    timestamp = clock();
    timestamp(1) = timestamp(1) - 1;
    h.LastSendTime(timestamp);    
    h.LastReceiveTime(timestamp); 
    
    if ispc
        h.OSName    = 'Windows';
        h.OSValue   = 1;
    else % no support for Mac yet...
        h.OSName    = 'Linux';
        h.OSValue   = 2;
    end%if

    % now set the MAC parameter, the functions later down will evaluate and
    % overwrite it!
	h.NXTMAC = UseThisNXTMAC;
    
    
%% Now proceed with OS / mode specific handle creation etc.

    if strcmpi(ConnectionMode, 'any')
        % for any, try USB first...
        h = USB_OpenHandle(h, true);
        if ~h.Connected()
            % we should have all needed parameters, so lets go
            h = BT_OpenHandle(h, varargin{1}, varargin{2:end});
        end%if
        
    elseif strcmpi(ConnectionMode, 'USB')
        h = USB_OpenHandle(h, false);

    elseif strcmpi(ConnectionMode, 'Bluetooth')
        % pass inifile params and optional check this easy way, works as long
        % as we dont introduce new varargins for other reasons...
        h = BT_OpenHandle(h, varargin{1}, varargin{2:end});

    end%if
    

%% Actual work is done, acquire a slot in global array

    % although we should've a valid handle at this point (or already errored out),
    % only add it to global list if valid!
    if h.Connected()
        h.Index = length(NXTHANDLE_Array) + 1;
        NXTHANDLE_Array{h.Index} = h;
    end%if
    

%% Return handle
% TODO Actually, we wanted to return only the most important static part
% of a full handle, to make the struct more compact and don't confuse the
% user. The problem however is, that if someone retrieves a handle, doesn't
% set it as default handle and then calls an NXT_ function with the handle
% as varargin, this handle is "too short" to be used, as it doesn't contain
% the functions. What do we do?
% Workaround: We make a "sub-field" of the handle, another struct, called
% "functions". A handle would then be used like this:
% h.functions.BytesSent(), etc.
%
% 
%     handle.OSName                = h.OSName;
%     handle.OSValue               = h.OSValue;
%     handle.ConnectionTypeName    = h.ConnectionTypeName;
%     handle.ConnectionTypeValue   = h.ConnectionTypeValue;
% 
%     handle.Handle                = h.Handle;
% 
%     handle.IniFilename           = h.IniFilename;
%     handle.ComPort               = h.ComPort;
%     handle.BaudRate            	 = h.BaudRate;
%     handle.DataBits            	 = h.DataBits;
%     handle.Timeout             	 = h.Timeout;
% 
%     handle.SendSendPause       	 = h.SendSendPause;
%     handle.SendReceivePause      = h.SendReceivePause;
% 
%     handle.NXTMAC                = h.NXTMAC;
%     handle.CreationTime       	 = h.CreationTime;  
%     handle.Index                 = h.Index;
    
    handle = h;

    
end%function


%% --- FUNCTION BT_OpenHandle -- the original :-)
function hOut = BT_OpenHandle(hIn, inifilename, varargin)

%% copy struct info, initialize

    hOut = hIn;
    hOut.Connected(false);

    
%% Read parameter from ini file
    inisection = 'Bluetooth';

    ComPort             = readFromIniFile(           inisection, 'SerialPort',       inifilename);
    if ispc % WINDOWS serial port version....
        BaudRate            = str2double(readFromIniFile(inisection, 'BaudRate',         inifilename));
        DataBits            = str2double(readFromIniFile(inisection, 'DataBits',         inifilename));
        Timeout             = str2double(readFromIniFile(inisection, 'Timeout',          inifilename));
        if isempty(Timeout) || isnan(Timeout)
            Timeout = 2;
            warning('MATLAB:RWTHMindstormsNXT:Bluetooth:missingInifileParameters','Timeout in bluetooth settings inifile not set. Using default value (2 seconds).')
        end%if
    else % LINUX file handle version
        % on linux, there is no baudrate etc to set, so we ignore these ini
        % settings, as they are not necessary, and we don't want to display
        % error messages for ini settings that are not needed in linux.
        %
        % we set the variables to default values anyway to avoid
        % potential errors or is-empty-failures, just in case some functions
        % checks this...
        BaudRate = 9600;
        DataBits = 8;
        Timeout  = 2;
    end%if
    BT_SendSendPause    = str2double(readFromIniFile(inisection, 'SendSendPause',    inifilename));
    BT_SendReceivePause = str2double(readFromIniFile(inisection, 'SendReceivePause', inifilename));


%% Set default sending/receiving delay times if they are not set manually 
    if isempty(BT_SendSendPause) || isnan(BT_SendSendPause) || isempty(BT_SendReceivePause) || isnan(BT_SendReceivePause)
        warning('MATLAB:RWTHMindstormsNXT:Bluetooth:missingInifileParameters','SendSendPause or SendReceivePause in bluetooth settings inifile not set. Using default values.')
        BT_SendSendPause    =  5; %ms
        BT_SendReceivePause =  30; %ms
    end%if



%% Open bluetooth connection
    textOut(sprintf('Opening Bluetooth connection on port %s... ', ComPort));

    try
        if ispc
            handle = serial(ComPort,'BaudRate', BaudRate, 'DataBits', DataBits, 'Timeout', Timeout);
            fopen(handle);
        else
            % is unix
            handle = fopen(ComPort, 'r+');
        end
    catch
        textOut(sprintf('failed.\n'));
        error('MATLAB:RWTHMindstormsNXT:Bluetooth:couldNotOpenConnection','Could not open bluetooth connection using port %s, BaudRate=%d, DataBits=%d.\nTry using COM_CloseNXT(''all'') first.', ComPort, BaudRate, DataBits);
    end%try

    textOut(sprintf('done.\n'));


    
%% Fill handle structure with information

    

    hOut.ConnectionTypeName     = 'Bluetooth';
    hOut.ConnectionTypeValue    = 2;
    hOut.Handle                 = handle;
    hOut.IniFilename            = inifilename;
    hOut.ComPort            	= ComPort;
    hOut.BaudRate               = BaudRate;
    hOut.DataBits               = DataBits;
    hOut.Timeout                = Timeout;
    hOut.SendSendPause          = BT_SendSendPause;
    hOut.SendReceivePause       = BT_SendReceivePause;
    hOut.Connected(true);

    
%% If required, send test (keep alive) packet
    if nargin > 2
        if ischar(varargin{1}) && strcmpi(varargin{1}, 'check')
          
            %NOTE this command below is very dangerous, as it is included
            %in a try-catch environment. If ANYTHING goes wrong with the NXT
            %command, we have NO idea what happened, and just say "hey,
            %bluetooth is not working", even if m-files are missing etc...
            try
                status = NXT_SendKeepAlive('reply', hOut);
            catch
                errordlg('*** Bluetooth connection not working ***. Please make sure that the NXT is turned on and your bluetooth adapter is connected.', 'No NXT found');
                error('MATLAB:RWTHMindstormsNXT:Bluetooth:connectionNotWorking', ...
                     ['*** Bluetooth connection not working ***.' char(10) ...
                      'Please make sure that the NXT is turned on and your bluetooth adapter is connected.']);
            end%if

            %NOTE should we really display the same message here as above?
            % Apparently, transfer succeeded, but only the status reply is
            % wrong. This is interesting: Malformed packet etc? But probably
            % never happens wnyway...
            if status ~= 0
                errordlg('*** Bluetooth connection not working ***. Please make sure that the NXT is turned on and your bluetooth adapter is connected.', 'No NXT found');
                error('MATLAB:RWTHMindstormsNXT:Bluetooth:connectionNotWorking', ...
                     ['*** Bluetooth connection not working ***.' char(10) ...
                      'Please make sure that the NXT is turned on and your bluetooth adapter is connected.']);
            end%if

        else

            error('MATLAB:RWTHMindstormsNXT:invalidVararginParameter','Only possible varargin is ''check''');
        end%if
    end%if

%% Get NXT's MAC, and see if it matches constraints
    if ~isempty(hOut.NXTMAC) 

        %FIXME get NXT's name & mac in here, and compare...
        
    end%if

end%function


%% --- FUNCTION USB_OpenHandle
function hOut = USB_OpenHandle(hIn, SuppressErrors)
    
    textOut(sprintf('Opening USB connection...\n'));
    
    if hIn.OSValue == 1
        hOut = USB_OpenHandle_Windows(hIn, SuppressErrors);
    elseif hIn.OSValue == 2
        hOut = USB_OpenHandle_Linux(hIn, SuppressErrors);
    end%if
    
end%function


%% --- FUNCTION USB_OpenHandle_Windows
function hOut = USB_OpenHandle_Windows(hIn, SuppressErrors)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Temporary USB construction site for the RWTH - Mindstorms NXT Toolbox
%           http://www.mindstorms.rwth-aachen.de
%
% Based on the code by Vital van Reeven
%           http://forums.nxtasy.org/index.php?showtopic=2018
%           http://www.vitalvanreeven.nl/page156/fantomNXT.zip
%
% Linus Atorf, 29.3.2008
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


% Basic structure of this function: Load fantom library, create NXT iterator (if
% it fails, no NXT on USB present), loop through all NXT on this iterator
% (important, otherwise it won't work), take the NXT we want, open it, and done.


    %TODO we dont want to suppress all errors! but we do want to avoid errors
    %caused by missing libraries and errors of the kind "no nxt was found"


    % copy struct info
    hOut = hIn;
    hOut.Connected(false); % should be false already, why am I so paranoid and set it anyway?
    
    hOut.ConnectionTypeValue    = 1;
    hOut.ConnectionTypeName     = 'USB';
    
    
%% Load Fantom library (only if necessary)
    if ~libisloaded('fantom')
        textOut(sprintf('  - Loading library "fantom"... '));
        try
            % use "our" wrapper file...
            loadlibrary('fantom', @fantom_proto);
            textOut(sprintf('done.\n'));
        catch 
            textOut(sprintf('failed.\n'));
            if ~SuppressErrors
                error('MATLAB:RWTHMindstormsNXT:USB:Windows:couldNotLoadLibraryFantom', 'The "fantom" library could not be loaded. Make sure the official Mindstorms NXT driver is installed correctly!')
            else
                return
            end%if
        end%try
    else
        textOut(sprintf('  - Library "fantom" already loaded.\n'));
    end%if
        
    
    
%% Create NXT iterator
    
    textOut(sprintf('  - Creating NXT iterator... '))

    % last 3 params are:                        searchbluetooth, searchtimeout, status;
    [hNXTIterator status] = calllib('fantom', 'nFANTOM100_createNXTIterator', 0, 0, 0);
    displayUSBWinStatus(status)
    
    % if there is no NXT connected, we don't get an iterator...
    if status 
        if ~SuppressErrors
            errordlg('No NXT found on USB bus! Make sure the NXT is turned on and device drivers ("Fantom") are properly installed. Rebooting your NXT might help!', 'No NXT found')
            error('MATLAB:RWTHMindstormsNXT:USB:Windows:noNXTfound', 'No NXT found on USB bus! Make sure the NXT is turned on and device drivers ("Fantom") are properly installed. Rebooting your NXT might help!')
        else
            % don't do anything but exit
            textOut(sprintf('Could not open USB connection.\n'));
            return
        end%if
    end%if


    
%% Loop through all NXTs
    gotNXT = false;
    
    textOut(sprintf('  - Enumerating NXT devices on USB bus\n'));
    status = 0;
    while(~status) % successful if == 0

        % the first time we're in this loop, the iterator already points to the
        % first NXT, so we can retrieve it's device id straight away. At this
        % point we already have an NXT, otherwise we wouldn't have gotten an
        % iterator in the first place!
        
        textOut(sprintf('    . getting current NXT''s device id... '));
        [deviceID stat] = calllib('fantom', 'nFANTOM100_iNXTIterator_getName', hNXTIterator, blanks(255), 0); %deviceid, status; ok
        displayUSBWinStatus(stat)
        if ~stat
            textOut(sprintf(['      deviceID = ' deviceID '\n']));
        end%if

        % replace :: with spaces, get 4th item
        MAC = sscanf(strrep(deviceID, '::', ' '), '%*s %*s %*s %s %*s'); 
        
        % only get NXT if MAC matches and if we don't already have one
        if ~gotNXT && (isempty(hIn.NXTMAC) || strcmpi(hIn.NXTMAC, MAC))
            textOut(sprintf('    . opening current NXT... '));
            [hNXT stat]= calllib('fantom', 'nFANTOM100_iNXTIterator_getNXT', hNXTIterator, 0); %status; ok            
            displayUSBWinStatus(stat)
            if stat
                if ~SuppressErrors
                    errordlg('Could not open desired device! Is it already in use? Rebooting your NXT might help!', 'Could not open NXT')
                    error('MATLAB:RWTHMindstormsNXT:USB:Windows:couldNotOpenNXT', 'Could not open desired device! Is it already in use? Rebooting your NXT might help!')
                else
                    textOut(sprintf('Could not open USB connection./n'));
                    return
                end%if
            end%if
            
            % remember for later!
            hOut.NXTMAC = MAC;
            hOut.Handle = hNXT;
            hOut.Connected(true);

            gotNXT = true; % set flag to mark our great success

        end%if
            
        % Note that we HAVE to advance to the last device of the iterator,
        % until the call fails! If we don't do so, the next time we create
        % a fresh iterator after properly destroying the current, we would
        % get an invalid one. Really annoying, took me ages to find out,
        % but since we now know, everything is fine.
        
        textOut(sprintf('    . advancing to next NXT... '));
        status = calllib('fantom', 'nFANTOM100_iNXTIterator_advance', hNXTIterator, 0); 
        if status
            % here we've got our error, just as expected (and desired, see comment above)
            % the while-loop won't iterate now, so we don't need a return here
            textOut(sprintf('done. (this was the last one)\n'));
        else
           textOut(sprintf('done.\n'));
        end%if
        
    end%while - proceed to next NXT

    textOut(sprintf('  - Destroying NXT iterator... '));
    status = calllib('fantom', 'nFANTOM100_destroyNXTIterator', hNXTIterator, 0);
    displayUSBWinStatus(status)
    
    % did we have success?
    if ~gotNXT
        if ~SuppressErrors
            errordlg('No NXT found that matches your parameters (MAC address or NXT''s name)! Please check the constraints. (Rebooting your NXT might help!)', 'No matching NXT found')
            error('MATLAB:RWTHMindstormsNXT:USB:Windows:noMatchingNXTfound', 'No NXT found that matches your parameters (MAC address or NXT''s name)! Please check the constraints. (Rebooting your NXT might help!)')
        else
            textOut(sprintf('Could not open USB connection (no matching NXT found)./n'));
            return
        end%if
    end%if
      

end%function


%% --- FUNCTION displayUSBWinStatus(status)
function displayUSBWinStatus(status)
% little convenient helper to save some lines of code with textOut
    if status
        textOut(sprintf('failed.\n'));
        textOut(sprintf(['VISA error ' num2str(status) ': ' getVISAErrorString(status) '\n']))
    else
        textOut(sprintf('done.\n'));
    end%if
end%function


%% --- FUNCTION displayLibusbStatus(status)
% little convenient helper to save some lines of code with textOut
function displayLibusbStatus(status)
% little convenient helper to save some lines of code with textOut
    if isnumeric(status) && (status < 0)
        textOut(sprintf('failed.\n'));
        textOut(sprintf(['Libusb error ' num2str(status) ': ' getLibusbErrorString(status) '\n']))
    else
        textOut(sprintf('done.\n'));
    end%if
end%function


%% --- FUNCTION USB_OpenHandle_Linux
function hOut = USB_OpenHandle_Linux(hIn, SuppressErrors)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Temporary USB construction site for the RWTH - Mindstorms NXT Toolbox
%           http://www.mindstorms.rwth-aachen.de
%
% Linus Atorf, 29.4.2008
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Basic structure of this function:
% Load library libusb, initialize it, get root bus object, loop through all
% busses, for each bus loop through all devices, see if vendor and product id
% match with lego NXT, try to open device, read serial no (which is = MAC), if it
% matches, keep it open. then do basic USB device initialization (set config,
% claim interface, reset device). dev should be ready then.

%NOTE There is still one big problem with Multi-NXT: If the first one that is
%connected is also the first that was opened (and is now in use), the usb_open_dev
%might fail when looping through all devs (since the dev is busy). Careful testing
%is necessary to see wether opening an "NXT in use" fails, or if it is possible
%and only fails when claiming its interface (I think that's the case actually, so
%this would not be an issue after all).
% new remark: doesn't seem to be a problem...


%% initialize etc...
    ID_VENDOR_LEGO = 1684; % hex2dec('0694');
    ID_PRODUCT_NXT = 2;    % hex2dec('0002');

    LIBUSB_Configuration = 1;
    LIBUSB_Interface = 0;
    
    
    hOut = hIn;
    hOut.Connected(false);

    hOut.ConnectionTypeValue    = 1;
    hOut.ConnectionTypeName     = 'USB';
    
%% Load libusb library (only if necessary)
    if ~libisloaded('libusb')
        textOut(sprintf('  - Loading library "libusb"... '));
        try
            % use "our" wrapper file...
            loadlibrary('libusb', @libusb_proto)
            textOut(sprintf('done.\n'));
        catch 
            textOut(sprintf('failed.\n'));
            if ~SuppressErrors
                error('MATLAB:RWTHMindstormsNXT:USB:Linux:couldNotLoadLibraryLibusb', 'The "libusb" library could not be loaded (file "libusb" or "libusb.so"). Make sure it is installed and paths are set correctly!')
            else
                return
            end%if
        end%try
    else
        textOut(sprintf('  - Library "libusb" already loaded.\n'));
    end%if


    textOut(sprintf('  - Initializing and browsing USB busses\n'));

%% Init libusb
    textOut(sprintf('    . initializing libusb.\n'))
    calllib('libusb', 'usb_init');
    
    % > - insert usb_set_debug(255) in pyusb.c right behind usb_init()
    % > - recompile and reinstall the module
    % > - hook up DebugView from www.sysinternals.com
    % > - rerun your application
    % so if we want debug mode, this needs to be uncommented!
    %calllib('libusb', 'usb_set_debug', 255);


    % these functions return the number of changes made to the busses and
    % devices since last call, don't neet it...
    textOut(sprintf('    . finding busses... '))
    ret = calllib('libusb', 'usb_find_busses');
    displayLibusbStatus(ret);
    
    textOut(sprintf('    . finding devices... '))
    ret = calllib('libusb', 'usb_find_devices');
    displayLibusbStatus(ret);

%% Get main root bus
    
    textOut(sprintf('    . getting root bus object... '))
    p = calllib('libusb', 'usb_get_busses');
    bus = libstruct('usb_bus', p);
    displayLibusbStatus(p);
    

    textOut(sprintf('  - Enumerating busses and devices\n'));

%% Cycle through all busses and devices: enumerate...
    foundNXT = false;
    while ~foundNXT % for all busses
        
        % we want to save some debug-lines (already got too many), so we don't
        % care what USB devices the user has. remove comment for advanced debug
        % mode
        %textOut(sprintf('    . current bus: %s\n', strtrim(char(bus.dirname))))

        dev = libstruct('usb_device', bus.devices);

        % -------------------------------------
        while true % for all devices! sorry for endless-loop, but we break later down

            % if no devs at all
            if isempty(dev)
                break
            end%if


            % we want to save some debug-lines (already got too many), so we don't
            % care what USB devices the user has. remove comment for advanced debug
            % mode
            %textOut(sprintf('    . current device: %s\n', strtrim(char(dev.filename))))


            % try a little string reading....

            % THIS DOESN'T WORK UNDER LINUX MOST OF THE TIME
            % APPARENTLY, WE CANNOT OPEN ALL PRESENT DEVICES
            % Since we don't want to try to open a device now, we keep this
            % working code (at least for public devices on Linux and for
            % alle devices on Windows) commented, maybe someone else can
            % use it to improve this function later on...
            % It's debug-info only anyway

    %        % open device to get a handle
    %         DevHandle = calllib('libusb', 'usb_open', dev);
    %  
    %         % now the string stuff
    %         buffer = blanks(255);
    %         % we don't need a real buffer or pointer, matlab seems to do this
    %         % for us, so we pass that buffer variable, without really needing
    %         % it. it seems like matlab "knows" how strings get written by
    %         % reference and returns the new value from the function. if you
    %         % compare the matlab-returnvalues of usb_get_string_simple using
    %         % libfunctionsview libusb, you'll find that they don't match whats
    %         % written inside usb.h. very nice and handy, thank you matlab :-)
    %         %pBuffer = libpointer('cstring', buffer); not needed, see above
    %         
    %         % now the actual call:
    %         [bytesRead newHandleOrWhat ManufacturerName] = calllib('libusb', 'usb_get_string_simple', ...
    %                    DevHandle, dev.descriptor.iManufacturer,  buffer, length(buffer));
    %         % again:
    %         [bytesRead newHandleOrWhat ProductName] = calllib('libusb', 'usb_get_string_simple', ...
    %                    DevHandle, dev.descriptor.iProduct,  buffer, length(buffer));
    %                
    %         disp(sprintf('        Manufacturer: %s', ManufacturerName))
    %         disp(sprintf('        Product: %s', ProductName))
    %         
    %         % close device again
    %         ret = calllib('libusb', 'usb_close', DevHandle);
    %         clear DevHandle %better doing it now than forgetting it later


            % note at this point that we only try to open a device if it's one
            % from LEGO!
    
            % check if it's LEGO and NXT
            if (dev.descriptor.idVendor == ID_VENDOR_LEGO) && (dev.descriptor.idProduct == ID_PRODUCT_NXT)
                textOut(sprintf('    . found NXT device\n'));

                textOut(sprintf('    . opening device... '));
                DevHandle = calllib('libusb', 'usb_open', dev);
                displayLibusbStatus(DevHandle);

                textOut(sprintf('    . reading serial number... '));
                % c-syntax from documentation so we know whats going on:
                % int usb_get_string_simple(usb_dev_handle *dev, int index, char *buf, size_t buflen);
                buffer = blanks(255);
                [bytesRead newHandleOrWhat SerialNo] = calllib('libusb', 'usb_get_string_simple', DevHandle, dev.descriptor.iSerialNumber,  buffer, length(buffer));
                displayLibusbStatus(bytesRead);
                                
                SerialNo = strtrim(SerialNo); % just to be safe
                textOut(sprintf('    . MAC = %s ', SerialNo))
                
                % is it the right one?
                if isempty(hIn.NXTMAC) || strcmpi(hIn.NXTMAC, SerialNo)
                    foundNXT = true;
                    textOut(sprintf('(MAC matches, this is our desired NXT)\n'));
                    break % search came to an end
                else
                    foundNXT = false;
                    textOut(sprintf('(no match, not using this NXT)\n'));
                    % don't forget to close the opened NXT that didnt match...
                    textOut(sprintf('    . closing device... '));
                    status = calllib('libusb', 'usb_close', DevHandle);
                    displayLibusbStatus(status);
                end%if
                
            end%if


            % we are at the end of our "pointer-queue"
            if isempty(dev.next)
                break
            else
                % get next device, not that easy, but it works now \o/
                ptr = dev.next;
                ptr.setdatatype('usb_device');
                dev = ptr.Value; 
                clear ptr % to be sure, don't want to risk memory leaks
            end%if
            
        end%while (for all devices)
        % -------------------------------------

        
        % no need to scan other busses if already found
        if foundNXT
            break
        end%if

        % jump to next bus - same procedure as for devices, see above
        if isempty(bus.next)
            break
        else
            ptr = bus.next;
            ptr.setdatatype('usb_bus');
            bus = ptr.Value;
            clear ptr
        end%if
        
    end%while (for all busses)


%% check if NXT present
    if ~foundNXT
        if ~SuppressErrors
            errordlg('No NXT found on USB bus! Make sure the NXT is turned on and access rights in /dev/ are properly set. Rebooting your NXT might help!', 'No NXT found')
            error('MATLAB:RWTHMindstormsNXT:USB:Linux:noNXTfound', 'No NXT found on USB bus! Make sure the NXT is turned on and access rights in /dev/ are properly set. Rebooting your NXT might help!')
        else
            % well, no NXT, exit silently
            return
        end%if
    end%if


%% NXT found, open connection

    % DevHandle is now the NXT we want
    
    % the following commands are standard procedure for USB devices, similar usage
    % can be found in the open source packages Python_NXT and LEGO::NXT (Perl)

    % flag to remember
    ErrorWhileOpening = false;
    
    % if the following fails with error -16, the NXT is probably already in
    % use (opened in another handle!)
    textOut(sprintf('    . setting active configuration... '));
    % somehow this doesn't work, so we use the hardcoded configuration 1!
    %calllib('libusb', 'usb_set_configuration', DevHandle, dev.config.bConfigurationValue);
    ret = calllib('libusb', 'usb_set_configuration', DevHandle, LIBUSB_Configuration);
    displayLibusbStatus(ret);
    if (ret < 0)
        ErrorWhileOpening = true;
    end%if

    textOut(sprintf('    . claiming interface... '));
    % again, interface is hardcoded (compare implementations in python and
    % perl, they do it the same way)
    %calllib('libusb', 'usb_claim_interface', DevHandle, dev.config.interface.altsetting.bInterfaceNumber);
    ret = calllib('libusb', 'usb_claim_interface', DevHandle, LIBUSB_Interface);
    displayLibusbStatus(ret);
    if (ret < 0)
        ErrorWhileOpening = true;
    end%if
    
    
    % we don't need an alternative interface, whatever that is.
    % but from debugging experience (also with Windows), you never know
    % when you might need this, so we keep it in here!
    
    % disp('    . set altinterface')
    % %  %[int32, usb_dev_handlePtr] usb_set_altinterface(usb_dev_handlePtr, int32)
    % ret = calllib('libusb', 'usb_set_altinterface', DevHandle, dev.config.interface.altsetting.bInterfaceNumber);
    % disp(sprintf('usb_strerror: %s', calllib('libusb', 'usb_strerror')))

    % is this necessary? but found it in perl and python versions...
    % the main point: never touch a running system
    textOut(sprintf('    . resetting device... '));
    ret = calllib('libusb', 'usb_reset', DevHandle);
    displayLibusbStatus(ret);
    if (ret < 0)
        ErrorWhileOpening = true;
    end%if
    
    % now it's time to decide:
    if ErrorWhileOpening || isnumeric(DevHandle)
        if ~SuppressErrors
            errordlg('Something went wrong while opening the NXT device via USB (is it already open in another handle?). Please try to reboot the NXT or call COM_CloseNXT(''all'')!', 'Could not open NXT')
            error('MATLAB:RWTHMindstormsNXT:USB:Linux:couldNotOpenNXT', 'Something went wrong while opening the NXT device via USB (is it already open in another handle?). Please try to reboot the NXT or call COM_CloseNXT(''all'')!')
        else
            % again, exit silently when no success
            return
        end%if
    end%if
    
    % finally, we're good to go!
    hOut.Handle = DevHandle;
    hOut.NXTMAC = SerialNo;

    % and, important:
    hOut.Connected(true);
    
    
%% clean up

    % is this needed? or will matlab destroy this private vars anyway after
    % finishing this function? just to be sure with pointers...
    clear p v bus dev newHandleOrWhat DevHandle


end%function

