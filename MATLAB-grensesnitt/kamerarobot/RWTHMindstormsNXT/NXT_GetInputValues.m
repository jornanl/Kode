function out = NXT_GetInputValues(port, varargin)
% Processes a complete sensor reading, i.e. requests input values and collects the answer.
%  
% Syntax
%   data = NXT_GetInputValues(port) 
%
%   data = NXT_GetInputValues(port, handle) 
%
% Description
%   data = NXT_GetInputValues(port) processes a complete sensor reading, i.e. requests input
%   values and collects the answer of the given sensor port. The value port can be addressed by
%   the symbolic constants SENSOR_1, SENSOR_2, SENSOR_3 and SENSOR_4 analog to the labeling
%   on the NXT Brick. The return value data is a struct variable. It contains several sensor
%   settings and information. 
%
%   data = NXT_GetInputValues(port, handle) uses the given Bluetooth connection handle. This should be a
%   serial handle on a PC system and a file handle on a Linux system.
%
%   If no Bluetooth handle is specified the default one (COM_GetDefaultNXT) is used.
%
% Output:
%     data.Port               % current port number (0..3)
%
%     data.Valid              % validation flag
%
%     data.Calibrated         % boolean, true if calibration file found and used
%
%     data.TypeByte           % sensor type
%
%     data.TypeName           % sensor mode
%
%     data.ModeByte           % mode
%
%     data.ModeName           % mode name
%
%     data.RawADVal           % raw A/D value
%
%     data.NormalizedADVal    % normalized A/D value
%
%     data.ScaledVal          % scaled value
%
%     data.CalibratedVal      % calibrated value
%
% Note:
%   Data are only valid if .Valid is ~= 0. This should usually be the
%   case, but a short while after setting a new sensor mode using
%   NXT_SetInputMode, you have to carefully check .Valid on your own!
%   Experience shows that only .ScaledVal is influenced by this, apparently
%   .NormalizedADVal seems valid all the time, but closer examination is
%   needed...
%
%   For more details see the official LEGO Mindstorms communication protocol.
%
% Examples
%   data = NXT_GetInputValues(SENSOR_3);
%
%   handle = COM_OpenNXT('bluetooth.ini','check');
%   data = NXT_GetInputValues(SENSOR_1, handle);
%
% See also: NXT_SetInputMode, GetLight, GetSwitch, GetSound, GetUltrasonic, SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4
%
% Signature
%   Author: Linus Atorf (see AUTHORS)
%   Date: 2007/10/15
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

%% Parameter check
% check if bluetooth handle is given; if not use default one
if nargin > 1
        handle = varargin{1};
else
    handle = COM_GetDefaultNXT;
end%if

% check if port number is valid
if port < 0 || port > 3 
    %FIXME this warning (...:Sensor:ignoringInvalidPort) should be changed
    % to the ...:Sensor:invalidPort error everywhere. It does not make sense
    % to pass an invalid port number (notice however the special port-number 
    % 255 for motor-ports)!
    warning('MATLAB:RWTHMindstormsNXT:Sensor:ignoringInvalidPort', 'NXT InputPort should be between 0 and 3, trying to use invalid port %d anyway...', port);
end%if


%% Use wrapper functions
NXT_RequestInputValues(port, handle);
out = NXT_CollectInputValues(handle);

end % end function



%% ### Function: Request Input Values Packet ###
function NXT_RequestInputValues(port, varargin)
% Sends the packet "GetInputValues" that requests a sensor reading
%
% usage: NXT_RequestInputValues(port, varargin)
%
%       port     :  port number
%       varargin :  bluetooth handle (optional)
%
% Example: NXT_RequestInputValues(0)
%

%% Parameter check
% check if bluetooth handle is given; if not use default one
if nargin > 1
        handle = varargin{1};
else
    handle = COM_GetDefaultNXT;
end%if


% check if port number is valid
if port < 0 || port > 3 
    warning('MATLAB:RWTHMindstormsNXT:Sensor:ignoringInvalidPort', 'NXT InputPort should be between 0 and 3, trying to use invalid port %d anyway...', port);
end%if


%% Build bluetooth command
[type cmd] = name2commandbytes('GETINPUTVALUES');


%% Pack bluetooth packet
packet = COM_CreatePacket(type, cmd, 'reply', port);
textOut(sprintf('+ Requesting sensor info from port %d...\n', port));

%% Send bluetooth packet
COM_SendPacket(packet, handle);

end % end function



%% ### Function: Collect Input Values Packet ###
function out = NXT_CollectInputValues(varargin)
% Retrieves the previously requested InputValues (a sensor reading) from the serial port buffer.
%
% Returns:
%                 out.Port
%                 out.Valid
%                 out.Calibrated
%                 out.TypeByte
%                 out.TypeName
%                 out.ModeByte
%                 out.ModeName
%                 out.RawADVal
%                 out.NormalizedADVal
%                 out.ScaledVal
%                 out.CalibratedVal
%
% Note: Data are only valid if .Valid is ~= 0. This should usually be the
% case, but a short while after setting a new sensor mode using
% NXT_SetInputMode, you have to carefully check .Valid on your own!
% Experience shows that only .ScaledVal is influenced by this, apparently
% .NormalizedADVal seems valid all the time, but closer examination is
% needed...

%% Parameter check
% check if bluetooth handle is given; if not use default one
if nargin > 0
        handle = varargin{1};
else
    handle = COM_GetDefaultNXT;
end%if


%% Get reference
[dontcare ExpectedCmd] = name2commandbytes('GETINPUTVALUES');

%% Collect bluetooth packet
[type cmd status content] = COM_CollectPacket(handle);

%% Check if packet is the right one
if cmd ~= ExpectedCmd || status ~= 0
    warning('MATLAB:RWTHMindstormsNXT:Bluetooth:discardingUnexpectedPacket', 'Received packed not expected. Discarding and trying to continue...');
    %TODO same as in NXT_GetOutputState, consider "faking" a real
    % NXT_GetInputValues return-struct when ignoring the warning above,
    % otherwise the idea of continuing program execution does not make
    % sense!
    return;
end%if


%% Interpret packet content
out.Port            = content(1);
out.Valid           = content(2);
out.Calibrated      = content(3);
out.TypeByte        = content(4);
out.TypeName        = byte2sensortype(content(4));
out.ModeByte        = content(5);
out.ModeName        = byte2sensormode(content(5));
out.RawADVal        = wordbytes2dec(content(6:7), 2); %unsigned
out.NormalizedADVal = wordbytes2dec(content(8:9), 2); % unsigned!
out.ScaledVal       = wordbytes2dec(content(10:11), 2, 'signed'); % signed!
out.CalibratedVal   = wordbytes2dec(content(12:13), 2, 'signed'); % currently unused


%TODO: we do not check the valid bit at this point, which means that the
% returned data CAN be invalid a short time after calling NXT_SetInputMode.
% you got to check the .Valid property on your own!

%TODO enable this data-not-valid warning?
% if ~out.Valid
%     warning('MATLAB:RWTHMindstormsNXT:Sensor:dataNotValidYet', 'Collected sensor data invalid, please look into this if they don''t get valid after a while!')
% end%if

end % end function

