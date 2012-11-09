function out = NXT_GetOutputState(port, varargin)
% Requests and retrieves an output motor state reading
%  
% Syntax
%   data = NXT_GetOutputState(port) 
%
%   data = NXT_GetOutputState(port, handle) 
%
% Description
%   data = NXT_GetOutputState(port) requests and retrieves an output motor state reading of the
%   given motor port. The value port can be addressed by the symbolic constants MOTOR_A,
%   MOTOR_B and MOTOR_C analog to the labeling on the NXT Brick.  The return value data is a
%   struct variable. It contains several motor settings and information. 
%
%   data = NXT_GetOutputState(port, handle) uses the given Bluetooth connection handle. This should be a
%   serial handle on a PC system and a file handle on a Linux system.
%
%   If no Bluetooth handle is specified the default one (COM_GetDefaultNXT) is used.
%
% Output:
%     data.Port               % current port number (0..3)
%
%     data.Power              % current motor power
%
%     data.Mode               % motor mode byte
%
%     data.ModeIsMOTORON      % flag: "motor is on"
%
%     data.ModeIsBRAKE        % flag: "motor uses the advanced brake mode (PVM)"
%
%     data.ModeIsREGULATED    % flag: "motor uses a regulation"
%
%     data.RegModeByte        % motor regulation byte
%
%     data.RegModeName        % name of regulation mode
%
%     data.TurnRatio          % turn ratio value
%
%     data.RunStateByte       % motor run state byte
%
%     data.RegStateName       % name of run state
%
%     data.TachoLimit         % tacho / angle limit
%
%     data.TachoCount         % current absolute tacho count
%
%     data.BlockTachoCount    % current relative tacho count
%
%     data.RotationCount      % current second relative tacho count
%
% For more details see the official LEGO Mindstorms communication protocol.
%
% Examples
%   data = NXT_GetOutputState(MOTOR_B);
%
%   handle = COM_OpenNXT('bluetooth.ini','check');
%   data = NXT_GetOutputState(MOTOR_A, handle);
%
% See also: GetMotorSettings, SendMotorSettings, NXT_SetOutputState, MOTOR_A, MOTOR_B, MOTOR_C
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
if port < 0 || port > 2    
    %FIXME this warning (...:Motor:ignoringInvalidPort) should be changed
    % to the ...:Motor:invalidPort error everywhere. It does not make sense
    % to pass an invalid port number (notice however the special port-number 
    % 255 for motor-ports when SETTING things (not reading))!
    warning('MATLAB:RWTHMindstormsNXT:Motor:ignoringInvalidPort', 'NXT OutputPort should be between 0 and 2, trying to use invalid port %d anyway...', port);
end%if


%% Use wrapper functions
NXT_RequestOutputState(port, handle);
out = NXT_CollectOutputState(handle);

end % end function



%% ### Function: Request Output State Packet ###
function NXT_RequestOutputState(port, varargin)
% Sends the "GetOutputState" packet: Requests the state of a specified motor,
% i.e. rotation count etc.
%
% usage: NXT_RequestOutputState(port, varargin)
%
%       port     :  port number
%       varargin :  bluetooth handle (optional)
%
% Example: NXT_RequestOutputState(1)
%

%% Parameter check
% check if bluetooth handle is given; if not use default one
if nargin > 1
        handle = varargin{1};
else
    handle = COM_GetDefaultNXT;
end%if

% check if port number is valid
if port < 0 || port > 2 
    warning('MATLAB:RWTHMindstormsNXT:Motor:ignoringInvalidPort', 'NXT OutputPort should be between 0 and 2, trying to use invalid port %d anyway...', port);
end%if


%% Build bluetooth command
[type cmd] = name2commandbytes('GETOUTPUTSTATE');


%% Pack bluetooth packet
packet = COM_CreatePacket(type, cmd, 'reply', port);
textOut(sprintf('+ Requesting output state from port %d...\n', port));


%% Send bluetooth packet
COM_SendPacket(packet, handle);

end % end function



%% ### Function: Collect Output State Packet ###
function out = NXT_CollectOutputState(varargin)
% Retrieves the previously requested output (motor) state packet form serial port
%
% Returns
%         out.Port
%         out.Power
%         out.Mode
%         out.ModeIsMOTORON
%         out.ModeIsBRAKE
%         out.ModeIsREGULATED
%         out.RegModeByte
%         out.RegModeName
%         out.TurnRatio
%         out.RunStateByte
%         out.RunStateName
%         out.TachoLimit
%         out.TachoCount
%         out.BlockTachoCount
%         out.RotationCount


%% Parameter check
% check if bluetooth handle is given; if not use default one
if nargin > 0
        handle = varargin{1};
else
    handle = COM_GetDefaultNXT;
end%if


%% Get reference
[dontcare ExpectedCmd] = name2commandbytes('GETOUTPUTSTATE');

%% Collect bluetooth packet
[type cmd status content] = COM_CollectPacket(handle);

%% Check if packet is the right one
if cmd ~= ExpectedCmd || status ~= 0
    warning('MATLAB:RWTHMindstormsNXT:Bluetooth:discardingUnexpectedPacket', 'Received packed not expected. Discarding and trying to continue...');
    %TODO In order to really continue execution (which is probably not
    % possible anyway), we should NOT return just 0 but instead a valid 
    % structure as below, but with zeros / empty strings inserted at the
    % right places. Otherwise the next line in a program will fail anyway,
    % when trying to access out.something that is not set!
    % We have to consider if it is not too dangerous on the other hand to
    % knowingly return zeros that seem like a valid answer, while it is
    % not. This can ONLY be fatal when the warning
    % "...:Bluetooth:discardingUnexpectedPacket" is disabled. The advantage
    % we would get from this doubtful fake however is what was the original
    % idea: To enable the program to continue. Maybe logical errors will
    % follow, but at least we don't break and raise an error when we don't
    % have to. So if some sort of demo-program for example does not work
    % under certain circumstances, disabling (or ignoring) the above
    % warning would be a last-second rescue-option. Question is, again: Is
    % it worh it? Does it occur? Is it possible that a program that is THIS
    % program at this point, will continue to work correctly anyway?
    % Depending on this answer, we should either leave the warnings (but
    % than fake the struct below properly), OR on the contrary raise hard
    % errors (then replace all the Bluetooth:discardingUnexpectedPacket
    % warnings).
    % My personal (Linus') opinion: Leave the warnings. You never know
    % what is going to happen, and having a warning is as good as having an
    % error, with the big advantage that you can ignore it! Still, at some
    % point, the code below should then be adjusted...
    
    out = 0;
    return;
end%if


%% Interpret packet content
out.Port            = content(1);
out.Power           = wordbytes2dec(content(2), 1, 'signed'); % fixed
out.Mode            = content(3);
[out.ModeIsMOTORON out.ModeIsBRAKE out.ModeIsREGULATED] = byte2outputmode(content(3));
out.RegModeByte     = content(4);
out.RegModeName     = byte2regmode(content(4));
out.TurnRatio       = wordbytes2dec(content(5), 1, 'signed');
out.RunStateByte    = content(6);
out.RunStateName    = byte2runstate(content(6));
out.TachoLimit      = wordbytes2dec(content(7:10), 4); %unsigned
out.TachoCount      = wordbytes2dec(content(11:14), 4, 'signed'); %signed
out.BlockTachoCount = wordbytes2dec(content(15:18), 4, 'signed'); %signed
out.RotationCount   = wordbytes2dec(content(19:22), 4, 'signed'); %signed

end % end function
