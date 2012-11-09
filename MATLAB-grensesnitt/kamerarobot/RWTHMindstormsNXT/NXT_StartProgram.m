function NXT_StartProgram(filename, varargin)
% Starts the given program on the NXT Brick
%  
% Syntax
%   NXT_StartProgram(filename) 
%
%   NXT_StartProgram(filename, handle)
%
% Description
%   NXT_StartProgram(filename) starts the embedded NXT Brick program determined by the string
%   filename. The maximum length is limited to 15 characters. The file
%   extension '.rxe' is added automatically if it was omitted.
%
%   NXT_SetBrickName(name, handle) uses the given NXT connection handle. This should be
%   a struct containing a serial handle on a PC system and a file handle on a Linux system. 
%
%   If no NXT handle is specified the default one (COM_GetDefaultNXT) is used.
%
% For more details see the official LEGO Mindstorms communication protocol.
%
% Examples
%   NXT_StartProgram('ResetCounter');
%
%   handle = COM_OpenNXT('bluetooth.ini','check');
%   NXT_StartProgram('Demo.rxe', handle);
%
% See also: NXT_StopProgram
%
% Signature
%   Author: Alexander Behrens, Linus Atorf (see AUTHORS)
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
% check if bluetooth handle is given. if not use default one
if nargin > 1
        handle = varargin{1};
else
    handle = COM_GetDefaultNXT;
end%if

% check if name is a string
if ~ischar(filename)
	error('MATLAB:RWTHMindstormsNXT:NXTProgramNameNotAString', 'Program name must be a string (char-array)');
end%if

% check if name length is less than 15
maxnamelen = 15+1+3;
if length(filename) < 1 || length(filename) > maxnamelen
    error('MATLAB:RWTHMindstormsNXT:invalidNXTProgramNameLength', 'Program name must have a length between 1 (e.g. "a.rxe") and %d chars', maxnamelen);
end%if


% check if filename extension is present
if length(filename) > 3
    if ~strcmpi(filename(end-3:end), '.rxe')
        filename = [filename '.rxe'];
    end
else
    filename = [filename '.rxe'];
end


%%  Write payload
% attach null terminator
payload = [real(filename) 0]';


%% Build bluetooth command
[type cmd] = name2commandbytes('STARTPROGRAM');


%% Pack bluetooth packet
packet = COM_CreatePacket(type, cmd, 'dontreply', payload); 
textOut(sprintf('+ Starting NXT program: %s \n', filename));


%% Send bluetooth packet
COM_SendPacket(packet, handle);

end%function
