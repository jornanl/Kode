function ResetMotorAngle(port, varargin)
% Resets the relative angle counter for the given motor
%  
% Syntax
%   ResetMotorAngle(port) 
%
%   ResetMotorAngle(port, handle) 
%
% Description
%   ResetMotorAngle(port) resets the relative angle counter for the motor connected to the given
%   port. The value port can be addressed by the symbolic constants MOTOR_A, MOTOR_B and
%   MOTOR_C analog to the labeling on the NXT Brick. The relative counter can be read by the
%   function GetMotorSettings.
% 
%   ResetMotorAngle(port, handle) uses the given Bluetooth connection handle. This should be a
%   serial handle on a PC system and a file handle on a Linux system. 
%
%   If no NXT handle is specified the default one (COM_GetDefaultNXT) is used.
%
% Examples
%   ResetMotorAngle(MOTOR_A);
%
%   handle = COM_OpenNXT('bluetooth.ini','check');
%   ResetMotorAngle(MOTOR_C, handle);
%
% See also: NXT_ResetMotorPosition, GetMotorSettings, MOTOR_A, MOTOR_B, MOTOR_C, COM_GetDefaultNXT
%
% Signature
%   Author: Linus Atorf, Alexander Behrens (see AUTHORS)
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
% check if NXT handle is given; if not use default one
if nargin > 1
    handle = varargin{1};
else
    handle = COM_GetDefaultNXT;
end%if

% as we can see, just a little wrapper that maps the following NXT function and
% parameter combination to a more obvious name
NXT_ResetMotorPosition(port, true, handle);
    
end%function
