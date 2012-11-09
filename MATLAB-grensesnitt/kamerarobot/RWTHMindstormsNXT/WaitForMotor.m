function WaitForMotor(whatmotor, varargin)
% Pauses execution until specific motor is not running anymore. 
%  
% Syntax
%   WaitForMotor(port) 
%
%   WaitForMotor(port, time) 
%
%   WaitForMotor(port, time, handle) 
%
% Description
%   WaitForMotor(port) pauses other MALTAB executions until the motor connected to the given
%   port is not running anymore. The value port can be addressed by the symbolic constants
%   MOTOR_A, MOTOR_B and MOTOR_C analog to the labeling on the NXT Brick. 
% 
%   WaitForMotor(port, time) specifies a maximum timeout (waiting time in seconds) until
%   execution will be continued anyway.
%
%   The last optional argument can be a valid NXT handle. If none is
%   specified, the default handle will be used (call COM_SetDefaultNXT to
%   set one).
%
%
% Note:
%   Be careful not using a timeout, wrong motor settings or blocked motors can lead to an
%   infinite loop and cause the application to hang... 
%
%   This function uses busy waiting, i.e. is constantly polling the
%   specified motor. Hence Bluetooth timeouts can lead to delays in this
%   function, meaning that execution of your program might continue
%   slightly later than expected (depending on Bluetooth connection
%   quality). Be aware of this (for example by setting an angle limit using
%   SetAngleLimit) to take precautions.
%
% Example
%   SetMotor(MOTOR_B);
%   	SetPower(-55);
%   	SetAngleLimit(1200);
%   SendMotorSettings();
%
%   WaitForMotor(MOTOR_A);
%
%   StopMotor(MOTOR_B, 'brake');
%
% See also: SendMotorSettings, StopMotor, MOTOR_A, MOTOR_B, MOTOR_C
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

UseTimeout = false;
Timeout = 0;
if nargin > 1
    if ischar(varargin{1}), Timeout = str2double(varargin{1}); else Timeout = varargin{1}; end
    UseTimeout = true;
end%if

% check if handle is given; if not use default one
if nargin > 2
    handle = varargin{2};
else
    handle = COM_GetDefaultNXT;
end%if


% toggle global screen out because we have a loop later on
global DisableScreenOut;
oldstate = DisableScreenOut;
DisableScreenOut = true; %#ok<NASGU>

if UseTimeout
    tictic(99)
end%if

data = GetMotorSettings(whatmotor, handle);
while(data.IsRunning)
    
    if UseTimeout && (toctoc(99) > Timeout)
        break
    end%if
    
    data = GetMotorSettings(whatmotor, handle);
    
    pause(0.002) % 2 ms pause. we do this for matlab so that we can break with CTRL + C if needed
    
end%while

% and reset to what it was before
DisableScreenOut = oldstate;

end%function

