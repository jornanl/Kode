function MotorRotateAbs(whatmotor,abs_angle,power, varargin)
% Rotates a motor to an absolute angle
%  
% Syntax
%   MotorRotateAbs(port,abs_angle,power) 
%
%   MotorRotateAbs(port,abs_angle,power, handle) 
%
% Description
%   MotorRotateAbs(port,abs_angle,power) rotates the motor connected to the given
%   port. The value port can be addressed by the symbolic constants MOTOR_A, MOTOR_B and
%   MOTOR_C analog to the labeling on the NXT Brick. The angle determines the absolute angle to
%   rotate. power represents the motor power (1...100).
%
%   The last optional argument can be a valid NXT handle. If none is
%   specified, the default handle will be used (call COM_SetDefaultNXT to
%   set one).
%
% Note:
%   *This function should only be used by the advanced user.*
%
% Example
%   MotorRotateAbs(MOTOR_B, 550, 30);
% 
% See also: SetMemoryCount, GetMemoryCount, NXT_SetOutputState, MOTOR_A, MOTOR_B, MOTOR_C
%
% Signature
%   Author: Robert Schwann, Bernd Neumann (see AUTHORS)
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

% check if handle is given; if not use default one
if nargin > 3
    handle = varargin{1};
else
    handle = COM_GetDefaultNXT;
end%if

if power<=0
    error('MATLAB:RWTHMindstormsNXT:Motor:invalidPower', ...
         ['Use power > 0. To select negative rotational direction, ' ...
          'absolute target angle must be smaller than current angle.'])
end

out = GetMotorSettings(whatmotor, handle);
% physical angle the motor actually has to go
angle_to_go = abs_angle - out.TachoCount;

% relative angle (for motor command)
rel_angle = abs_angle - GetMemoryCount(whatmotor, handle);
SetMemoryCount(whatmotor, abs_angle, handle);

if sign(rel_angle)*sign(angle_to_go)>0 || angle_to_go==0
    if rel_angle~=0
        NXT_SetOutputState(whatmotor,power*sign(rel_angle),1,1,...
            'SPEED',0,'RUNNING',abs(rel_angle),'dontreply', handle);
    end
else
     NXT_SetOutputState(whatmotor,-power*sign(angle_to_go),1,1,...
         'SPEED',0,'RUNNING',abs(rel_angle)+1,'dontreply', handle);
     NXT_SetOutputState(whatmotor,power*sign(angle_to_go),1,1,...
         'SPEED',0,'RUNNING',1,'dontreply', handle);
end
