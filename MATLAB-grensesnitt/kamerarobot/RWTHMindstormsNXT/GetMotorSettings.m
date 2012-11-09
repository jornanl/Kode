function out = GetMotorSettings(whatmotor, varargin)
% Returns the current motor data / settings (e.g. position, speed, etc.) from the specified motor
%  
% Syntax
%   data = GetMotorSettings(port) 
%
%   data = GetMotorSettings(port, handle) 
%
% Description
%   data = GetMotorSettings(port) returns the current motor data / settings of the given motor
%   port. The value port can be addressed by the symbolic constants MOTOR_A , MOTOR_B and
%   MOTOR_C analog to the labeling on the NXT Brick. The return value data is a struct variable.
%   It contains several motor settings and information.
%
%   The last optional argument can be a valid NXT handle. If none is
%   specified, the default handle will be used (call COM_SetDefaultNXT to
%   set one).
%
% Output:
%     data.IsRunning         % boolean, true if the motor "does something"
%
%     data.Power             % current power
%
%     data.AngleLimit        % current set angle limit, 0 means none set
%
%     data.TurnRatio         % current turn ratio
%
%     data.SpeedRegulation   % boolean, speed regulated or not?
%
%     data.SyncToMotor       % the motor this one is synced to. -1 means not synced
%
%     data.TachoCount        % internal, non-resettable rotation-counter (in degrees)
%
%     data.Angle             % current motor position, resettable using, ResetMotorAngle
%
%     data.MotorBrake        % boolean, is electronic braking enabled?
%
% Example
%   SetMotor(MOTOR_C);
%   	SetPower(67);
%   	SetAngleLimit(240);
%   	SpeedRegulation('on');
%   SendMotorSettings();
%
%   WaitForMotor(MOTOR_C);
%
%   data = GetMotorSettings(MOTOR_C);
%
% See also: SendMotorSettings, ResetMotorAngle
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


% check if handle is given; if not use default one
if nargin > 1
    h = varargin{1};
else
    h = COM_GetDefaultNXT;
end%if

if whatmotor < 0 || whatmotor > 2
    error('MATLAB:RWTHMindstormsNXT:Motor:invalidPort', 'Input argument for motor must be 0, 1, or 2')
end%if

% get motorstate
NXTMOTOR_State = h.NXTMOTOR_getState();


data = NXT_GetOutputState(whatmotor, h);


% the double() is necessary, because we don't really want to give them
% int32, sometimes messes around with calculations
out.MotorBrake      = data.ModeIsBRAKE;
out.Power           = double(data.Power);
out.AngleLimit      = double(data.TachoLimit);
out.TurnRatio       = double(data.TurnRatio);
out.TachoCount      = double(data.TachoCount);
out.Angle           = double(data.BlockTachoCount);


h.NXTMOTOR_resetRegulationState(whatmotor);

if strcmpi(data.RegModeName, 'SPEED')
    NXTMOTOR_State(whatmotor + 1).SyncedToSpeed = true;
    out.SpeedRegulation = true;
    out.SyncToMotor = -1;
elseif strcmpi(data.RegModeName, 'IDLE')
    out.SpeedRegulation = false;
    out.SyncToMotor = -1;
else % SYNC then... :-)
    
    %FIXME remove the next statement? not critical though
    % it is dangerous, because this functions should only READ information
    % but not write it. and the next command could overwrite the internal
    % user settings for the next sending command. maybe the user has
    % already use SpeedRegulation('on') but not sent the command yet. In
    % this case, this next instruction will delete the setting.
    % BUT, it is of course not recommended to use anything by SetXXX
    % commands right before the SendMotorSettings...
    
    % shouldnt be necessary but whatever
    NXTMOTOR_State(whatmotor + 1).SyncedToSpeed = false;
   
    % hmm...
    out.SyncToMotor = NXTMOTOR_State(whatmotor + 1).SyncedToMotor;
    if out.SyncToMotor == -1
        warning('MATLAB:RWTHMindstormsNXT:Motor:internalSyncStateNotUpToDate', ...
               ['Although SYNC bit is set, global motor state var doesn''t "know" to what motor. ' ...
                'This might be a toolbox-bug, please look into this...']);
    end%if
    
end%if

if strcmpi(data.RunStateName, 'IDLE')
    out.IsRunning = false; % false if IDLE, true otherwise
else
    out.IsRunning = true;
end%if

NXTMOTOR_State(whatmotor + 1).TachoCount = data.TachoCount;

% TODO the whole concept of storing this freshly retrieved information back
% to the handle might not be that useful anymore... as soon as the data
% arrives, it's basically out of date. So why would we want to keep it?

% save motor state back to handle
h.NXTMOTOR_setState(NXTMOTOR_State);

end%function

