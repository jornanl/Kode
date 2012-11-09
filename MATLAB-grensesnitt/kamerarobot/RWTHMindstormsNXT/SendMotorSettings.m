function SendMotorSettings(f_port, f_power, f_angle, f_speed, f_sync, f_ratio, f_ramp)
% Sends previously specified settings to current active motor.
%  
% Syntax
%   SendMotorSettings() 
%
%   SendMotorSettings(port, power, angle, speedRegulation, syncedToMotor, turnRatio, rampMode) 
%
% Description
%   SendMotorSettings() sends the previously specified settings of the current motor. The motor
%   settings are set by the functions SetMotor, SetPower, SetAngleLimit, SpeedRegulation,
%   SyncToMotor, SetTurnRatio and SetRampMode. 
%
%   SendMotorSettings(port, power, angle, speedRegulation, syncedToMotor, turnRatio, rampMode)
%   sends the given settings like motor port (MOTOR_A, MOTOR_B or MOTOR_C), the power
%   (-100...100, the angle limit, speedRegulation ('on', 'off'), syncedToMotor
%   (MOTOR_A, MOTOR_B, MOTOR_C), turnRatio (-100...100) and rampMode ('off', 'up',
%   'down'). 
%
% Note:
%   All settings like power, angle limit and so on, will only take effect once you send them to the
%   motor using THIS function. Note that if you have synced two motors together, this function
%   internally sends two packets to both the motors. This is required, but you can work as if this
%   was just one command.
%
% Example
%   SetMotor(MOTOR_B);
%   	SyncToMotor(MOTOR_C);
%   	SetPower(76);
%   	SetAngleLimit(4*360);
%   	SetTurnRatio(20);
%   SendMotorSettings();
%
% See also: GetMotorSettings, SetMotor, SetPower, SpeedRegulation, SyncToMotor, SetTurnRatio, SetRampMode, NXT_SetOutputState, MOTOR_A, MOTOR_B, MOTOR_C
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

%%
%
% The whole idea of the SetMotor, SetPower etc. concept is that we can
% reveal more complex functions later on. For beginners, a simple
% instruction like this is enough:
%   SetMotor(MOTOR_A);
%      SetPower(100);
%   SendMotorSettings;
%
% The active motor will be remembered for each current active NXT, as well
% as all motor settings. The handle manages all information about an NXT,
% including its motors.
%
% Compare these easy commands to MATLABs way of using figures:
% In the beginning, a command sequence like this is just fine:
%   figure
%   plot(X, '.r')
%
% But later on, you can also use advanced settings:
%   figure
%   plot(X, '.r')
%   colormap hot
%   axis square
%   material shiny
%   shading interp
%   lighting phong
%
% Just like you would use more advanced motor commands:
%   SetMotor(MOTOR_A)
%       SpeedRegulation on
%       SetPower 80
%       SetAngleLimit 500
%       SetRampMode up
%       SyncToMotor off
%       SetTurnRatio 0 
%   SendMotorSettings
%



%% check given arguments
if (nargin ~= 0) && (nargin ~= 7)
    help SendMotorSettings;
    error('MATLAB:RWTHMindstormsNXT:invalidParameterCount', ...
         ['Either none or 7 function arguments are needed. ' ...
          'Type "help SendMotorSettings" or see documentation!']);
end

%% get handle & motorstate
h = COM_GetDefaultNXT();
NXTMOTOR_State = h.NXTMOTOR_getState();



%% set parameter for function call with arguments
if nargin > 0
    
    if ~strcmpi(f_sync, 'off') && ~strcmpi(f_speed, 'off')
        error('MATLAB:RWTHMindstormsNXT:Motor:simultaneousSyncAndSpeedRegulationError', 'You can only use motor synchronization OR speed regulation at a time, but not both settings together.')
    end%if
    
    
    % don't influence with MotorSet, if different was set
    try
        oldmotor = h.NXTMOTOR_getCurrentMotor();
    catch
        oldmotor = NaN;
    end%try
    
    %TODO In this short version of SendMotorSettings, we use SyncToMotor
    % and SpeedRegulation consecutively. Even if the parameters are correct
    % (meaning that motor sync and speed reg are mutually exclusive), we
    % produce a warning at this point, if e.g. speedreg was activated
    % before and should now be deactivated while using sync. In this
    % combination of long and short versions of SendMotorSettings, this
    % warning produced by SpeedRegulation() is wrong. We have to avoid it
    % at this point by adding a SpeedRegulation('off'); before
    % SyncToMotor in the following command sequence, or by adding a little
    % if statement, depending on performance and complications...
    
    SetMotor(f_port);
        SyncToMotor(f_sync);
        SetPower(f_power);
        SetAngleLimit(f_angle);
        SpeedRegulation(f_speed);
        SetTurnRatio(f_ratio);
        SetRampMode(f_ramp);
        
    % for restore see later down
end


%% regular command sequence
whatmotor = h.NXTMOTOR_getCurrentMotor();

%function status = NXT_SetOutputState(OutputPort, Power, IsMotorOn, IsBrake, RegModeName, TurnRatio, RunStateName, TachoLimit, ReplyMode, varargin)


%% only use turnratio if motor is synced, ignore if all motors set...
if whatmotor ~= 255
    TurnRatio = 0; % only has affect if regmode is SYNC
    syncedmotor = -1;
    if NXTMOTOR_State(whatmotor + 1).SyncedToSpeed
        RegModeName = 'SPEED';
    elseif NXTMOTOR_State(whatmotor + 1).SyncedToMotor ~= -1
        RegModeName = 'SYNC';
        TurnRatio = NXTMOTOR_State(whatmotor + 1).TurnRatio;
        syncedmotor = NXTMOTOR_State(whatmotor + 1).SyncedToMotor; 
    else
        RegModeName = 'IDLE';
    end%if
end%if

%% if synced....
if whatmotor ~= 255
    if syncedmotor ~= -1
        % send other packet as well
        NXT_SetOutputState(syncedmotor, ... % port
            NXTMOTOR_State(syncedmotor + 1).Power, ... % power
            true, ... % motoron
            ~NXTMOTOR_State(syncedmotor + 1).BrakeDisabled, ... % brake
            RegModeName, ... % reg mode
            NXTMOTOR_State(syncedmotor + 1).TurnRatio, ... % turn ratio
            NXTMOTOR_State(syncedmotor + 1).RunStateName, ... % runstate 
            NXTMOTOR_State(syncedmotor + 1).AngleLimit, ...
            'dontreply'); 

        if (NXTMOTOR_State(syncedmotor+1).Power ~= 0)
            % set memory counter
            SetMemoryCount(syncedmotor, GetMemoryCount(syncedmotor) + ...
                           sign(NXTMOTOR_State(syncedmotor+1).Power) * NXTMOTOR_State(syncedmotor+1).AngleLimit); 
        end
    end%if
end%if

% if all motor ports are set, we remember this now
realport = whatmotor;
% but whatmotor cannot be > 2, or else the global var will be out of
% index...
if whatmotor == 255
    % we set it to the first motor in this case, if all motors are set,
    % their settings should be the same anyway...
    whatmotor = 0; % looks like bad style to override it here, i know...
    
    % need to set these now as they weren't set above
    if NXTMOTOR_State(whatmotor + 1).SyncedToSpeed
        RegModeName = 'SPEED';
    else
        % SYNC doesn't make sense for all motors!
        RegModeName = 'IDLE';
    end%if
    % also no sense makes a TurnRatio:
    TurnRatio = 0;
    
end%if


%% send "regular" packet...
NXT_SetOutputState(realport, ... % port
    NXTMOTOR_State(whatmotor + 1).Power, ... % power
    true, ... % motoron
    ~NXTMOTOR_State(whatmotor + 1).BrakeDisabled, ... % brake
    RegModeName, ... % reg mode
    TurnRatio, ... % turn ratio
    NXTMOTOR_State(whatmotor + 1).RunStateName, ... % runstate 
    NXTMOTOR_State(whatmotor + 1).AngleLimit, ...
    'dontreply'); 

    if (NXTMOTOR_State(whatmotor+1).Power ~= 0)
        % set memory counter
        SetMemoryCount(whatmotor, GetMemoryCount(whatmotor) + ...
                       sign(NXTMOTOR_State(whatmotor+1).Power) * NXTMOTOR_State(whatmotor+1).AngleLimit); 
    end
    

    
%% Restore previous motor number setting
try % try because oldmotor might not've been set
    % applies only to short-notation with nargin == 7...
    if ~isnan(oldmotor)
        h.NXTMOTOR_setCurrentMotor(oldmotor);
    end
catch
    % nothing, its nothing to restore here...
end%


%% save motor state back to handle
h.NXTMOTOR_setState(NXTMOTOR_State);


end%function
