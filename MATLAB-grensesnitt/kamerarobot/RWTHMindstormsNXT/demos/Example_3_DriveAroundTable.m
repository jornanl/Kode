%% Example 3: Drive Around Table
% In this little demo, our bot drives a square on the floor around a well known table.

%% Clear and close
clear all
close all


%% Constants
TableLength      = 2850; % in degrees of motor rotations :-)
QuarterTurnTicks = 245;  % in motor degrees, how much is a 90° turn of the bot?


%% Open NXT connection
handle = COM_OpenNXT('bluetooth.ini', 'check');
COM_SetDefaultNXT(handle);


%% Initialize Motors...
% we send this in case they should still be spinning or something...
StopMotor('all', 'off');
% and we also "clean up" before we start:
ResetMotorAngle(MOTOR_B);
ResetMotorAngle(MOTOR_C);


%% Start the engines, main loop begins (repeated 4 times)
% 4 times because we got 4 equal sides of the table :-)
for j = 1 : 4

%% Drive
    SetMotor(MOTOR_B);
        SyncToMotor(MOTOR_C); % this means we have to set parameters only once
        SetPower(75);
        SetAngleLimit(TableLength);
        SetTurnRatio(0); % straight ahead
    SendMotorSettings(); % and GO!
    
    % let the robot start:
    pause(1);

%% Check for the end end of table
    WaitForMotor(GetMotor);
    
    % give it a little time to correct its mistakes (hey synchronisation
    % mode :-)
    pause(2);
    
    % apparently we've stopped!
    % then release the motors
    StopMotor('all', 'off');
    % if we don't do that, syncing again doesn't work
    
    % and again, if we don't rest relative counters, synced turning etc doesnt work...
    ResetMotorAngle(MOTOR_B);
    ResetMotorAngle(MOTOR_C);
    
    
%% Now please turn 

    SetMotor(MOTOR_B);
        SyncToMotor(MOTOR_C); % this means we have to set parameters only once
        SetPower(30) % slower is more acurate
        SetAngleLimit(QuarterTurnTicks);
        SetTurnRatio(100) % turn right
    SendMotorSettings();  % and GO!

    % leave the bot time to start turning
    pause(1);
    
%% Check for the end of rotation

    WaitForMotor(GetMotor);
    
    % give it a little time to correct its mistakes (hey synchronisation mode :-)
    pause(2);
    
    % apparently we've stopped!
    % then release the motors
    StopMotor('all', 'off');
    % if we don't do that, syncing again doesn't work
    
    % and again, if we don't rest relative counters, synced turning etc doesnt work...
    ResetMotorAngle(MOTOR_B);
    ResetMotorAngle(MOTOR_C);
    
%% Thats it. Repeat 4 times....
end%for


% Hey! End of a hard day's work
% Just to show good style, we close down our motors again:
StopMotor('all', 'off');
% although this was completely unnecessary....

% nice


%% Close Bluetooth

COM_CloseNXT(handle);
