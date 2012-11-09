%% Tool: ToolboxTest
% This script tries to call as many toolbox functions as (reasonably) possible,
% with a lot of parameter combinations. Consider it as testcase
% that every toolbox MUST pass before release and as cross-check
% if the installation is valid.
% Make sure to connect a digital sensor to SENSOR_4 before starting.
% Motors on MOTOR_A and MOTOR_B and a light-sensor on SENSOR_1 are optional.


%% Clean up previous handles
COM_CloseNXT all

%% Set up Matlab
clear all % if you use clear all, call COM_CloseNXT all before, as we did!
close all
format compact

DebugMode off


%% Set up ports

analogPort = SENSOR_1;
digitalPort = SENSOR_4;
 

%% Connect to NXT
disp('Connecting...')

% different syntaxes and things
h = COM_OpenNXT('bluetooth.ini', 'check');
COM_CloseNXT(h);
h = COM_OpenNXT('bluetooth.ini', 'check');
COM_CloseNXT('all');

h = COM_OpenNXTEx('Any', '', 'bluetooth.ini', 'check');



%% Try the commands WITHOUT default handle
disp('*** Commands without default handle')

%% *** SENSORS
disp('Sensors...')

port = analogPort;
%% analog
%light
OpenLight(port, 'active', h);
NXT_ResetInputScaledValue(port, h);
GetLight(port, h);
CloseSensor(port, h);

OpenLight(port, 'inactive', h);
GetLight(port, h);
CloseSensor(port, h);

%sound
OpenSound(port, 'db', h);
GetSound(port, h);
CloseSensor(port, h);

OpenSound(port, 'dba', h);
GetSound(port, h);
CloseSensor(port, h);

%switch
OpenSwitch(port, h);
GetSwitch(port, h);
CloseSensor(port, h);


port = digitalPort;

%% I2C
disp('I2C...')

%ultrasonic
OpenUltrasonic(port, '', h);
GetUltrasonic(port, h);
CloseSensor(port, h);

OpenUltrasonic(port, 'snapshot', h);
USMakeSnapshot(port, h);
USGetSnapshotResults(port, h);
CloseSensor(port, h);

%compass
OpenCompass(port, h);
GetCompass(port, h);
CalibrateCompass(port, true, h);
pause(0.5);
CalibrateCompass(port, false, h);
CloseSensor(port, h);

%acceleration
OpenAccelerator(port, h);
GetAccelerator(port, h);
CloseSensor(port, h);

%infrared
OpenInfrared(port, h);
GetInfrared(port, h);
CloseSensor(port, h);


%% MOTORS
port = MOTOR_A;
port2 = MOTOR_B;
disp('Motors...')

StopMotor('all', 'brake', h);
StopMotor('all', 'off', h);

SwitchLamp(port, 'on', h);
SwitchLamp(port, 'off', h);

GetMotorSettings(port, h);

ResetMotorAngle(port, h);

MotorRotateAbs(port,50,20, h);

WaitForMotor(port, 1, h);


%% Direct Commands
disp('Some direct commands...')


NXT_ResetMotorPosition(port, true, h);
NXT_ResetMotorPosition(port, false, h);
 
NXT_SendKeepAlive('dontreply', h);

NXT_PlayTone(800, 500, h);
 
sleeptime = NXT_SendKeepAlive('reply', h);
battery   = NXT_GetBatteryLevel(h);
firmware  = NXT_GetFirmwareVersion(h);

battery
sleeptime
firmware


%% Set Default!

COM_SetDefaultNXT(h);

%% Try the commands WITH default handle
disp('*** Commands with default handle')

%% *** SENSORS
disp('Sensors...')

port = analogPort;
%% analog
%light
OpenLight(port, 'active');
NXT_ResetInputScaledValue(port);
GetLight(port);
CloseSensor(port);

OpenLight(port, 'inactive');
GetLight(port);
CloseSensor(port);

%sound
OpenSound(port, 'db');
GetSound(port);
CloseSensor(port);

OpenSound(port, 'dba');
GetSound(port);
CloseSensor(port);

%switch
OpenSwitch(port);
GetSwitch(port);
CloseSensor(port);


port = digitalPort;

%% I2C
disp('I2C...')

%ultrasonic
OpenUltrasonic(port, '');
GetUltrasonic(port);
CloseSensor(port);

OpenUltrasonic(port, 'snapshot');
USMakeSnapshot(port);
USGetSnapshotResults(port);
CloseSensor(port);

%compass
OpenCompass(port);
GetCompass(port);
CalibrateCompass(port, true);
pause(0.5);
CalibrateCompass(port, false);
CloseSensor(port);

%acceleration
OpenAccelerator(port);
GetAccelerator(port);
CloseSensor(port);

%infrared
OpenInfrared(port);
GetInfrared(port);
CloseSensor(port);


%% MOTORS
port = MOTOR_A;
port2 = MOTOR_B;
disp('Motors...')

StopMotor('all', 'brake');
StopMotor('all', 'off');

SwitchLamp(port, 'on');
SwitchLamp(port, 'off');

GetMotorSettings(port);

ResetMotorAngle(port);

MotorRotateAbs(port,50,20);

WaitForMotor(port, 1);


%% Direct Commands
disp('Some direct commands...')


NXT_ResetMotorPosition(port, true);
NXT_ResetMotorPosition(port, false);
 
NXT_SendKeepAlive('dontreply');

NXT_PlayTone(800, 500);
 
sleeptime = NXT_SendKeepAlive('reply');
battery   = NXT_GetBatteryLevel(h);
firmware  = NXT_GetFirmwareVersion(h);

battery
sleeptime
firmware











 






%% More Motor Stuff
disp('More Motor...')
SetMotor(port);
    GetMotor();
    
    SyncToMotor('off');
    SpeedRegulation('off');
    SetRampMode('off');
    SetTurnRatio(0);
    SetAngleLimit(234);
    SetPower(34)
SendMotorSettings();    

pause(0.5);
StopMotor(port, 'off');

SetMotor(port);
    SyncToMotor('off');
    SpeedRegulation('on');
    SetRampMode('off');
    SetTurnRatio(0);
    SetAngleLimit(134);
    SetPower(-34)
SendMotorSettings();    

pause(0.5);
StopMotor(port, 'off');

SetMotor(port);
    SyncToMotor('off');
    SpeedRegulation('on');
    SetRampMode('up');
    SetTurnRatio(0);
    SetAngleLimit(434);
    SetPower(44)
SendMotorSettings();    

pause(0.5);
StopMotor(port, 'off');

SetMotor(port);
    SpeedRegulation('off');
    SyncToMotor(port2);
    SetRampMode('off');
    SetTurnRatio(34);
    SetAngleLimit(334);
    SetPower(34)
SendMotorSettings();    

pause(0.5);
StopMotor('all', 'off');

CloseSensor(port);

COM_CloseNXT(h);

disp('*** Test successful so far ***')






