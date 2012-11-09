%% Example 4: Drive Until Wall
% In this little demo our robot drives forward until it detects a wall.

%% Matlab
clear all
close all


%% Constants
MaxDriveTime = 30; % in seconds, stop after this time...
USPort       = SENSOR_4;


%% Initialize NXT connection
handle = COM_OpenNXT('bluetooth.ini', 'check');
COM_SetDefaultNXT(handle);


%% Reset Motor / remember start position
ResetMotorAngle(MOTOR_B);
ResetMotorAngle(MOTOR_C);
StartPos = GetMotorSettings(MOTOR_B);


%% Prepare sensor
OpenUltrasonic(USPort)


%% And GO!

SetMotor(MOTOR_B);
    SyncToMotor(MOTOR_C);
    SetPower(60);
    SetAngleLimit(0);
    SetTurnRatio(0);
    SetRampMode ('off');
SendMotorSettings();


%% Detect wall (sensor loop)
tictic(1); 
while(toctoc(1) < MaxDriveTime)
   if GetUltrasonic(USPort) < 30
       break
   end%if
end%while


%% Immediately stop all motors!
StopMotor('all', 'off');
% but where are we now?
CurPos = GetMotorSettings(MOTOR_B);


%% Signal
NXT_PlayTone(440, 1000);


%% Drive back home!
% To try to understand what exactly the different rotation count values
% mean, we compare them just for fun
TotalAbsDist = StartPos.TachoCount - CurPos.TachoCount;
Dist         = CurPos.Angle;

SetMotor(MOTOR_B);
    SyncToMotor(MOTOR_C);
    SetPower(-60);
    SetAngleLimit(abs(Dist));
SendMotorSettings();


%% Wait until there
WaitForMotor(1, MaxDriveTime);

% brake
StopMotor('all', 'brake');

pause(1);

%% clean up
CloseSensor(SENSOR_4)
StopMotor ('all', 'off');


%% Close NXT connection
COM_CloseNXT(handle);
