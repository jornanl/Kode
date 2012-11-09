%% Example 2: Read Sound Sensor
% Example to read the sound sensor value in db:
%
% Prepare workspace by cleaning all old settings to be on the safe side. 
clear all
close all

% Open new NXT connection according to the previous generated configuration file. 
handle = COM_OpenNXT('bluetooth.ini', 'check');
COM_SetDefaultNXT(handle);

% Set the correct sound sensor mode and input port. 
OpenSound(SENSOR_2, 'DB');

% Get the current sound sensor value in dB. 
value = GetSound(SENSOR_2)

% Close the sound sensor. 
CloseSensor(SENSOR_2);

% Close Bluetooth connection. 
COM_CloseNXT(handle);