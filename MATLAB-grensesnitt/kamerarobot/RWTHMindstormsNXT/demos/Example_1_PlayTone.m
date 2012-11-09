%% Example 1: Play Tone and Get Battery Level
% Example to play a specific tone with the NXT Brick and retrieve the current battery level: 
%
% Prepare workspace by cleaning all old settings to be on the safe side. 
clear all
close all

% Open new NXT connection according to the previous generated configuration file. 
handle = COM_OpenNXT('bluetooth.ini', 'check');
COM_SetDefaultNXT(handle);

%Play tone with frequency 800Hz and duration of 500ms. 
NXT_PlayTone(500,500);

NXT_PlayTone(500,600);

%Get current battery level. 
voltage = NXT_GetBatteryLevel

%Close Bluetooth connection. 
COM_CloseNXT(handle);