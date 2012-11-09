function [ output_args ] = nxt_init()
%INIT_NXT Summary of this function goes here
%   Detailed explanation goes here

%Global variabel som brukes til å gi svar fra NXT-modulen
%global nxt_data;
%clear nxt_data;


COM_CloseNXT('all');
disp('Connecting...');
disp('Initialiserer NXT-forbindelsen');

%Les hjelpfil i /NXT/RWTHMinstormsNXT/doc/ for hjelp til å sette opp BT.
nxt_handle = COM_OpenNXTEx('USB','');
%nxt_handle = COM_OpenNXTEx('Bluetooth','', 'bluetooth.ini', 'check');


COM_SetDefaultNXT(nxt_handle);

disp('Batteri: ');
disp(NXT_GetBatteryLevel);

end
