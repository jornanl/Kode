function [ output_args ] = nxt_cam_init(handles)
%INIT_NXT Initialize NXT camera robot
%   Detailed explanation goes here


% Cam-robot object as global structure
handles.kamerarobot = mapping_init();

%Vertex-lists (now defined in LegoGui.m)
%handles.vertex_list_left = [];
%handles.vertex_list_right = [];

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
