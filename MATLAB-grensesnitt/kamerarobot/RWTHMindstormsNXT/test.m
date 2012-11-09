COM_CloseNXT('all');
disp('Connecting...');
disp('Initialiserer NXT-forbindelsen');

%Les hjelpfil i /NXT/RWTHMinstormsNXT/doc/ for hjelp til å sette opp BT.
%nxt_handle = COM_OpenNXTEx('USB','');
% nxt_handle = COM_OpenNXTEx('Bluetooth','', 'bluetooth.ini', 'check');

nxt_handle = COM_OpenNXT('bluetooth.ini');

COM_SetDefaultNXT(nxt_handle);



