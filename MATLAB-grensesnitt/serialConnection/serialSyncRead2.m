function [error,data]=serialSyncRead2(serialCon)

global nxt_data;
error=0;

% Nytt for NXT
if strcmp(serialCon,'NXT')
    data = nxt_data;
    clear nxt_data;
% Slutt
else
    data = fread(serialCon); 
end


if( isempty(data) )
    error=1;
end;