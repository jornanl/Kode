function [error] = serialAsyncWrite(serialCon,command);
 
error=0;
n = numel(command);

if strcmp(serialCon,'NXT')
    error = nxt_kommando(command);
% Slutt
else

for i = 1:n

    fwrite(serialCon,command(i),'uchar','async');

    %wait for sending
    myTimeOut = 0;
    while(serialCon.BytesToOutput > 0)
        pause(0.01);
        myTimeOut = myTimeOut+1;
        if(myTimeOut == 500)
            error = 1;
            return;
        end;
    end;
    
    %clean up??
end;
end
         
         
         
         


