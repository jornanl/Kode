function [error] = serialAsyncWrite(serialCon,command)
disp('The command is: ');
disp(command);
global nxt_data;

error=0;
n = numel(command);

% Nytt for NXT
if strcmp(serialCon,'NXT')
    [error, outData] = nxt_kommando(command);
    if outData
        nxt_data = outData;
    end
% Slutt
else

try
    for i = 1:n
        fwrite(serialCon,char(command(i)),'uchar','async');%uchar=8 bits unsigned char,
        myTimeOut = 0;
        while( serialCon.BytesToOutput > 0)
            pause(0.01);
            myTimeOut = myTimeOut+1;
            if( myTimeOut == 200 )
                error = 1;
                stopasync(serialCon);
                break;
            end
        end
    end
catch
    error=1;
end   
end
