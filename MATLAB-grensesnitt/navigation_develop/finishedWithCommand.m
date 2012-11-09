function [myError, outData] = finishedWithCommand(SerialLink)

myError = 1;
outData = 0;

if strcmp(SerialLink,'simulator'),
   outData = 1;

else
    [myError, data] = serialSyncRead(SerialLink,1);
    
    if data == 'm'
        outData = 1;
    else
        outData = 0;
    end
end