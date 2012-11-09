%FULLSCAN
% perform a full scan 360 degrees around the robot
% return an array with dim [n x 3], where n is the number of observations,
% first col contains sensorheader, second and third col are global position
% of sensordata given in (x,y)

function [myError, outData] = fullScan(myCon,myHandles)

myError = 1;
outData = [];



if strcmp(myCon,'simulator'),
    outData=simRobot9('f',myHandles);
    outData(:,2:3)=outData(:,2:3)*1000;
    myError=0;
    
elseif strcmp(myCon,'NXT_kamera'),
    camPath=strcat('http://', get(myHandles.camIPText, 'String'), '/img/snapshot.cgi');
    tempstr=strcat('Address to snapshot from web cam: ', camPath);
    disp(tempstr);
    
    [myHandles.vertex_list_right myHandles.vertex_list_left] = mapping_find_and_plot_vertex(camPath, myHandles.kamerarobot, myHandles.vertex_list_right, myHandles.vertex_list_left, myHandles);
    
    
    %Convert from cm to mm
    verteces = 10* [myHandles.vertex_list_right; myHandles.vertex_list_left];
    
    if isempty(verteces),
        %no data points
        disp('no verteces detected by camera')
        return;
    else
        myError=0;
        outData(:, 2) = verteces(:,1);
        outData(:, 3) = verteces(:,2);
        outData(:, 1) = 1;        %sensorheader (just one sensor)
    end
    
    %resetting vertex lists
    myHandles.vertex_list_right=[];
    myHandles.vertex_list_left=[];
    
else
    % serial interface to full scan...
    if myCon.bytesAvailable,
            disp('fullScan: Bytes in buffer before sending "f" to robot. Number of bytes:')
            disp(myCon.bytesAvailable)
            disp('bytes read from buffer:')
            dummyData=fread(myCon)
    end
    
    %if( serialAsyncWrite(myCon,'1') ),
    if( serialAsyncWrite(myCon,'f') ),
        set(myHandles.statusText,'String','navigation_develop/fullScan: Unable to send serial data' );
    else
                
        [myError,data1]=serialSyncRead(myCon,1);
        disp('fullScan: Received response to f: ')
        disp(char(data1))
        if(myError)
            disp('error reading byte after sending f ')
            return;
        %elseif data1=='1', % start receiving full scan data
        elseif data1=='f', % start receiving full scan data
            i=0;
            disp('got f, waiting for data')
            data2=0;
            while data2~='g', % while not finished...
                 %disp('getting sensornummer or g to leave mode:')
                bytes_available2=myCon.bytesAvailable;
                 %if myCon.bytesAvailable,
                 [myError,data2]=serialSyncRead(myCon,1);
                 disp('Data motat fra robot i fullscan');
                 disp(data2);
                %end
                 if (myError)
                    return
                elseif data2=='g'
                    %outData
                    %figure(2);
                    b=outData(:,2); % x data from all of the sensors
                    c=outData(:,3); % y data from all of the sensors     
                    %plot (b,c,'bx')
                    
                    
                    disp('scan ferdig, fikk g')
                    set(myHandles.statusText,'String','Finished fullscan' );
                    
                    disp('antall målepunkter mottatt:')
                    disp(length(b))
                    
                elseif data2=='5'   %Has to be here for no data
                    disp('no data 1')   
                elseif data2=='6'             
                    disp('no data 2')
                elseif data2=='7'
                    disp('no data 3')
                 elseif data2=='8'                
                    disp('no data 4')
                elseif (data2=='1' || data2=='2' || data2=='3' || data2=='4'),
                    i=1+i; % Number of row in outData[]
                    %disp('got sensornummer (49-52) waiting for xxyy')
                    %disp(data2-48)
                    %data(1)
                    outData(i,1)=data2 - 48; % sensor number
                    
                    [myError,data3]=serialSyncRead(myCon,4); % data3 has four rows
                    disp('Data fra sensorer');
                    disp(data3);
                    if (myError),
                        return
                    else
                        %converting from signed int,
                        xpos = data3(1)*256+data3(2);
                        if( xpos > 32767) % negative value
                            myX= -(65536-xpos);
                        else
                            myX=xpos;
                        end

                        ypos = data3(3)*256+data3(4);
                        if(ypos > 32767)% negative value
                            myY= -(65536-ypos);
                        else
                            myY=ypos;
                        end
                    
                    end
                    outData(i,2)=myX;
                    outData(i,3)=myY;
                else
                    disp('received f, but the following position data is wrong')
                end
            end
        end
    end
    if myCon.bytesAvailable,
        data = fread(myCon);
        disp('leftover data on serialconnection after fullscan: ')
        disp(double(data))
    end
    
end