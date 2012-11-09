%% Example 5: Next Generation Ultrasound
% This script demonstrates the results of the ultrasound "snapshot mode"!
% Interpretation of the results however is difficult.
%
% Just connect an NXT to the USB port, adjust the US port (or connect it to
% SENSOR_2), and see what's happening. The script will exit after 200
% measurements...


%% Set up Matlab
clear
close all
format compact


%% Set up ports
portUS      = SENSOR_2;


%% Get USB handle
COM_CloseNXT all
h = COM_OpenNXT();
COM_SetDefaultNXT(h);


%% Lets go then!
figure('name', 'Next Generation Ultrasound')
set(gca, 'Color', 'black');
hold on


OpenUltrasonic(portUS, 'snapshot')

n          = 8;         % bytes the US sensor received
count      = 200;       % how many readings until end?
plotcols   = 8;         % how many out of n echos to plot?
outOfRange = 160;       % setting for out of range readings

colors = flipud(hot(8));

data = zeros(1, n); 
allX = (1:count+1)';


for i = 1 : count
    USMakeSnapshot(portUS)
    pause(0.05);            % wait for the sound to travel
    echos = USGetSnapshotResults(portUS);

    echos(echos == 255) = outOfRange;

    echos = [echos(1); diff(echos)];

    data = vertcat(data, echos');
    x = allX(1:i+1);
    
    clf
    hold on
    set(gca, 'Color', 'black');
    
    axis([0 count 0 outOfRange])

    for j = plotcols : -1 : 1
        area(x, data(:, j) , 'FaceColor', colors(j, :))
    end
    
end%for


%% Clean up
CloseSensor(portUS)
COM_CloseNXT(h);
