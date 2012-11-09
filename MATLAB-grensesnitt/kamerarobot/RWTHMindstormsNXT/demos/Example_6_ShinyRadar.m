%% Example 6: ShinyRadar
% This function provides a live moving ultrasonic radar.


function ShinyRadar

%% Clean up previous handles
COM_CloseNXT all

%% Set up Matlab
clear all % if you use clear all, call COM_CloseNXT all before, as we did!
close all
format compact


%% Set up ports & vars
portMotor   = MOTOR_A;
portUS      = SENSOR_4;
GearFactor  = 4;
MotorRange  = 170 * GearFactor;
MotorSpeed  = 25;


%% Initialize GFX
figure('Name', 'Shiny Radar'); %, 'Position', [50 60 1200 500]);
axis equal
axis([-170 170 0 170]); 
set(gca, 'Color', 'black');
hold on


hScanLine = ResetFigure(false);



%% Open connection
h = COM_OpenNXT('bluetooth.ini', 'check');
COM_SetDefaultNXT(h);


%% Reset Motor, open sensor
StopMotor all off
NXT_ResetMotorPosition(portMotor, false);
NXT_ResetMotorPosition(portMotor, true);

OpenUltrasonic(portUS);

%% Turn right so start position
SetMotor(portMotor)
    SetPower(20)
    SetAngleLimit(MotorRange/2 - 20)
SendMotorSettings
WaitForMotor(portMotor)
pause(0.5)
StopMotor(portMotor, 'off');


%% First motor go
tmp = GetMotorSettings(portMotor);
StartPos = tmp.TachoCount;

SetMotor(portMotor);
    SetPower(-MotorSpeed);
    SpeedRegulation on
    SetAngleLimit(MotorRange);
SendMotorSettings



%% Main loop
while(true)
    
    % get current pos
    tmp = GetMotorSettings(portMotor);
    CurPos = tmp.TachoCount;
    phi = pi - ((CurPos - (StartPos - MotorRange)) / MotorRange) * pi;
    %alpha = phi * 180 / pi
    %x = cos(phi)
    %y = sin(phi)
    
    % get ultrasonic
    distUS = GetUltrasonic(portUS);

    
    % plot where we are
    set(hScanLine, 'XData', [0; cos(phi) * 150])
    set(hScanLine, 'YData', [0; sin(phi) * 150]);
    
    % plot radar dot
    if (distUS > 1) && (distUS < 200)
        plot(cos(phi) * distUS, sin(phi) * distUS, 'g.')
    end%if
    
    
    drawnow
    
    % reverse direction if necessary
    if CurPos < (StartPos - MotorRange)
        StopMotor(portMotor, 'off')
        NXT_ResetMotorPosition(portMotor, false);
        NXT_ResetMotorPosition(portMotor, true);

        SetMotor(portMotor)
            SetPower(MotorSpeed)
            SetAngleLimit(MotorRange)
        SendMotorSettings
        
        hScanLine = ResetFigure(true);
    end%if
    
    % reverse direction if necessary
    if CurPos > StartPos
        StopMotor(portMotor, 'off')
        NXT_ResetMotorPosition(portMotor, false);
        NXT_ResetMotorPosition(portMotor, true);
        SetMotor(portMotor)
            SetPower(-MotorSpeed)
            SetAngleLimit(MotorRange)
        SendMotorSettings
        
        hScanLine = ResetFigure(false);
    end%if    
    
    

end%while



%% Clean up
StopMotor all off
CloseSensor(portUS);

COM_CloseNXT(h)


end%function


function hScanNew = ResetFigure(leftTrue)

    cla

%% Plot Radar outlines

    % circles
    col = [0.3 0.3 0.3];
    for j = 1 : 3
        phi = linspace(0, pi, 60) ;% - pi;
        x = cos(phi) * 50 * j;
        y = sin(phi) * 50 * j;
        plot(x, y, '-', 'Color', col)
    end%for

    % lines
    len = 160;
    plot([0; cos(pi/4)*len], [0; sin(pi/4)*len], '-', 'Color', col)
    plot([0; 0], [0; len], '-', 'Color', col)
    plot([0; -cos(pi/4)*len], [0; sin(pi/4)*len], '-', 'Color', col)

    % draw scanline again:
    if leftTrue
        hScanNew = plot([0; -150], [0; 0], '-r');
    else
        hScanNew = plot([0; 150], [0; 0], '-r');
    end%if


    drawnow
end%function
