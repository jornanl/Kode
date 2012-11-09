%% Tool: RWTH - Mindstorms NXT Toolbox Benchmark
% This script calls and times some basic toolbox functions,
% so it can be used to get a rough idea of the machine's speed or to
% compare different methods of communication (USB and Bluetooth)

function ToolboxBenchmark()


%% Clean up previous handles
COM_CloseNXT all

%% Set up Matlab
clear all % if you use clear all, call COM_CloseNXT all before, as we did!
close all
format compact

% get no of logical cpus present
import java.lang.*;
r=Runtime.getRuntime;
numCPUs= r.availableProcessors;


disp(' ')
disp('*** RWTH - Mindstorms NXT Toolbox Benchmark')
if ispc; OS = 'Windows'; else OS = 'Linux'; end
rwthver  = ver('RWTHMindstormsNXT');
disp(['    Toolbox version: ' rwthver.Version])
disp(['    MATLAB version:  ' version])
disp(['    Running on ' OS ' (' sprintf('%d',numCPUs) ' CPUs), ' datestr(now)])




%% Set up ports

portLight   = SENSOR_1;
portSound   = SENSOR_2;
portSwitch  = SENSOR_3;
portUS      = SENSOR_4;
portMotor   = MOTOR_A;
portMotor2  = MOTOR_B;
 

%% Connect to NXT

h = COM_OpenNXTEx('Any', '', 'bluetooth.ini', 'check');
COM_SetDefaultNXT(h);

disp(['    Connection type is ' h.ConnectionTypeName])



fprintf('Preparing benchmark... ');

%% Close all sensors to be sure
for j = 0 : 3
    CloseSensor(j);
end%for

%% Open Sensors
OpenLight(portLight, 'active');
OpenSound(portSound, 'db');
OpenSwitch(portSwitch);
OpenUltrasonic(portUS);

%% Stop and reset all motors
StopMotor all off
for j = 0 : 2
    % reset both absolute and relative positions
    NXT_ResetMotorPosition(j, false);
    NXT_ResetMotorPosition(j, true);
end%for


%% Call test functions to load them into memory
TestLight;
TestSound;
TestSwitch;
TestUS;
TestMotorRead;

fprintf('done.\n')


%% Estimate speed for later test
fprintf('Estimating speed... ');

EstimatingTime = 3; % in sec
if h.ConnectionTypeValue == 2 % BT
    PacketsPerSec = 15; %hardcoded BT optimum
else
    PacketCounter = 0;
    tic
    while(toc < EstimatingTime)
        TestMotorRead;
        PacketCounter = PacketCounter + 1;
    end%while
    PacketsPerSec = PacketCounter / EstimatingTime;
end%if



fprintf(['done. (%.1f packets/sec)\n'], PacketsPerSec)



%% Actual benchmarking begins
TestingTime = 3; %in sec
TestingCalls = PacketsPerSec * TestingTime;


fprintf('Starting benchmark, testing time is ~%d sec per unit\n', TestingTime);


fprintf('- Testing BEEP... ');
DoBenchmark(@TestBeep, TestingCalls * 3);

fprintf('- Testing LIGHT... ');
DoBenchmark(@TestLight, TestingCalls);

fprintf('- Testing SOUND... ');
DoBenchmark(@TestSound, TestingCalls);
 
fprintf('- Testing SWITCH... ');
DoBenchmark(@TestSwitch, TestingCalls);

fprintf('- Testing ULTRASONIC... ');
DoBenchmark(@TestUS, TestingCalls / 2);

% for motor read, we let the motor running...
SetMotor(portMotor)
    SetPower(20)
SendMotorSettings
SetMotor(portMotor2)
    SetPower(-20)
SendMotorSettings

fprintf('- Testing MOTOR READ... ');
DoBenchmark(@TestMotorRead, TestingCalls);

StopMotor all off

fprintf('- Testing MOTOR WRITE... ');
DoBenchmark(@TestMotorWrite, TestingCalls);

StopMotor all off


%% Clean up
% Close all sensors
for j = 0 : 3
    CloseSensor(j);
end%for

COM_CloseNXT(h);






%% **** NESTED FUNCTIONS ****

    function DoBenchmark(func, times)
        
        startCPU = cputime;
        tic;
        for zzz = 1 : times % dont need index, avoid confusion with outer func
            func();
        end%for
        cpuTaken = cputime - startCPU;
        timeTaken = toc;

        timePerCall = timeTaken / times;
        callsPerSec = times / timeTaken;
        
        cpuLoad = cpuTaken / timeTaken;
        threadLoad = cpuLoad * numCPUs;
        
        
        fprintf('done.     (took %.1f secs)\n', toc);
        fprintf('  Calls/sec: %.2f\n', callsPerSec);
        fprintf('  Time/call: %.1f ms\n', timePerCall * 1000);
        fprintf('  CPU usage: %.0f%% (thread usage: %.0f%%)\n',cpuLoad * 100, threadLoad * 100)

    end%function

    function TestBeep()
        NXT_PlayTone(fix(rand * 800 + 300), 2);
    end%function

    function TestLight()
        dummy = GetLight(portLight);
    end%function
    
    function TestSound()
        dummy = GetSound(portSound);
    end%function

    function TestSwitch()
        dummy = GetSwitch(portSwitch);
    end%function

    function TestUS()
        dummy = GetUltrasonic(portUS);
    end%function

    function TestMotorRead
        % randomly use different motors
        if rand > 0.5
            dummy = GetMotorSettings(portMotor);
        else
            dummy = GetMotorSettings(portMotor2);
        end%if
    end%function

    function TestMotorWrite
        % randomly use different motors and settings
        if rand > 0.5
            SetMotor(portMotor)
                SetPower(45)
                SpeedRegulation on
                SetAngleLimit(3446)
                SetRampMode down
            SendMotorSettings
        else
            SetMotor(portMotor2)
                SetPower(-80)
                SpeedRegulation off
                SetAngleLimit off
                SetRampMode up
            SendMotorSettings
        end%if
        
    end%function


end%function