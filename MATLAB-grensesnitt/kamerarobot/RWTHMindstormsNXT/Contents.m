% RWTH - Mindstorms NXT Toolbox
% Version 2.00 25-Sep-2008
% Files
%   CalibrateCompass           - Enables calibration mode of the HiTechnic compass sensor
%   checkStatusByte            - Interpretes the status byte of a return package and creates an error message
%   CloseSensor                - Closes a specified sensor port (e.g. turns off the active light mode of the NXT light sensor)
%   COM_CloseNXT               - Closes and deletes a specific NXT handle, or clears all existing handles
%   COM_CollectPacket          - Reads data from a USB or serial/Bluetooth port and retrieves exactly one packet
%   COM_CreatePacket           - Generates a valid Bluetooth packet ready for transmission (i.e. calculates length)
%   COM_GetDefaultNXT          - Returns the global default NXT handle if it was previously set
%   COM_MakeBTConfigFile       - Creates a Bluetooth configuration file which is needed to open a Bluetooth connection.
%   COM_OpenNXT                - Opens a USB or Bluetooth connection to an NXT device and returns a handle for future use
%   COM_OpenNXTEx              - Opens a Bluetooth or USB connection to an NXT device and returns a handle for future use
%   COM_ReadI2C                - Requests and reads sensor data via I2C from a correctly configured digital sensor.
%   COM_SendPacket             - Sends a communication protocol packet (byte-array) via a USB or Bluetooth channel (serial port)
%   COM_SetDefaultNXT          - Sets a global default NXT handle that will be used by other functions if no handle is specified.
%   DebugMode                  - Gets or sets the debug state (i.e. if textOut will print messages to the command window)
%   GetAccelerator             - Reads the current value of the HiTechnic acceleration sensor
%   GetCompass                 - Reads the current value of the HiTechnic compass sensor
%   GetInfrared                - Reads the current value of the Hitechnic infrared sensor (infrared seeker)
%   GetLight                   - Reads the current value of the NXT light sensor
%   GetMemoryCount             - Gets the internal NXT memory counter (manual mapping replica)
%   GetMotor                   - Reads the current motor set by SetMotor(). Raises an error if no motor was set
%   GetMotorSettings           - Returns the current motor data / settings (e.g. position, speed, etc.) from the specified motor
%   GetSound                   - Reads the current value of the NXT sound sensor
%   GetSwitch                  - Reads the current value of the NXT switch / touch sensor
%   GetUltrasonic              - Reads the current value of the NXT ultrasonic sensor
%   MAP_GetCommModule          - Reads the IO map of the communication module
%   MAP_GetInputModule         - Reads the IO map of the input module
%   MAP_GetOutputModule        - Reads the IO map of the output module
%   MAP_GetSoundModule         - Reads the IO map of the sound module
%   MAP_GetUIModule            - Reads the IO map of the user interface module
%   MAP_SetOutputModule        - Writes the IO map to the output module
%   MOTOR_A                    - Symbolic constant MOTOR_A (returns 0)
%   MOTOR_B                    - Symbolic constant MOTOR_B (returns 1)
%   MOTOR_C                    - Symbolic constant MOTOR_C (returns 2)
%   MotorRotateAbs             - Rotates a motor to an absolute angle
%   NXT_GetBatteryLevel        - Returns the current battery level in milli volts
%   NXT_GetFirmwareVersion     - Returns the protocol and firmware version of the NXT
%   NXT_GetInputValues         - Processes a complete sensor reading, i.e. requests input values and collects the answer.
%   NXT_GetOutputState         - Requests and retrieves an output motor state reading
%   NXT_LSGetStatus            - Gets the number of available bytes from low speed (digital) sensor (e.g. ultrasonic)
%   NXT_LSRead                 - Reads data from a low speed (digital) sensor
%   NXT_LSWrite                - Writes the given data to a low speed (digital) sensor
%   NXT_MessageWrite           - Sends a message to NXT
%   NXT_PlaySoundFile          - Plays the given sound file on the NXT Brick
%   NXT_PlayTone               - Plays a tone with the given frequency and duration 
%   NXT_ReadIOMap              - Reads the IO map of the given module ID
%   NXT_ResetInputScaledValue  - Resets the sensors ScaledVal back to 0, depends on currently configured mode (see NXT_SetInputMode)
%   NXT_ResetMotorPosition     - Resets NXT internal counter for specified motor, relative or absolute counter
%   NXT_SendKeepAlive          - Sends a KeepAlive packet. Optional: requests sleep time limit.
%   NXT_SetBrickName           - Sets a new name for the NXT Brick (connected to the specified handle)
%   NXT_SetInputMode           - Sets mode, configures and initializes sensor ready to be read out
%   NXT_SetOutputState         - Sends previously specified settings to current active motor.
%   NXT_StartProgram           - Starts the given program on the NXT Brick
%   NXT_StopProgram            - Stops a specific program on the NXT Brick
%   NXT_StopSoundPlayback      - Stops the current sound playback
%   NXT_WriteIOMap             - Writes the IO map to the given module ID
%   OpenAccelerator            - Initializes and sets the mode of the HiTechnic acceleration sensor
%   OpenCompass                - Initializes and sets the mode of the HiTechnic magnetic compass sensor
%   OpenInfrared               - Initializes and sets the mode of the HiTechnic infrared seeker sensor
%   OpenLight                  - Sets the parameter mode of the NXT light sensor
%   OpenSound                  - Sets the parameter mode of the NXT sound sensor
%   OpenSwitch                 - Sets the parameter mode of the NXT switch / touch sensor
%   OpenUltrasonic             - Initializes and sets the mode of the NXT ultrasonic sensor
%   OptimizeToolboxPerformance - Copies binary versions of typecastc to toolbox for better performance 
%   readFromIniFile            - Reads parameters from a configuration file (usually *.ini)
%   ResetMotorAngle            - Resets the relative angle counter for the given motor
%   SendMotorSettings          - Sends previously specified settings to current active motor.
%   SENSOR_1                   - Symbolic constant SENSOR_1 (returns 0)
%   SENSOR_2                   - Symbolic constant SENSOR_2 (returns 1)
%   SENSOR_3                   - Symbolic constant SENSOR_3 (returns 2)
%   SENSOR_4                   - Symbolic constant SENSOR_4 (returns 3)
%   SetAngleLimit              - Sets the angle limit (in degrees) of the current motor port
%   SetMemoryCount             - Sets the internal NXT memory counter (manual mapping replica)
%   SetMotor                   - Sets the current motor to use for motor setting commands
%   SetPower                   - Sets the power of the current active motor
%   SetRampMode                - Sets the runstate of the current active motor
%   SetTurnRatio               - Sets the turn ratio of the current active motor
%   SpeedRegulation            - Enables / disables the speed regulation mode of the current active motor
%   StopMotor                  - Stops / brakes specified motor. (Synchronisation will be lost after this)
%   SwitchLamp                 - Switches the LEGO lamp on or off (has to be connected to a motor port)
%   SyncToMotor                - Enables synchronization regulation for current active and specified motor
%   textOut                    - Wrapper for fprintf() which can optionally write screen output to a logfile
%   tictic                     - Similar to MATLAB's tic(), but extended to save "more states"
%   toctoc                     - Similar to MATLAB's toc(), but extended to save "more states"
%   USGetSnapshotResults       - Retrieves up to eight echos (distances) stored inside the US sensor
%   USMakeSnapshot             - Causes the ultrasonic sensor to send one snapshot ("ping") and record the echos
%   WaitForMotor               - Pauses execution until specific motor is not running anymore. 
