function [DistanceCM] = GetUltrasonic(port, varargin)
% Reads the current value of the NXT ultrasonic sensor
%
% Syntax
%   distance = GetUltrasonic(port)
%
%   distance = GetUltrasonic(port, handle)
%
% Description
%   distance = GetUltraSonic(port) returns the current measurement value distance of the NXT
%   ultrasonic sensor. distance represents the measured distance in cm.
%   If no echo can be detected (which could indicate that either there is
%   no obstacle in the way, or the ultrasound does not get reflected, e.g.
%   by fur-like surfaces), the reading will be 255. If no measurement can
%   be made (defect sensor, cable disconnected, etc.), a value of -1 will
%   be returned.
%
%   The given port number specifies the connection port. The value port can be
%   addressed by the symbolic constants SENSOR_1 , SENSOR_2, SENSOR_3 and SENSOR_4 analog to
%   the labeling on the NXT Brick.
%
%   The last optional argument can be a valid NXT handle. If none is
%   specified, the default handle will be used (call COM_SetDefaultNXT to
%   set one).
%
% Note:
%
%   This function only works when the sensor was correctly opened with
%   OpenUltrasonic(port). If the sensor is being used in snapshot mode,
%   GetUltrasonic will not work correctly!
%
%   For different uses, see also OpenUltrasonic(port, 'snapshot') and the
%   functions USMakeSnapshot and USGetSnapshotResults.
%
%
% Example
%   OpenUltrasonic(SENSOR_4);
%   distance = GetUltrasonic(SENSOR_4);
%   CloseSensor(SENSOR_4);
%
% See also: OpenUltrasonic, USMakeSnapshot, USGetSnapshotResults, CloseSensor, NXT_LSRead, NXT_LSWrite
%
% Signature
%   Author: Linus Atorf, Alexander Behrens (see AUTHORS)
%   Date: 2008/01/15
%   Copyright: 2007-2008, RWTH Aachen University
%
%
% ***********************************************************************************************
% *  This file is part of the RWTH - Mindstorms NXT Toolbox.                                    *
% *                                                                                             *
% *  The RWTH - Mindstorms NXT Toolbox is free software: you can redistribute it and/or modify  *
% *  it under the terms of the GNU General Public License as published by the Free Software     *
% *  Foundation, either version 3 of the License, or (at your option) any later version.        *
% *                                                                                             *
% *  The RWTH - Mindstorms NXT Toolbox is distributed in the hope that it will be useful,       *
% *  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS  *
% *  FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.             *
% *                                                                                             *
% *  You should have received a copy of the GNU General Public License along with the           *
% *  RWTH - Mindstorms NXT Toolbox. If not, see <http://www.gnu.org/licenses/>.                 *
% ***********************************************************************************************

    % check if handle is given; if not use default one
    if nargin > 1
        handle = varargin{1};
    else
        handle = COM_GetDefaultNXT;
    end%if

    % also accept strings as input
    if ischar(port)
        port = str2double(port);
    end%if


    % The old version looked like this:
    %RequestLen = 1;
    %I2Cdata = hex2dec(['02'; '42']); % Read Measurement Byte 0 (see LEGO Mindstorms NXT 
    %                                 % Ultrasonic Sensor - I2C Communication Protocol)

    % retrieve 1 byte from device 0x02, register 0x42
    data = COM_ReadI2C(port, 1, uint8(2), uint8(66), handle);
    
    if isempty(data)
        DistanceCM = -1;
    else
        % this double() is so important!!!
        DistanceCM = double(data(1));
    end%if


    
end%function
