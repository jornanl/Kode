function light = GetLight(port, varargin)
% Reads the current value of the NXT light sensor
%
% Syntax
%   light = GetLight(port)
%
%   light = GetLight(port, handle)
%
% Description
%   light = GetLight(port) returns the current light value light of the NXT light
%   sensor. The measurement value light represents the normalized (default) sound value (0..1023 /
%   10 Bit). The normalized value mode is set per default by the function OpenLight.
%   The given port number specifies the connection port. The value port can be
%   addressed by the symbolic constants SENSOR_1 , SENSOR_2, SENSOR_3 and SENSOR_4 analog to
%   the labeling on the NXT Brick.
%
%   The last optional argument can be a valid NXT handle. If none is
%   specified, the default handle will be used (call COM_SetDefaultNXT to
%   set one).
%
%   For more complex settings the function NXT_GetInputValues can be used.
%
% Examples
%   OpenLight(SENSOR_1, 'ACTIVE');
%   light = GetLight(SENSOR_1);
%   CloseSensor(SENSOR_1);
%
% See also: NXT_GetInputValues, OpenLight, CloseSensor, SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4
%
% Signature
%   Author: Linus Atorf, Alexander Behrens (see AUTHORS)
%   Date: 2007/10/15
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

%% check if handle is given; if not use default one
    if nargin > 1
        handle = varargin{1};
    else
        handle = COM_GetDefaultNXT;
    end%if
    
%% Call NXT_GetInputValues function
    in = NXT_GetInputValues(port, handle);

%% Return normalized sound value (0...1023 / 10 Bit)    
    light = double(in.NormalizedADVal);

end    
    