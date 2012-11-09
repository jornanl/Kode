function res = toctoc(id)
% Similar to MATLAB's toc(), but extended to save "more states"
%  
% Syntax
%   seconds = toctoc(id) 
%
% Description
%   toctoc(id) returns the time taken (in seconds) since the call to
%   tictic(id) with the same id. Each id generates independent results,
%   e.g. toctoc(3) will not be influenced by the start of tictic(2). 
%
% Note:
%   Do not use too large values for id, as they will take up memory. Also
%   make sure that you do not mix up those tictic ids.
%   Note that toctoc() does not generate a screen output like toc() does.
%
% Example
%   id = 7;
%   tictic(id);
%      % do something...
%   timeTaken = toctoc(id);
%
% See also: tictic
%
% Signature
%   Author: Linus Atorf (see AUTHORS)
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

    global ticticStart;
    try
        res = etime(clock, ticticStart(id, :));
    catch
        if nargin == 0
            % we use the official MATLAB error identifier in this case, as
            % it is appropriate...
            error('MATLAB:inputArgUndefined', 'Input argument "id" is undefined.')
        else
            error('MATLAB:RWTHMindstormsNXT:invalidToctocID', 'Unable to use toctoc with this index / id, did you use tictic(id) before? Do not use CLEAR inbetween!')
        end%if
    end%try
end%function