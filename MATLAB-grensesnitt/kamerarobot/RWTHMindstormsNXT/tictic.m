function tictic(id)
% Similar to MATLAB's tic(), but extended to save "more states"
%  
% Syntax
%   tictic(id) 
%
% Description
%   tictic(id) will start tic command determined by a specific id. E.g.
%   tictic(3) will not influence measurements of tictic(2). 
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
% See also: toctoc
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
    ticticStart(id, :) = clock;

end%function
