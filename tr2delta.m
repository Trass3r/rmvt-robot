%TR2DELTA Convert transform difference to a delta vector
%
%   Delta = tr2delta(T)
%   Delta = tr2delta(T1, T2)
%
% Return a delta vector (dtx, dty, dtz, dRx, dRy, dRz) equivalent to the 
% infinitessimal transformation T or from T1 to T2.
%
% The Delta is an approximation to the velocity screw.
%
% SEE ALSO: skew, delta2tr

% Copyright (C) 1993-2008, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

function S = skew(v)
    if isvector(v),
        S = [  0   -v(3)  v(2)
              v(3)  0    -v(1)
             -v(2) v(1)   0];
    else
        if ishomog(v)
            d = [	v(1:3,4);
                0.5*[v(3,2)-v(2,3); v(1,3)-v(3,1); v(2,1)-v(1,2)]];
        else
            S = 0.5*[v(3,2)-v(2,3); v(1,3)-v(3,1); v(2,1)-v(1,2)];
        end
	end
