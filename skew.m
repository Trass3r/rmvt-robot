%SKEW Convert to/from skew symmetric form
%
%   S = skew(v)
%
% Create 3x3 skew-symmetric matrix from 3x1 vector v
%
%   v = skew(S);
%
% Assuming that S is skew-symmetric, extract the 3 unique values from it.  We
% actually take the mean of the two elements that correspond to each unique
% element, ie. vx = 0.5*(S(3,2)-S(2,3))

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
		d = [	t1(1:3,4);
			0.5*[t1(3,2)-t1(2,3); t1(1,3)-t1(3,1); t1(2,1)-t1(1,2)]];
	end
