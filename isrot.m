%ISROT Test if argument is a rotation matrix
%
%	H = ISROT(R)
%
%  Returns true if the argument is of dimension 3x3.
%
%	H = ISROT(R, true)
%
%  Additional test that determinant is 1

% Copyright (C) 2008, by Peter I. Corke
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

function h = isrot(r, dtest)
	if ndims(r) == 2,
		h =  all(size(r) == [3 3]);
	else
		h = false;
    end

    if nargin > 1,
        if h & dtest,
            h = abs(det(r) - 1) < eps;
        end
    end