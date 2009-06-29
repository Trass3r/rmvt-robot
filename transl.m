%TRANSL Create translational transform
%
%	TR = TRANSL(X, Y, Z)
%
% Returns a homogeneous transformation representing a translation of X, Y
% and Z.
%
%	TR = TRANSL( P )
%
% Returns a homogeneous transformation for the point P = [X Y Z].  If P is an
% Mx3 matrix return a 4x4xM matrix representing a sequence of homogenous transforms.
%
%
%	P = TRANSL(T)
%
% Returns the translational part of a homogenous transform as a 3-element 
% column vector as a column vector.
%
% If T has 3 dimensions, ie. 4x4xM it is considered a homgoeneous transform
% sequence and returns an Mx3 matrix where each row is the translational component
% corresponding to each transform.
%
% See also: CTRAJ.

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

function r = transl(x, y, z)
	if nargin == 1
		if ishomog(x),
            % T -> P
			r = x(1:3,4);
		elseif ndims(x) == 3
            % T -> P
			r = squeeze(x(1:3,4,:))';
		elseif numrows(x) == 1
            % P -> T
			t = x(:);
			r =    [eye(3)			t;
				0	0	0	1];
        else
            % P -> T
            n = numrows(x);
            r = repmat(eye(4,4), [1 1 n]);
            r(1:3,4,:) = x';
		end
	elseif nargin == 3
        % P -> T
		t = [x; y; z];
		r =    [eye(3)			t;
			0	0	0	1];
	end
