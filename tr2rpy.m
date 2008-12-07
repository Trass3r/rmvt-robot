%TR2RPY Convert a homogeneous transform matrix to roll/pitch/yaw angles
%
%	[R P Y] = TR2RPY(M)
%
% Returns a vector of roll/pitch/yaw angles corresponding to M, either a rotation
% matrix or the rotation part of a homogeneous transform.
% part of the homogeneous transform TR.  The angles correspond to rotations
% about the X, Y and Z axes respectively.
%
% See also:  RPY2TR, TR2EUL

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

function rpy = tr2rpy(m)
	
	s = size(m);
	if length(s) > 2,
		rpy = [];
		for i=1:s(3),
			rpy = [rpy; tr2rpy(m(:,:,i))];
		end
		return
	end
	rpy = zeros(1,3);

	if abs(m(1,1)) < eps & abs(m(2,1)) < eps,
		% singularity
		rpy(3) = 0;
		rpy(2) = atan2(-m(3,1), m(1,1));
		rpy(1) = atan2(-m(2,3), m(2,2));
	else,
		rpy(3) = atan2(m(2,1), m(1,1));
		sp = sin(rpy(1));
		cp = cos(rpy(1));
		rpy(2) = atan2(-m(3,1), cp * m(1,1) + sp * m(2,1));
		rpy(1) = atan2(sp * m(1,3) - cp * m(2,3), cp*m(2,2) - sp*m(1,2));
	end
