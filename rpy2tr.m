%RPY2TR Roll/pitch/yaw to homogenous transform
%
% 	TR = RPY2TR([R P Y])
%	TR = RPY2TR(R,P,Y)
%
% Returns a homogeneous tranformation for the specified roll/pitch/yaw angles.
% These correspond to rotations about the Z, Y, X axes respectively.
%
% See also: TR2RPY, EUL2TR

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

function T = rpy2tr(roll, pitch, yaw)
	if (nargin == 1),
		if numcols(roll) ~= 3,
			error('bad arguments')
		end
		pitch = roll(:,2);
		yaw = roll(:,3);
		roll = roll(:,1);
	end

	if numrows(roll) == 1,
		r = rotz(roll) * roty(pitch) * rotx(yaw);
		T = r2t(r);
	else
		for i=1:numrows(roll),
			r = rotz(roll(i)) * roty(pitch(i)) * rotx(yaw(i));
			T(:,:,i) = r2t(r);
		end
	end
