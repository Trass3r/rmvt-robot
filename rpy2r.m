%RPY2TR Roll/pitch/yaw to homogenous transform
%
% 	TR = RPY2R([R P Y])
%	TR = RPY2R(R,P,Y)
%
% Returns a 3x3 rotation matrix for the specified roll/pitch/yaw angles.
% These correspond to rotations about the Z, Y, X axes respectively.
%
% See also: TR2RPY, EUL2TR

% $Log: not supported by cvs2svn $
% Revision 1.3  2002/04/14 11:00:30  pic
% Fixed error in axis rotation order.
%
% Revision 1.2  2002/04/01 11:47:16  pic
% General cleanup of code: help comments, see also, copyright, remnant dh/dyn
% references, clarification of functions.
%
% $Revision: 1.1 $
% Copyright (C) 1993-2002, by Peter I. Corke

function r = rpy2r(roll, pitch, yaw)
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
	else
		for i=1:numrows(roll),
			r(:,:,i) = rotz(roll(i)) * roty(pitch(i)) * rotx(yaw(i));
		end
	end
