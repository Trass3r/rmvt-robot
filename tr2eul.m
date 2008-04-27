%TR2EUL Convert a homogeneous transform matrix to Euler angle form
%
%	[PHI THETA PSI] = TR2EUL(TR)
%
% Returns a vector of Euler angles corresponding to the rotational part of 
% the homogeneous transform TR.  The 3 angles correspond to rotations about
% the Z, Y and Z axes respectively.
%
% See also:  EUL2TR, TR2RPY

% $Log: not supported by cvs2svn $
% Revision 1.4  2002/04/14 10:17:54  pic
% Removed code to deal with singular case, main code gives correct answer.
% Updated comments.
%
% Revision 1.3  2002/04/01 11:47:18  pic
% General cleanup of code: help comments, see also, copyright, remnant dh/dyn
% references, clarification of functions.
%
% $Revision: 1.5 $
% Copyright (C) 1993-2002, by Peter I. Corke

function euler = tr2eul(m)
	
	s = size(m);
	if length(s) > 2,
		euler = [];
		for i=1:s(3),
			euler = [euler; tr2eul(m(:,:,i))];
		end
		return
	end

	euler = zeros(1,3);

	% Method as per Paul, p 69.
	% phi = atan2(ay, ax)
	% Only positive phi is returned.
	if abs(m(1,3)) < eps & abs(m(2,3)) < eps,
		% singularity
		euler(1) = 0;
		sp = 0;
		cp = 1;
		euler(2) = atan2(cp*m(1,3) + sp*m(2,3), m(3,3));
		euler(3) = atan2(-sp * m(1,1) + cp * m(2,1), -sp*m(1,2) + cp*m(2,2));
	else
		euler(1) = atan2(m(2,3), m(1,3));
		sp = sin(euler(1));
		cp = cos(euler(1));
		euler(2) = atan2(cp*m(1,3) + sp*m(2,3), m(3,3));
		euler(3) = atan2(-sp * m(1,1) + cp * m(2,1), -sp*m(1,2) + cp*m(2,2));
	end
