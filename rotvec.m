%ROTVEC Rotation about arbitrary axis
%
% 	TR = ROTVEC(V, THETA)
%
% Returns a homogeneous transformation representing a rotation of THETA 
% about the vector V.
%
% See also: TROTX, ROTX, ROTY, ROTZ.

% Copyright (C) 1993-2002, by Peter I. Corke
%
% CHANGES:
% 7/01	unitize the vector
% $Log: not supported by cvs2svn $
% Revision 1.4  2008-04-27 09:50:54  cor134
% Remove debug print
%
% Revision 1.3  2002/04/01 11:47:16  pic
% General cleanup of code: help comments, see also, copyright, remnant dh/dyn
% references, clarification of functions.
%
% $Revision: 1.5 $

function r = rotvec(v, t)

	v = unit(v);
	ct = cos(t);
	st = sin(t);
	vt = 1-ct;
	v = v(:);
	r =    [ct		-v(3)*st	v(2)*st
		v(3)*st		ct		-v(1)*st
		-v(2)*st	v(1)*st		ct	];
	r = [v*v'*vt+r zeros(3,1); 0 0 0 1];
