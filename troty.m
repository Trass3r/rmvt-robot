%ROTY Rotation about Y axis
%
%	TR = ROTY(theta)
%
% Returns a homogeneous transformation representing a rotation of theta 
% about the Y axis.
%
% See also: ROTX, ROTZ, ROTVEC.

% $Log: not supported by cvs2svn $
% Revision 1.2  2002/04/01 11:47:16  pic
% General cleanup of code: help comments, see also, copyright, remnant dh/dyn
% references, clarification of functions.
%
% $Revision: 1.1 $
% Copyright (C) 1993-2002, by Peter I. Corke

function r = troty(t)
	r =    [roty(t) [0 0 0]'; 0 0 0 1];
