%OA2TR Convert O/A vectors to homogeneous transformation
%
% 	TR = OA2TR(O, A)
%
% Returns a homogeneous tranformation for the specified orientation and 
% approach vectors.
%
% See also: RPY2TR, EUL2TR

% $Log: not supported by cvs2svn $
% $Revision: 1.2 $
% Copyright (C) 1993-2002, by Peter I. Corke

function r = oa2tr(o, a)
	n = cross(o, a);
	r = [unit(n(:)) unit(o(:)) unit(a(:)) zeros(3,1); 0 0 0 1];
