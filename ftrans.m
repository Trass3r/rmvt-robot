%FTRANS Transform force/moment
%
%	FT = FTRANS(T, F)
%
% Transforms a force/moment F in the base frame to FT in the frame T.
% F and FT are 6-vectors of the form [Fx Fy Fz Mx My Mz]

% $Log: not supported by cvs2svn $
% $Revision: 1.2 $
% Copyright (C) 1999-2002, by Peter I. Corke

function Ft = ftrans(T, F)

	f = F(1:3); m = F(4:6);
	k = cross(f, transl(T) ) + m;

	mt = rot(T)' * k;
	ft = rot(T)' * F(1:3);

	Ft = [ft; mt];
