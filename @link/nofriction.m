%NOFRICTION	return link object with zero friction 
%
%	LINK = NOFRICTION(LINK)
%
%

% MOD HISTORY

%	Copyright (C) 1999 Peter. I. Corke

function  l2 = nofriction(l, only)

	l2 = link(l);

	if strcmpi(only(1:3), 'coulomb'),
		l2.B = 0;
	end
	l2.Tc = [0 0];
