%CHAR	create string representation of LINK object

%	Copright (C) Peter Corke 1999
function s = char(l)

	jtype = 'RP';

	if l.mdh == 0,
		conv = 'std';
	else
		conv = 'mod';
	end

	s = sprintf('%f\t%f\t%f\t%f\t%c\t(%s)', l.alpha, l.A, l.theta, l.D, ...
		jtype((l.sigma==1) + 1), ...
		conv);
