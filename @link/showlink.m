%SHOWLINK	show all parameters of LINK object
%
%	SHOWLINK(link)

%	Copyright (C) 1999 Peter. I. Corke

function showlink(l)

	llab = 6;
	for n =fieldnames(l)'
		v = getfield(l, char(n));
		name = char(n);
		spaces = char(' '*ones(1,llab-length(name)));
		val = num2str(v);
		label = [name spaces ' = '];
		if numrows(val) > 1,
			pad = {label; char(' '*ones(numrows(val)-1,1))};
		else
			pad = label;
		end
		disp([char(pad) val]);
	end
