%SHOWLINK	show all link parameters of ROBOT object
%
%	SHOWLINK(robot)

%	Copyright (C) 1999 Peter. I. Corke

function showlink(r)

	l = r.link;
	for i=1:r.n,
		fprintf('Link %d------------------------\n', i);
		showlink(l{i});
	end
