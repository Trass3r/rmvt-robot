%NOFRICTION	return robot object with zero link friction 
%
%	ROBOT = NOFRICTION(ROBOT)
%
%

% MOD HISTORY

%	Copyright (C) 1999 Peter. I. Corke

function  r2 = nofriction(r, varargin)

	r2 = robot(r);

	for i=1:r2.n,
		l2{i} = nofriction(r.link{i}, varargin{:});
	end

	r2.link = l2;
	r2.name = ['NF/' r.name];
