%TR2JAC Compute a Jacobian to map differentials between frames
%
%	J = TR2JAC(T)
%
% Returns a 6x6 Jacobian matrix to map differentials (joint velocity) between 
% frames related by the homogeneous transform T.
%
% See also: TR2DIFF, DIFF2TR, DIFF

% $Log: not supported by cvs2svn $
% $Revision: 1.2 $
% Copyright (C) 1993-2002, by Peter I. Corke

function J = tr2jac(t)
	J = [	t(1:3,1)'	cross(t(1:3,4),t(1:3,1))'
		t(1:3,2)'	cross(t(1:3,4),t(1:3,2))'
		t(1:3,3)'	cross(t(1:3,4),t(1:3,3))'
		zeros(3,3)	t(1:3,1:3)'		];
		
