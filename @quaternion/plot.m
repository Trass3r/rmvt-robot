%PLOT Plot a quaternion object 
%
%	PLOT(Q)
%
% Display the quaternion as a rotated coordinate frame.
%
% SEE ALSO: QUATERNION

% $Log: not supported by cvs2svn $
% Revision 1.3  2002/04/14 11:02:54  pic
% Changed see also line.
%
% Revision 1.2  2002/04/01 12:06:47  pic
% General tidyup, help comments, copyright, see also, RCS keys.
%
% $Revision: 1.4 $
%
% Copyright (C) 1999-2002, by Peter I. Corke

function plot(Q, off, fmt, color)
	%axis([-1 1 -1 1 -1 1])

	if nargin < 2,
		off = [0 0 0];
	end
	if nargin < 3,
		fmt = '%c';
	end
	if nargin < 4,
		color = 'b';
	end
	% create unit vectors
	o = [0 0 0]';
	x1 = Q*[1 0 0]';
	y1 = Q*[0 1 0]';
	z1 = Q*[0 0 1]';

	get(gca, 'Tag')
	if strcmp(get(gca, 'Tag'), 'trplot') == 0,
		fprintf('No tag\n');
		clf
		axes
		set(gca, 'Tag', 'trplot')
		fprintf('set tag\n');
		xlabel( 'X');
		ylabel( 'Y');
		zlabel( 'Z');
	end
	ih = ishold;
	hold on
	plot3([0;x1(1)]+off(1), [0; x1(2)]+off(2), [0; x1(3)]+off(3), color);
	h = text(off(1)+x1(1), off(2)+x1(2), off(3)+x1(3), sprintf(fmt, 'X'));
	set(h, 'Color', color);

	plot3([0;y1(1)]+off(1), [0; y1(2)]+off(2), [0; y1(3)]+off(3), color);
	h = text(off(1)+y1(1), off(2)+y1(2), off(3)+y1(3), sprintf(fmt, 'Y'));
	set(h, 'Color', color);

	plot3([0;z1(1)]+off(1), [0; z1(2)]+off(2), [0; z1(3)]+off(3), color);
	h = text(off(1)+z1(1), off(2)+z1(2), off(3)+z1(3), sprintf(fmt, 'Z'));
	set(h, 'Color', color);
	grid on
	if ~ishold,
		hold off
	end
	axis equal
