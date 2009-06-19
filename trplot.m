%TRPLOT Plot a transformation
%
%   TRPLOT(T)
%   TRPLOT(T, name)
%   TRPLOT(T, name, color)
%
% In a set of axes draw a coordinate frame.  The frame can optionally
% be named and have a specified color.
%
% Name can be a string which is appears in the axis labels, X<name>, etc
%   eg. trplot(trotx(.2), 'cam')
%
% Name can be a cell array {axname, framename}, axname acts the same way
% as above, framename is written near the origin.  We can use LaTeX math
% mode.  For axname include a %c where the axis name (X, Y, Z) will be
% substituted  
%  eg.  trplot(trotx(.2), {'$%c_{cam}$','$\{X_{cam}\}$'})
%
% Note that if you want the frame name to be '{frame}' you need to escape
% the braces, eg. '\{frame\}.

% Copyright (C) 2008, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

function trplot(T, name, color)

    if ndims(T) == 3,
        clf
        hold on
        for i=1:size(T,3),
            trplot(T(:,:,i), num2str(i));
        end
        hold off
        shg
        return
    end

    % trplot( Q.r, fmt, color);
    if size(T) == [3 3],
        T = r2t(T);
    end

	if nargin < 2,
		name = '';
	end
	if nargin < 3,
		color = 'b';
	end

	% create unit vectors
	o = T*[0 0 0 0]';
	x1 = T*[1 0 0 0]';
	y1 = T*[0 1 0 0]';
	z1 = T*[0 0 1 0]';

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
% 	plot3([0;x1(1)]+off(1), [0; x1(2)]+off(2), [0; x1(3)]+off(3), color);
% 	h = text(off(1)+x1(1), off(2)+x1(2), off(3)+x1(3), sprintf(fmt, 'X'));
% 	set(h, 'Color', color);
% 
% 	plot3([0;y1(1)]+off(1), [0; y1(2)]+off(2), [0; y1(3)]+off(3), color);
% 	h = text(off(1)+y1(1), off(2)+y1(2), off(3)+y1(3), sprintf(fmt, 'Y'));
% 	set(h, 'Color', color);
% 
% 	plot3([0;z1(1)]+off(1), [0; z1(2)]+off(2), [0; z1(3)]+off(3), color);
% 	h = text(off(1)+z1(1), off(2)+z1(2), off(3)+z1(3), sprintf(fmt, 'Z'));
% 	set(h, 'Color', color);

    arrow_opts = {'EdgeColor', color, 'FaceColor', color, 'Width', 2};
    text_opts = {'Color', color};
    
    mstart = [o o o]';
    mend = [x1+o y1+o z1+o]';
    
    % draw the 3 arrows
    arrow(mstart(:,1:3), mend(:,1:3), arrow_opts{:});

    % name             for axes
    % {nam1, nam2}     axes, framename
    if isstr(name)
        axname = name;
        framename = '';
    end
    if iscell(name),
        axname = name{1};
        framename = name{2};
    end

    if strcmp(axname, ''),
            fmt = '%c';
    else
            if axname(1) == '$',
                fmt = axname;
                text_opts = {text_opts{:}, 'Interpreter', 'latex'};
            else,
                fmt = sprintf('%%c%s', axname);
            end
    end
    % add the labels to each axis
	h = text(o(1)+x1(1), o(2)+x1(2), o(3)+x1(3), sprintf(fmt, 'X'));
	set(h, text_opts{:});

	h = text(o(1)+y1(1), o(2)+y1(2), o(3)+y1(3), sprintf(fmt, 'Y'));
	set(h, text_opts{:});

	h = text(o(1)+z1(1), o(2)+z1(2), o(3)+z1(3), sprintf(fmt, 'Z'));
	set(h, text_opts{:});
    
    if ~strcmp(framename, ''),
        h = text(o(1)-0.02*x1(1), o(2)-0.02*y1(2), o(3)-0.02*z1(3), framename);
        set(h, 'VerticalAlignment', 'middle', ...
            'Interpreter', 'latex', ...
            'HorizontalAlignment', 'center', text_opts{:});
    end
    
	grid on
	if ~ishold,
		hold off
	end
	axis equal
