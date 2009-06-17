%ROBOT Robot object constructor
%
% Create a robot
%   ROBOT           create a ROBOT object with no links
%   ROBOT(robot)        create a copy of an existing ROBOT object
%   ROBOT(robot, LINK)  replaces links for robot object
%   ROBOT(LINK, ...)    create from a cell array of LINK objects
%   ROBOT(DH, ...)      create from legacy DH matrix
%   ROBOT(DYN, ...)     create from legacy DYN matrix
%
% Optional trailing arguments are:
%   Name            robot type or name
%   Manufacturer        who built it
%   Comment         general comment
%
% If the legacy matrix forms are used the default name is the workspace
% matrix that held the data.
% Object methods
% Object properties (read/write)
%
%   R.gravity                     direction of gravity [gx gy gz]
%   R.base                        pose of robot's base 4x4 homog xform
%   R.tool                        robot's tool transform, T6 to tool tip: 4x4 homog xform
%   R.qlim                        joint limits, [qlower qupper] nx2
%   R.offset                      kinematic joint coordinate offsets nx1
%   R.name                        name of robot, used for graphical display
%   R.manuf                       annotation, manufacturer's name
%   R.comment                     annoation, general comment
%
%   R.plotopt                     options for plot(robot), cell array
%   R.lineopt                     line drawing options for plot(robot), cell array
%   R.shadowopt                   shadow drawing options for plot(robot), cell array
%
% Object properties (read only)
%
%	R.config		return joint configuration string
%	R.mdh		return MDH convention boolean (0=DH, 1=MDH)
%	R.islimit 		return joint limit boolean vector
%	R.dh		return legacy DH matrix
%	R.dyn		return legacy DYN matrix
%   R.handle    save graphics handles in object
%   R.q     set joint angles for plot(robot)
%
% For robot models prior to Toolbox release 5 (pre Matlab objects) the
% following object constructors are provided.
%
%   L = LINK(DYN_ROW)       create from row of legacy DYN matrix
%   L = LINK(DYN_ROW, CONVENTION)   create from row of legacy DYN matrix
%


% Copyright (C) 1999-2008, by Peter I. Corke
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

classdef robot

    properties
        gravity
        base
        tool
        lineopt
        shadowopt
        name
        manuf
        comment
    end

    properties (SetAccess = private)
        n
        links
        mdh
        handle
        q
        plotopt
    end

    properties (Dependent = true, SetAccess = private)
        config
        offset
        dyn
        dh
        qlim
        spherical
    end

    methods
        function r = robot(L, a1, a2, a3)

            if nargin == 0
                r.name = 'noname';
                r.manuf = '';
                r.comment = '';
                r.links = [];
                r.n = 0;
                r.mdh = 0;
                r.gravity = [0; 0; 9.81];
                r.base = eye(4,4);
                r.tool = eye(4,4);
                r.handle = [];  % graphics handles
                r.q = [];   % current joint angles
                r.plotopt = {};
                r.lineopt = {'Color', 'black', 'Linewidth', 4};
                r.shadowopt = {'Color', 'black', 'Linewidth', 1};
            elseif isa(L, 'robot')
                r = L;
                if nargin == 2,
                    r.links = a1;
                end
            else
                % assume arguments are: name, manuf, comment
                if nargin > 1,
                    r.name = a1;
                else
                    r.name = 'noname';
                end
                if nargin > 2,
                    r.manuf = a2;
                else
                    r.manuf = '';
                end
                if nargin > 3,
                    r.comment = a3;
                else
                    r.comment = '';
                end

                if isa(L, 'double')
                    % legacy matrix
                    dh_dyn = L;
                    clear L
                    for j=1:numrows(dh_dyn)
                        L(j) = links(dh_dyn(j,:));
                    end
                    % get name of variable
                    r.name = inputname(1);
                    r.links = L;
                elseif isa(L, 'link')

                    r.links = L;
                else
                    error('unknown type passed to robot');
                end
                r.n = length(L);

                % set the robot object mdh status flag
                mdh = [];
                for j = 1:length(L)
                    mdh = [mdh L.mdh];
                end
                if all(mdh == 0)
                    r.mdh = mdh(1);
                elseif all (mdh == 1)
                    r.mdh = mdh(1);
                else
                    error('robot has mixed D&H links conventions');
                end

                % fill in default base and gravity direction
                r.gravity = [0; 0; 9.81];
                r.base = eye(4,4);
                r.tool = eye(4,4);
                r.handle = [];
                r.q = [];
                r.plotopt = {};
                r.lineopt = {'Color', 'black', 'Linewidth', 4};
                r.shadowopt = {'Color', 'black', 'Linewidth', 1};
            end
        end


        %MTIMES Multiply robot objects
        %
        % Robot objects can be multiplied r1*r2 which is mechanically equivalent
        % to concatenating the two robots, or mounting robot r2 on the end of robot r1.
        function r2 = mtimes(r, l)
            if isa(l, 'robot')
                r2 = robot(r);
                r2.links = [r2.links l.links];
                r2.n = length(r2.links);
            elseif isa(l, 'link')
                r2 = robot(r);
                r2.links = [r2.links l];
                r2.n = length(r2.links);
            end
        end

        function display(r)
            disp(' ');
            disp([inputname(1), ' = '])
            disp(' ');
            disp(char(r))
            disp(' ');
        end

        %CHAR String representation of robot parametesrs
        function s = char(r)

            s = sprintf('%s (%d axis, %s)', r.name, r.n, r.config);

            if ~isempty(r.manuf)
                s = strcat(s, [' [' r.manuf ']']);
            end
            if ~isempty(r.comment)
                s = strcat(s, [' <' r.comment '>']);
            end
            s = strcat(s, sprintf('\n      grav = [%.2f %.2f %.2f]\n', r.gravity));
            if getfield(r, 'mdh') == 0,
                s = strcat(s, sprintf('\t\tstandard D&H parameters\n'));
            else
                s = strcat(s, sprintf('\t\tmodified D&H parameters\n'));
            end

            s = strcat(s, sprintf('\n\n      alpha           a       theta           d\n'));
            for i = 1:r.n,
                s = strcat(s, sprintf('\n%s', char(r.links(i))));
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %   set/get methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        function r = set.tool(r, v)
            if ~ishomog(v)
                error('base must be a homogeneous transform');
            end
            r.tool = v;
        end

        function r = set.base(r, v)
            if ~ishomog(v)
                error('base must be a homogeneous transform');
            end
            r.base = v;
        end

        function r = set.offset(r, v)
            if length(v) ~= length(v),
                error('offset vector length must equal number DOF');
            end
            L = r.links;
            for i=1:r.n,
                L(i).offset = v(i);
            end
        end

        function v = get.offset(r)
            v = [r.links.offset];
        end

        function r = set.qlim(r, v)
            if numrows(v) ~= r.n,
                error('insufficient rows in joint limit matrix');
            end
            L = r.links;
            for i=1:r.n,
                L(i).qlim = v(i,:);
            end
        end

        function v = get.qlim(r)
            v = [r.links.qlim];
        end

        function r = set.gravity(r, v)
            if isvec(v, 3),
                r.gravity = v;
            else
                error('gravity must be a 3-vector');
            end
        end

        function v = get.config(r)
            v = '';
            for i=1:r.n,
                v(i) = r.links(i).RP;
            end
        end

        %%%%%%%%% joint limit test
        function v = islimit(r,q)
            L = r.links;
            if length(q) ~= r.n,
                error('argument for islimit method is wrong length');
            end
            v = [];
            for i=1:r.n,
                v = [v; r.links(i).islimit(q(i))];
            end
        end

        %%%%%%%%% legacy DH/DYN support
        function v = get.dh(r)
            v = [];
            L = r.links;
            for i=1:r.n,
                v = [v; L(i).dh];
            end
        end

        function v = get.dyn(r)
            v = [];
            L = r.links;
            for i=1:r.n,
                v = [v; L(i).dyn];
            end
        end

        function v = get.spherical(r)
            L = r.links(end-2:end);

            v = false;
            if ~isempty( find( [L(1).a L(2).a L(3).a L(2).d L(3).d] ~= 0 ))
                return
            end

            if (abs(L(1).alpha) == pi/2) & (abs(L(1).alpha + L(2).alpha) < eps),
                v = true;
                return;
            end
        end

    end % methods
end % classdef
