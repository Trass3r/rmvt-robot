%Navigation Abstract superclass for navigation classes
%
% An abstract superclass for implementing navigation classes.  This class
% subclasses the Matlab handle class which means that pass by reference semantics
% apply.
%
% Usage for subclass:
%
%   nav = Navigation(occgrid, options)  create an instance object
%
%   nav                     show summary statistics about the object
%   nav.visualize()         display the occupancy grid
%
%   nav.plan(goal)          plan a path to coordinate goal
%   nav.path(start)         display a path from start to goal
%   p = nav.path(start)     return a path from start to goal
%
% Object properties (read only)
%
%   occgrid     occupancy grid, world model
%   goal        goal coordinate
%
% Methods to be provided in subclass:
%   goal_set(goal)
%   world_set(occgrid)
%   navigate_init()
%   navigate_init2()
%
% Methods to be provided in subclass:
%
%   plan()      generate a plan for motion to goal
%   next()      returns coordinate of next point on path from start to goal
%   

% Peter Corke 8/2009.

classdef Navigation < handle

    properties
        occgrid     % occupancy grid
        goal        % goal coordinate
        start       % start coordinate

        navhook     % function handle, called on each navigation iteration
        verbose     % verbosity
        seed            % current random seed
    end

    methods (Abstract)
        next(obj)
    end % method Abstract

    methods

        % constructor
        function nav = Navigation(occgrid, varargin)
            
            nav.occgrid_set(occgrid);
            if nargin >= 1
                nav.occgrid = occgrid;
            end

            % default values of options
            nav.verbose = false;
            nav.navhook = [];
            nav.seed = [];

            % parse options
            count = 1;
            while count <= length(varargin)
                switch lower(varargin{count})
                case 'verbose'
                    nav.verbose = true; count = count+1;
                case 'hook'
                    nav.navhook = varargin{count+1}; count = count+1;
                case 'seed'
                    nav.seed = varargin{count+1}; count = count+1;
                otherwise
                    error( sprintf('unknown option <%s>', varargin{count}));
                end
                count = count + 1;
            end

            % save current random seed so we can repeat the expt
            defaultStream = RandStream.getDefaultStream;
            if isempty(nav.seed)
                nav.seed = defaultStream.State;
            else
                defaultStream.State = nav.seed;
            end
        end

        % set the occupancy grid
        %  can be overriden in a subclass
        function occgrid_set(nav, og)
            nav.occgrid = og;
        end

        % set the goal coordinate
        %  can be overriden in a subclass
        function goal_set(nav, goal)
            if nav.occgrid( goal(2), goal(1)) == 1
                error('Navigation: cant set goal inside obstacle');
            else
                nav.goal = goal;
            end
        end


        % invoked from subclass path() method
        function p = path(nav, start)
            if nargin < 2
                % display the world
                nav.visualize();

                % prompt the user to click a goal point
                fprintf('** click a starting point ');
                [x,y] = ginput(1);
                fprintf('\n');
                nav.start = round([x;y]);
             else
                nav.start = start(:);
             end

             nav.navigate_init();

            if nargout == 0
                % render the world
                nav.visualize();
            end

            nav.navigate_init2(start);

            p = [];
            robot = nav.start;

            % iterate using the next() method until we reach the goal
            while true
                if nargout == 0
                    plot(robot(1), robot(2), 'g.');
                    drawnow 
                end

                % move to next point on path
                robot = nav.next(robot);
                p = [p; robot'];

                if isempty(robot)
                    % we've arrived
                    break
                end
                if isa(nav.navhook, 'function_handle')
                    nav.navhook(nav, robot(1), robot(2));
                end
            end
            % only return p if nargout>0
        end

        function verbosity(nav, v)
            nav.verbose = v;
        end
            
        % called at each point on the path as
        %   navhook(nav, robot)
        %
        % can be used for logging data, animation, etc.
        function navhook_set(nav, navhook)
            nav.navhook = navhook
        end

            
        function visualize(nav)
            clf
            image(nav.occgrid+1)
            colormap([1 1 1; 1 0 0;])  
            set(gca, 'Ydir', 'normal');
            xlabel('x');
            ylabel('y');
            grid on
            hold on
        end

        function navigate_init(nav)
        end

        % initialize navigation for this starting point
        function navigate_init2(nav, robot)
        end

        function display(nav)
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(nav) );
        end % display()

        function s = char(nav)
            s = 'Nav object';
        end

        function message(nav, str)
            if nav.verbose
                fprintf('Navigation:: %s\n', str);
            end
        end
    end % method

end % classdef
