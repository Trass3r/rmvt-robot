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

        navhook     % function handle, called on each navigation iteration
        verbose     % verbosity
        seed            % current random seed
    end

    methods (Abstract)
        next(obj)
        plan(obj)
    end % method Abstract

    methods

        % TODO fix up set methods for goal
        % setup argument callback like features, can we inherit from that.

        % constructor
        %
        % nav = Navigation(occgrid, options)
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
            nav.verbose = false;
            nav.navhook = [];
            nav.seed = [];
            
            nav = tb_optparse(nav, varargin);
            
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

        function set.goal(nav, goal)
            if ~isempty(nav.occgrid) && nav.occgrid( goal(2), goal(1)) == 1
                error('Navigation: cant set goal inside obstacle');
            else
                nav.goal = goal(:);
            end
            
            nav.goal_set(goal);
        end
        
        % invoked when goal changes
        %  can be overriden in a subclass
        function goal_set(nav, goal)
                disp('in base class goal_set');
        end
        


        % invoked from subclass path() method
        %
        %  get start position, interactively if not given
        %  navigate_init()
        %  visualize the environment
        %  navigate_init2()
        %  iterate on the next() method of the subclass
        function p = path(nav, start)
            if nargin < 2
                % display the world
                nav.visualize();

                % prompt the user to click a goal point
                fprintf('** click a starting point ');
                [x,y] = ginput(1);
                fprintf('\n');
                start = round([x;y]);
            end
            start = start(:);

            if nargout == 0
                % render the world
                nav.visualize('backgroundony');
                hold on
            end
            
            nav.navigate_init(start);

            p = [];
            % robot is a column vector
            robot = start;

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

        %VISUALIZE
        %
        % Display the occupancy grid with an optional distance field
        function visualize(nav, varargin)
            disp('base visualize')
            
            
            opt.goal = [];
            opt.distance = [];
            opt.backgroundony = false;
            
            opt = tb_optparse(opt, varargin);
            
            % we create a display colormap:
            %     0 is red for obstacles
            %     1 is distance 0 from the goal (ie. the goal)
            %     N is large distance from the goal
            cmap = [ 1 0 0];
            clf
            if isempty(opt.distance)
                % simple occ grid:
                %   0 is free,     white, color index = 1
                %   1 is obstacle, red, color index = 2
                cmap = [1 1 1; cmap];  % non obstacles are white
                image(nav.occgrid+1, 'CDataMapping', 'direct');
                colormap(cmap)
                
            else
                % distance is a goal distance image
                d = opt.distance(isfinite(opt.distance));
                maxdist = max(d(:)) + 1;
                
                cmap = [cmap; gray(maxdist)];
                opt.distance(nav.occgrid > 0) = 0;
                image(opt.distance+1, 'CDataMapping', 'direct');
                colormap(cmap)
                colorbar
            end
            
            
            set(gca, 'Ydir', 'normal');
            xlabel('x');
            ylabel('y');
            grid on
            hold on
            
            if ~isempty(nav.goal)
                plot(nav.goal(1), nav.goal(2), 'bd', 'MarkerFaceColor', 'b');
            end
            hold off
        end

        % initialize navigation for this starting point
        function navigate_init(nav, start)
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
