classdef Navigation < handle

    properties
        occgrid
        goal
        niter
        navhook

        verbose
    end

    methods (Abstract)
        plan(obj)
        next(obj)
    end % method Abstract

    methods

        % constructor
        function nav = Navigation(occgrid, goal)
            
            if nargin >= 1
                nav.occgrid = occgrid;
            end
            if nargin >= 2
                nav.goal_set(goal);
            end
            nav.verbose = false;
            nav.navhook = [];

            nav.niter = 0;
        end

        function occgrid_set(nav, og)
            nav.occgrid = og;
        end

        function verbosity(nav, v)
            nav.verbose = v;
        end
            
        function goal_set(nav, goal)

            if nav.occgrid( goal(2), goal(1)) == 1
                error('cant set goal inside obstacle');
            else
                nav.goal = goal;
            end
        end

        % called at each point on the path as
        %   navhook(nav, robot)
        %
        % can be used for logging data, animation, etc.
        function navhook_set(nav, navhook)
            nav.navhook = navhook
        end

            
        function visualize(nav)
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

        function path = navigate(nav, robot, varargin)

            % render the world
            nav.visualize();

            % find the starting location for the robot
            if (nargin < 2) || isempty(robot)
                disp('Click a start point');
                [x,y] = ginput(1);
                robot = round([x,y]);
            end

            nav.navigate_init2(robot);

            path = robot;

            % iterate using the next() method until we reach the goal
            while true
                plot(robot(1), robot(2), 'g.');
                drawnow 
                robot = nav.next(robot);
                if isempty(robot)
                    break
                end
                path = [path; robot];
                if isa(nav.navhook, 'function_handle')
                    nav.navhook(nav, robot(1), robot(2));
                end
            end
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

    end % method

end % classdef
