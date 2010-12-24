classdef DXform < Navigation

    properties
        metric;     % distance metric
        distance;   % distance transform results
    end

    methods

        function dx = DXform(world, varargin)

            % TODO NEEDS PROPER ARG HANDLER


            % invoke the superclass constructor
            dx = dx@Navigation(world, varargin{:});

            dx.metric = 'euclidean';

            %% MORPH THE WORLD

            % set up the distance metrics
            if nargin < 3
                %dx.metric = 'cityblock';
            end

        end

        function s = char(dx)
            s = sprintf('dx2 object:');
            if ~isempty(dx.goal)
                s = strcat( sprintf(' goal=%d,%d\n', dx.goal(1), dx.goal(2)) );
            end
        end

        function setworld()
            % invoked by superclass constructor
        end

        function setgoal(dx, goal)
            % for the imorph primitive we need to set the target pixel to 0,
            % obstacles to NaN and the rest to Inf.
            % invoked by superclass constructor

            if ~isempty(goal)
                % point goal case
                world(world>0) = NaN;
                world(world==0) = Inf;
                if world(goal(2), goal(1)) > 0
                    error('goal inside obstacle')
                else
                    world(goal(2), goal(1)) = 0;
                end
            else
                world(world==0) = Inf;
                world(world~=Inf) = 0;
            end

        end
            
        function plan(dx, goal, show)
            % plan()
            % plan(dt)
            % plan(goal)
            % plan(goal, dt)
            %
            %DISTANCEXFORM Distance transform of occupancy grid
            %
            %   dist = distancexform(world, goal)
            %
            %   Compute the distance transform for the occupancy grid, world, with
            %  respect to the specified goal point (x,y).
            %
            %   dist = distancexform(world, goal, metric)
            %
            %  Specify the metric, either 'cityblock' or 'Euclidean'
            %
            %   dist = distancexform(world, goal, metric, show)
            %
            % Show an animation of the distance transform being formed, with a delay
            % of show seconds between frames.


            if nargin < 3
                show = 0;
            end

            if nargin > 1
                dx.goal = goal;
            end

            if isempty(dx.goal)
                error('No goal specified');
            end

            %dx.occgrid(dx.goal(2), dx.goal(1))
            dx.distance = distancexform(dx.occgrid, dx.goal, dx.metric, show);


        end


        function visualize(dx, varargin)
            visualize@Navigation(ds, 'distance', dx.distance);

        end

        function n = next(dx, robot)
            directions = [
                -1 -1
                0 -1
                1 -1
                -1 0
                0 0
                1 0
                -1 1
                0 1
                1 1];

            x = robot(1); y = robot(2);
            region = dx.distance(y-1:y+1,x-1:x+1);

            [mn,k] = min(region(:));

            x = x + directions(k,2);
            y = y + directions(k,1);

            if all([x;y] == dx.goal)
                n = [];     % indicate we are at the goal
            else
                n = [x; y];  % else return the next closest point to the goal
            end
            % beyond the edge of the map there be dragons...
            x = max(2, min(numcols(dx.distance)-1, x));
            y = max(2, min(numrows(dx.distance)-1, y));

        end % next

        function visualize3d(dx, p, varargin)
            surf(dx.distance);
            shading interp
            k = sub2ind(size(dx.distance), p(:,2), p(:,1));
            height = dx.distance(k);
            hold on
            if length(varargin) == 0
                varargin{1} = 'k.';
            end
            plot3(p(:,1), p(:,2), height, varargin{:})             
        end
    end % methods
end % classdef
