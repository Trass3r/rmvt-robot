%PRM Class for probabilistic roadmap navigation
%
% A concrete class that implements the PRM navigation algorithm.
% This class subclasses the Navigation class.
%
% Usage for subclass:
%
%   prm = PRM(occgrid, options)  create an instance object
%
%   prm                     show summary statistics about the object
%   prm.visualize()         display the occupancy grid
%
%   prm.plan(goal)          plan a path to coordinate goal
%   prm.path(start)         display a path from start to goal
%   p = prm.path(start)     return a path from start to goal
%

% Peter Corke 8/2009.

classdef PRM < Navigation

    properties
        npoints         % number of points to find
        distthresh

        graph           % graph Object representing random nodes

        vgoal
        vstart
        localGoal
        localPath
        gpath            % path through the graph
    end

    methods

        % constructor
        function prm = PRM(varargin)

            % invoke the superclass constructor
            prm = prm@Navigation(varargin{:});

            prm.graph = PGraph(2);  % planar graph

            % TODO: need a means to set these
            %  {'npoints', [], 'distthresh', []}
            prm.npoints = 100;
            prm.distthresh = 0.3*max(size(prm.occgrid));
        end

        function plan(prm, goal)


            % build a graph over the free space
            prm.message('create the graph');
            prm.graph.clear();
            create_graph(prm);
        end
        
        function path(prm, start, goal)
            
            
            if nargin < 3
                error('must specify start and goal');
            end
            
            % set the goal coordinate
            prm.goal = goal;


            
            % invoke the superclass path function, which iterates on our
            % next method
            path@Navigation(prm, start);

            
        end

        function navigate_init(prm, start)
            % find the vertex closest to the goal
            prm.vgoal = prm.graph.closest(prm.goal);
            
            % find the vertex closest to the start
            prm.vstart = prm.graph.closest(start);

            % are the vertices connected?
            if prm.graph.component(prm.vstart) ~= prm.graph.component(prm.vgoal)
                error('Navigation: start and goal not connected');
            end
            
            % find a path through the graph
            prm.message('planning path through graph');
            prm.graph.goal(prm.vgoal);   % set the goal 
            prm.gpath = prm.graph.path(prm.vstart);
            % the path is a list of nodes from vstart to vgoal
            % discard the first vertex, since we plan a local path to it
            prm.gpath = prm.gpath(2:end);

            % start the navigation engine with a path to the nearest vertex
            prm.graph.showVertex(prm.vstart);

            prm.localPath = bresenham(start, prm.graph.coord(prm.vstart));
            prm.localPath = prm.localPath(2:end,:);
        end

        function n = next(prm, p)

            %disp('next')
            if all(p(:) == prm.goal)
                n = [];     % we've arrived
                return;
            end

            if numrows(prm.localPath) == 0
                % local path is consumed, move to next vertex
                if length(prm.gpath) == 0
                    % we have arrived at the goal vertex
                    % make the path from this vertex to the goal coordinate
                    prm.localPath = bresenham(p, prm.goal);
                    prm.localPath = prm.localPath(2:end,:);
                    prm.localGoal = [];
                else
                    % set local goal to next vertex in gpath and remove it
                    % from the list
                    prm.localGoal = prm.gpath(1);
                    prm.gpath = prm.gpath(2:end);

                    % compute local path to the next vertex
                    prm.localPath = bresenham(p, prm.graph.coord(prm.localGoal));
                    prm.localPath = prm.localPath(2:end,:);
                    prm.graph.showVertex(prm.localGoal);
                end
            end

            n = prm.localPath(1,:);     % take the first point
            prm.localPath = prm.localPath(2:end,:); % and remove from the path
        end

        function create_graph(prm)

            for j=1:prm.npoints
                % pick a point not in obstacle
                while true
                    x = randi(numcols(prm.occgrid));
                    y = randi(numrows(prm.occgrid));
                    if prm.occgrid(y,x) == 0
                        break;
                    end
                end
                new = [x; y];

                vnew = prm.graph.add_node(new);

                [d,v] = prm.graph.distances(new);
                % test neighbours in order of increasing distance
                for i=1:length(d)
                    if v(i) == vnew
                        continue;
                    end
                    if d(i) > prm.distthresh
                        continue;
                    end
                    if ~prm.clearpath(new, prm.graph.coord(v(i)))
                        continue;
                    end
                    prm.graph.add_edge(vnew, v(i));
                end
            end
        end

        function c = clearpath(prm, p1, p2)
            p = bresenham(p1, p2);

            for pp=p'
                if prm.occgrid(pp(2), pp(1)) > 0
                    c = false;
                    return;
                end
            end
            c = true;
        end

        function s = char(prm)
            s = '';
            s = strvcat(s, sprintf('PRM: %dx%d', size(prm.occgrid)));
            s = strvcat(s, sprintf('  graph size: %d', prm.npoints));
            s = strvcat(s, sprintf('  dist thresh: %f', prm.distthresh));
            s = strvcat(s, char(prm.graph) );
        end
        
        
        function visualize(prm, varargin)
            
            opt.backgroundony = false;
            [opt,varargin] = tb_optparse(opt, varargin);
            
            % display the occgrid
            visualize@Navigation(prm);
            
            if ~opt.backgroundony
                hold on
                prm.graph.plot()%varargin{:});
                hold off
                
            end
        end


    end % method
end % classdef
