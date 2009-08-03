classdef PRM < Navigation

    properties
        npoints
        distthresh

        vertices
        edgelist
        edgelen         % length of this edge    

        curLabel        % current label
        ncomponents     % number of components
        labels          % label of each vertex
        labelset        % set of labels

        cost            % distance from goal vertex
        seed            % current random seed

        localGoal
        localPath
    end

    methods

        % constructor
        function prm = PRM(varargin)

            % invoke the superclass constructor
            prm = prm@Navigation(varargin{:});
            prm.labelset = zeros(1, 0);
            prm.labels = zeros(1, 0);
            prm.edgelist = zeros(2, 0);
            prm.edgelen = zeros(1, 0);
            prm.vertices = zeros(2, 0);

            prm.npoints = 100;
            prm.distthresh = 0.4*max(size(prm.occgrid));

            % save current random seed so we can repeat the expt
            defaultStream = RandStream.getDefaultStream;
            prm.seed = defaultStream.State;

            prm.ncomponents = 0;
            prm.curLabel = 0;

        end

        function s = char(prm)
            s = '';
            s = strvcat(s, sprintf('PRM: %dx%d', size(prm.occgrid)));
            s = strvcat(s, sprintf('  %d vertices', numcols(prm.vertices)));
            s = strvcat(s, sprintf('  %d edges', numcols(prm.edgelist)));
            s = strvcat(s, sprintf('  %d components', prm.ncomponents));
        end

        function navigate_init2(prm, p)
            % start the navigation engine with a path to the nearest vertex
            prm.localGoal = prm.closest(p);
            prm.localGoal
            prm.goal
            prm.showVertex(prm.localGoal);
            prm.localPath = bresenham(p, prm.vertices(:,prm.localGoal));
            prm.localPath = prm.localPath(2:end,:);
        end

        function n = next(prm, p)

            n = prm.localPath(1,:);     % take the first point
            prm.localPath = prm.localPath(2:end,:); % and remove from the path

            if all(n == prm.goal)
                n = [];     % we've arrived
                return;
            end

            if numrows(prm.localPath) == 0
                % local path is consumed, move to next vertex
                vnd = prm.neighbours(prm.localGoal);
                [mn,k] = min(prm.cost(vnd(1,:)));
                if mn == 0
                    % we have arrived at the goal
                    prm.localPath = bresenham(prm.vertices(:,k), prm.goal);
                    prm.localPath = prm.localPath(2:end,:);
                    prm.localGoal = [];
                else
                    prm.localGoal = vnd(1,k);
                    prm.localPath = bresenham(p, prm.vertices(:,prm.localGoal));
                    prm.localPath = prm.localPath(2:end,:);
                    prm.showVertex(prm.localGoal);
                end
            end
        end

        function plan(prm)
            create_graph(prm);
            plan_on_graph(prm);
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

                [d,v] = distances(prm, new);
                % test neighbours in order of increasing distance
                label = 0;
                for i=1:length(d)
                    if d(i) > prm.distthresh
                        break;
                    end
                    if ~prm.clearpath(new, prm.vertices(:,v(i)))
                        continue;
                    end
                    if label == 0
                        label = prm.labels(v(i));
                        prm.appendedge(v(i), d(i));
                    elseif label ~= prm.labels(v(i))
                        prm.merge(prm.labels(v(i)), label);
                        prm.appendedge(v(i), d(i));
                    end
                end
                prm.vertices = [prm.vertices new];
                if label == 0
                    label = prm.newlabel();
                end
                prm.labels = [prm.labels label];
            end
        end

        function appendedge(prm, v, d)
            prm.edgelist = [prm.edgelist [v; numcols(prm.vertices)+1]];
            prm.edgelen = [prm.edgelen d];
        end

        function showComponent(prm, c)
            k = prm.labels == c;
            showVertices(prm, k);
        end

        function l = newlabel(prm)
            prm.curLabel = prm.curLabel + 1;
            l = prm.curLabel;
            prm.ncomponents = prm.ncomponents + 1;
            prm.labelset = union(prm.labelset, l);
        end

        % merge label1 and label2, label2 dominates
        function merge(prm, l1, l2)
            prm.labels(prm.labels==l1) = l2;
            prm.ncomponents = prm.ncomponents - 1;
            prm.labelset = setdiff(prm.labelset, l1);
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

        function visualize(prm)
            visualize@Navigation(prm);

            % show vertices
            plot(prm.vertices(1,:), prm.vertices(2,:), ...
                'LineStyle', 'None', ...
                'Marker', 'o', ...
                'MarkerFaceColor', 'b', ...
                'MarkerEdgeColor', 'b');
            % show edges
            for e=prm.edgelist
                v1 = prm.vertices(:,e(1));
                v2 = prm.vertices(:,e(2));
                plot([v1(1) v2(1)], [v1(2) v2(2)], 'k--');
            end
        end

        function showVertices(prm, v)
            for vv=v
                showVertex(prm, vv);
            end
        end

        function showVertex(prm, v)
            plot(prm.vertices(1,v), prm.vertices(2,v), ...
                'LineStyle', 'None', ...
                'Marker', 'o', ...
                'MarkerSize', 12, ...
                'MarkerFaceColor', 'y', ...
                'MarkerEdgeColor', 'y');
        end

            
        % which edges contain v
        function e = edges(prm, v)
            e = [find(prm.edgelist(1,:) == v) find(prm.edgelist(2,:) == v)];
        end

        % return neighbours of v
        function vnd = neighbours(prm, v)
            e = prm.edges(v);
            d = prm.edgelen(e);
            vn = prm.edgelist(:,e);
            vn = vn(:)';
            vn(vn==v) = [];   % remove references to self

            vnd = [vn; d];
        end

        function plan_on_graph(prm)
            % cost is total distance from goal
            prm.cost = zeros(1, numcols(prm.vertices));

            vg = prm.closest(prm.goal);

            prm.descend(vg);
        end

        function descend(prm, vg)

            % get neighbours and their distance
            for vnd = prm.neighbours(vg);
                vn = vnd(1); d = vnd(2);
                if prm.cost(vn) > 0
                    continue;
                end
                prm.cost(vn) = prm.cost(vg) + d;
                descend(prm, vn);
            end
        end

        function d = distance(prm, v1, v2)
            d = norm2( prm.vertices(v1) - prm.vertices(v2));
        end

        function [d,k] = distances(prm, p)
            dx = prm.vertices(1,:) - p(1);
            dy = prm.vertices(2,:) - p(2);
            d = norm2(dx, dy);
            [d,k] = sort(d, 'ascend');
        end

        % return the closest vertext to coordinate p
        function c = closest(prm, p)
            dx = prm.vertices(1,:) - p(1);
            dy = prm.vertices(2,:) - p(2);
            d = norm2(dx, dy);
            [mn,c] = min(d);
        end

    end % method
end % classdef
