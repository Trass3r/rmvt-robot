% Implementation notes:
%
% All the state is kept in the structure called d
% X is an index into the array of states.
% state pointers are kept as matlab array index rather than row,col format

%TODO use pgraph class

% pic 7/09

classdef Dstar < Navigation

    properties
        costmap   % world cost map: obstacle = Inf
        G         % index of goal point

        % info kept per cell (state)
        b       % backpointer (0 means not set)
        t       % tag: NEW OPEN CLOSED
        h       % pathcost

        % list of open states: 2xN matrix
        %   each open point is a column, row 1 = index of cell, row 2 = k
        openlist

        k_old
        k_min

        niter

        openlist_maxlen     % keep track of maximum length

        % tag state values
        NEW = 0;
        OPEN = 1;
        CLOSED = 2;
    end

    methods

        % constructor
        function ds = Dstar(world, goal)

            % invoke the superclass constructor
            ds = ds@Navigation(world);
            ds.occgrid2costmap(ds.occgrid);

            % init the D* state variables
            ds.reset();

            if nargin > 1
                ds.goal_set(goal);
            end

        end

        function reset(ds)
            % build the matrices required to hold the state of each cell for D*
            ds.b = zeros(size(ds.costmap), 'uint32');  % backpointers
            ds.t = zeros(size(ds.costmap), 'uint8');   % tags
            ds.h = Inf*ones(size(ds.costmap));         % path cost estimate
            ds.openlist = zeros(2,0);               % the open list, one column per point

            ds.openlist_maxlen = -Inf;
        end


        function costmap_set(ds, costmap)
            ds.costmap = costmap;
        end

        function c = costmap_get(ds)
            c = ds.costmap;
        end

        function occgrid2costmap(ds, og, cost)
            if nargin < 3
                cost = 1;
            end
            ds.costmap = og;
            ds.costmap(ds.costmap==1) = Inf;      % occupied cells have Inf driving cost
            ds.costmap(ds.costmap==0) = cost;     % unoccupied cells have driving cost
        end

        function s = char(ds)
            s = '';
            s = strvcat(s, sprintf('D*: costmap %dx%d, open list %d\n', size(ds.costmap), numcols(ds.openlist)));
        end

        function goal_set(ds, goal)
            disp('in goal_set');
            goal_set@Navigation(ds, goal);

            % keep goal in index rather than row,col format
            ds.G = sub2ind(size(ds.occgrid), goal(2), goal(1));
            if ds.costmap(ds.G) == Inf
                error('cant set goal inside obstacle');
            end
            ds.INSERT(ds.G, 0, 'goalset');
            ds.h(ds.G) = 0;
        end

        function visualize(ds, varargin)
            visualize@Navigation(ds, 'distance', ds.h);

        end

        function n = next(ds, current)
            X = sub2ind(size(ds.costmap), current(2), current(1));
            X = ds.b(X);
            if X == 0
                n = [];
            else
                [r,c] = ind2sub(size(ds.costmap), X);
                n = [c,r];
            end
        end

        function plan(ds, goal)
            if nargin > 1
                ds.goal = goal;
            end
            % for replanning no goal is needed, 
            if isempty(ds.goal)
                error('must specify a goal point');
            end
            
            ds.niter = 0;
            spinner = '-\|/';
            spincount = 0;
            while true
                if mod(ds.niter, 10) == 0
                    spincount = spincount + 1;
                    fprintf('\r%c', spinner( mod(spincount, length(spinner))+1 ) );
                end
                ds.niter = ds.niter + 1;

                if ds.PROCESS_STATE() < 0
                    break;
                end
                if ds.verbose
                    disp(' ')
                end
            end
            fprintf('\r');
        end

        function modify_cost(ds, point, newcost)
            X = sub2ind(size(ds.costmap), point(2), point(1));
            ds.costmap(X) = newcost;
            if ds.t(X) == ds.CLOSED
                ds.INSERT(X, ds.h(X), 'modifycost');
            end
        end

        % The main D* function as per the Stentz paper, comments Ln are the original
        % line numbers.
        function r = PROCESS_STATE(d)

            %% states with the lowest k value are removed from the
            %% open list
            X = d.MIN_STATE();                          % L1

            if isempty(X)                               % L2
                r = -1;
                return;
            end

            k_old = d.GET_KMIN(); d.DELETE(X);          % L3

            if k_old < d.h(X)                           % L4
                if d.verbose
                    fprintf('k_old < h(X):  %f %f\n', k_old, d.h(X));
                end
                for Y=d.neighbours(X)                   % L5
                    if (d.h(Y) <= k_old) && (d.h(X) > d.h(Y)+d.c(Y,X))  % L6
                        d.b(X) = Y;
                        d.h(X) = d.h (Y) + d.c(Y,X);                    % L7
                    end
                end
            end

            %% can we lower the path cost of any neighbours?
            if k_old == d.h(X)                          % L8
                if d.verbose
                    fprintf('k_old == h(X): %f\n', k_old);
                end
                for Y=d.neighbours(X)                   % L9
                    if (d.t(Y) == d.NEW) || ...                         % L10-12
                            ( (d.b(Y) == X) && (d.h(Y) ~= (d.h(X) + d.c(X,Y))) ) || ...
                            ( (d.b(Y) ~= X) && (d.h(Y) > (d.h(X) + d.c(X,Y))) )
                        d.b(Y) = X; d.INSERT(Y, d.h(X)+d.c(X,Y), 'L13');   % L13
                    end
                 end
            else                                        % L14
                if d.verbose
                    disp('k_old > h(X)');
                end
                for Y=d.neighbours(X)                   % L15
                    if (d.t(Y) == d.NEW) || ( (d.b(Y) == X) && (d.h(Y) ~= (d.h(X) + d.c(X,Y))) )
                        d.b(Y) = X; d.INSERT(Y, d.h(X)+d.c(X,Y), 'L18');   % L18
                    else
                        if ( (d.b(Y) ~= X) && (d.h(Y) > (d.h(X) + d.c(X,Y))) )
                            d.INSERT(X, d.h(X), 'L21');                    % L21
                        else
                            if (d.b(Y) ~= X) && (d.h(X) > (d.h(Y) + d.c(Y,X))) && ...
                                    (d.t(Y) == d.CLOSED) && d.h(Y) > k_old
                                d.INSERT(Y, d.h(Y), 'L25');                % L25
                            end
                        end
                    end
                 end
            end
            
            r = 0;
            return;
        end % process_state(0

        function kk = k(ds, X)
            i = find(ds.openlist(1,:) == X);
            kk = ds.openlist(2,i);
        end

        function INSERT(ds, X, h_new, where)
            if ds.verbose
                fprintf('insert (%s) %d = %f\n', where, X, h_new);
            end

            i = find(ds.openlist(1,:) == X);
            if length(i) > 1
                error( sprintf('d*:INSERT: state in open list %d times', X) );
            end

            if ds.t(X) == ds.NEW
                k_new = h_new;
                % add a new column to the open list
                ds.openlist = [ds.openlist [X; k_new]];
            elseif ds.t(X) == ds.OPEN
                k_new = min( ds.openlist(2,i), h_new );
            elseif ds.t(X) == ds.CLOSED
                k_new = min( ds.h(X), h_new );
                % add a new column to the open list
                ds.openlist = [ds.openlist [X; k_new]];
            end

            if numcols(ds.openlist) > ds.openlist_maxlen
                ds.openlist_maxlen = numcols(ds.openlist);
            end

            ds.h(X) = h_new;
            ds.t(X) = ds.OPEN;
        end

        function DELETE(ds, X)
            if ds.verbose
                fprintf('delete %d\n', X);
            end
            i = find(ds.openlist(1,:) == X);
            if length(i) ~= 1
                error( sprintf('d*:DELETE: state %d doesnt exist', X) );
            end
            ds.openlist(:,i) = []; % remove the column
            ds.t(X) = ds.CLOSED;
        end

        % return the index of the open state with the smallest k value
        function ms = MIN_STATE(ds)
            if length(ds.openlist) == 0
                ms = [];
            end
            % find the minimum k value on the openlist
            [kmin,i] = min(ds.openlist(2,:));

            % return its index
            ms = ds.openlist(1,i);
        end

        function kmin = GET_KMIN(ds)
            kmin = min(ds.openlist(2,:));
        end

        % return the cost of moving from state X to state Y
        function cost = c(ds, X, Y)
            [r,c] = ind2sub(size(ds.costmap), [X; Y]);
            dist = sqrt(sum(diff([r c]).^2));
            dcost = (ds.costmap(X) + ds.costmap(Y))/2;

            cost = dist * dcost;
        end

        % return index of neighbour states as a row vector
        function Y = neighbours(ds, X)
            dims = size(ds.costmap);
            [r,c] = ind2sub(dims, X);

            % list of 8-way neighbours
            Y = [r-1 r-1 r-1 r r  r+1 r+1 r+1; c-1 c c+1 c-1 c+1 c-1 c c+1];
            k = (min(Y)>0) & (Y(1,:)<=dims(1)) & (Y(2,:)<=dims(2));
            Y = Y(:,k);
            Y = sub2ind(dims, Y(1,:)', Y(2,:)')';
        end

    end % method
end % classdef
