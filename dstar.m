% Implementation notes:
%
% All the state is kept in the structure called d
% X is an index into the array of states.
% This is the PROCESS_STATE() function

classdef dstar < handle

    properties
        occgrid

        costmap   % world cost map

        % info kept per cell (state)
        b       % backpointer (0 means not set)
        h       % pathcost

        % info for the open list
        openlist    % list of open states
            % 3xN matrix: state index;  k
        k_old
        k_min
        %k
        G       % goal
        t       % tag: NEW OPEN CLOSED

        openlist_maxlen
        niter
        navhook

        verbose

        % tag state values
        NEW = 0;
        OPEN = 1;
        CLOSED = 2;
    end

    methods

        % constructor
        function ds = dstar(costmap, goal)
            ds.costmap = costmap;
            ds.b = zeros(size(costmap), 'uint32');
            ds.t = zeros(size(costmap), 'uint8');
            ds.h = Inf*ones(size(costmap));
            ds.openlist = zeros(2,0);
            ds.verbose = false;
            ds.navhook = [];

            ds.openlist_maxlen = -Inf;
            ds.niter = 0;

            if nargin >= 2
                ds.setgoal(goal);
            end
        end

        function costmap_set(ds, costmap)
            ds.costmap = costmap;
        end

        function c = costmap_get(ds)
            c = ds.costmap;
        end

        function occgrid_set(ds, og, cost)
            if nargin < 3
                cost = 1;
            end
            costmap = og;
            costmap(costmap==0) = cost;     % unoccupied cells have driving cost
            costmap(costmap==1) = Inf;      % occupied cells have Inf driving cost
        end

        function verbosity(ds, v)
            ds.verbose = v;
        end
            
        function s = display(ds)
            s = char(ds);
        end

        function s = char(ds)
            s = sprintf('D*: open list %d\n', numcols(ds.openlist));
        end

        function setgoal(ds, goal)
            ds.G = sub2ind(size(ds.costmap), goal(2), goal(1));
            if ds.costmap(ds.G) == Inf
                error('cant set goal inside obstacle');
            end
            ds.INSERT(ds.G, 0);
            ds.h(ds.G) = 0;
        end

        function navhook_set(ds, navhook)
            ds.navhook = navhook
        end
            
        function p = navigate(ds, robot, varargin)
            p = robot;
            navhook = if ds.navhook
            while true
                plot(robot(1), robot(2), 'g.');
                drawnow 
                robot = ds.next(robot);
                if isempty(robot)
                    break
                end
                p = [p; robot];
                if isa('function_handle', ds.navhook)
                    ds.navhook(ds, robot(1), robot(2));
                end
            end
        end

        function n = next(ds, current)
            X = sub2ind(size(ds.costmap), state(2), state(1));
            X = ds.b(X);
            if X == 0
                n = [];
            else
                [r,c] = ind2sub(size(ds.costmap), X);
                n = [c,r];
            end
        end

        function reset(ds)
            ds.t = zeros(size(costmap), 'uint8');
        end

        function plan(ds)
            ds.niter = 0;
            while true
                ds.niter = ds.niter + 1;

                if ds.PROCESS_STATE() < 0
                    break;
                end
                pause
                if ds.verbose
                    disp(' ')
                end
            end
        end

        function modify_cost(ds, point, newcost)
            X = sub2ind(size(ds.costmap), point(2), point(1));
            ds.costmap(X) = newcost;
            if ds.t(X) == ds.CLOSED
                ds.INSERT(X, ds.h(X));
            end

        end

        % The main D* function as per the Stentz paper, comments Ln are the original
        % line numbers.
        function r = PROCESS_STATE(d)
            X = d.MIN_STATE();                          % L1

            if isempty(X)                               % L2
                r = -1;
                return;
            end

            k_old = d.GET_KMIN(); d.DELETE(X);          % L3

            if k_old < d.h(X)                           % L4
                if ds.verbose
                    fprintf('k_old < h(X):  %f %f\n', k_old, d.h(X));
                end
                for Y=d.neighbours(X)                   % L5
                    if (d.h(Y) <= k_old) && (d.h(X) > d.h(Y)+d.c(Y,X))  % L6
                        d.b(X) = Y;
                        d.h(X) = d.h (Y) + d.c(Y,X);                    % L7
                    end
                end
            end

            if k_old == d.h(X)                          % L8
                if d.verbose
                    fprintf('k_old == h(X): %f\n', k_old);
                end
                for Y=d.neighbours(X)                   % L9
                    if (d.t(Y) == d.NEW) || ...                         % L10-12
                            ( (d.b(Y) == X) && (d.h(Y) ~= (d.h(X) + d.c(X,Y))) ) || ...
                            ( (d.b(Y) ~= X) && (d.h(Y) > (d.h(X) + d.c(X,Y))) )
                        d.b(Y) = X; d.INSERT(Y, d.h(X)+d.c(X,Y));       % L13
                    end
                 end
            else                                        % L14
                if d.verbose
                    disp('k_old > h(X)');
                end
                for Y=d.neighbours(X)                   % L15
                    if (d.t(Y) == d.NEW) || ( (d.b(Y) == X) && (d.h(Y) ~= (d.h(X) + d.c(X,Y))) )
                        d.b(Y) = X; d.INSERT(Y, d.h(X)+d.c(X,Y));   % L18
                    else
                        if ( (d.b(Y) ~= X) && (d.h(Y) > (d.h(X) + d.c(X,Y))) )
                            d.INSERT(X, d.h(X));                    % L21
                        else
                            if (d.b(Y) ~= X) && (d.h(X) > (d.h(Y) + d.c(Y,X))) && ...
                                    (d.t(Y) == d.CLOSED) && d.h(Y) > k_old
                                d.INSERT(Y, d.h(Y));                % L25
                            end
                        end
                    end
                 end
            end
            
            r = 0;
            return;
        end % process_state(0

        function kk = k(X)
            i = find(ds.openlist(1,:) == X);
            kk = ds.openlist(2,i);
        end

        function INSERT(ds, X, h_new)
            if ds.verbose
                fprintf('insert %d = %f\n', X, h_new);
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

        function ms = MIN_STATE(ds)
            if length(ds.openlist) == 0
                ms = [];
            end
            [kmin,i] = min(ds.openlist(2,:));
            %[kmin,i] = min(ds.k(ds.openlist(1,:)));
            ms = ds.openlist(1,i);
        end

        function kmin = GET_KMIN(ds)
            kmin = min(ds.openlist(2,:));
        end

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
            Y = [r-1 r-1 r-1 0 0  r+1 r+1 r+1; c-1 c c+1 c-1 c+1 c-1 c c+1];
            k = (min(Y)>0) & (Y(1,:)<=dims(1)) & (Y(2,:)<=dims(2));
            Y = Y(:,k);

            Y = sub2ind(dims, Y(1,:)', Y(2,:)')';
        end

    end % method
end % classdef
