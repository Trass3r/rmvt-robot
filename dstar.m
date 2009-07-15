% Implementation notes:
%
% All the state is kept in the structure called d
% X is an index into the array of states.
% This is the PROCESS_STATE() function

classdef dstar < handle

    properties
        b       % backpointer (0 means not set)
        c       % cost
        h       % pathcost
        k_old
        k_min
        k
        t       % tag: NEW OPEN CLOSED
        openlist    % list of open states
            % 3xN matrix: state index; h; k
        G

        NEW = 0;
        OPEN = 1;
        CLOSED = 2;
    end

    methods

        function kk = k(X)
            i = find(openlist(1,:) == X);
            kk = openlist(3,i);
        end

        function hh = h(X)
            i = find(openlist(1,:) == X);
            hh = openlist(2,i);
        end

        function k_set(X, k_new)
            disp('set k');
            i = find(openlist(1,:) == X);
            openlist(3,i) = k_new;
        end

        function h_set(X, h_new)
            disp('set h');
            i = find(openlist(1,:) == X);
            openlist(2,i) = h_new;
        end

        function d = dstar(c)
            d.b = zeros(size(c), 'uint32');
            d.c = c;
            d.t = zeros(size(c), 'uint8');
        end

        function setgoal(i,j)
            G = sub2ind(size(c), i, j);
            insert(G, 0);
        end

        function insert(X, h_new)
            i = find(openlist(1,:) == X);
            if length(i) > 0
                error( sprintf('d*:insert: state %d already exists', X) );
            end

            if t(X) == NEW
                k_new = h_new;
            elseif t(X) == OPEN
                k_new = min( k(X), h_new );
            elseif t(X) == CLOSED
                k_new = min( h(X), h_new );
            end

            % add a new column to the open list
            openlist = [openlist [X; h_new; k_new]];
            t(X) = OPEN;
        end

        function delete(X)
            i = find(openlist(1,:) == X);
            if length(i) ~= 1
                error( sprintf('d*:delete: state %d doesnt exist', X) );
            end
            openlist(:,i) = []; % remove the column
            t(X) = CLOSED;
        end

        function ms = min_state()
            if length(openlist) == 0
                ms = [];
            end
            [kmin,i] = min(k(openlist(3,:)));
            ms = openlist(1,i);
        end

        function kmin = get_kmin()
            kmin = min(k(openlist(3,:)));
        end

        % return index of neighbour states as a row vector
        function Y = neighbours(X)
            [r,c] = ind2sub(size(c), X);

            % 4-way neighbours for now
            Y = [];
            Y = [Y sub2ind(size(c), r-1, c)];
            Y = [Y sub2ind(size(c), r+1, c)];
            Y = [Y sub2ind(size(c), r, c-1)];
            Y = [Y sub2ind(size(c), r, c+1)];
        end

        function plan()
            while true
                if process_state() < 0
                    break;
                end
            end
        end

        function r = process_state()
            X = min_state();

            if isempty(X)
                r = -1;
                return;
            end

            k_old = get_kmin(d); delete(X);

            if k_old < h(X)
                % for each neighbour Y of X
                for Y=neighbours(X)
                    if (h(Y) <= k_old) && (h(X) > h(Y)+c(Y,X))
                        b(X) = Y; h_set(X,  h(Y) + c(Y,X) );
                    end
                end
            elseif k_old == h(X)
                % for each neighbour Y of X
                for Y=neighbours(X)
                    if (t(Y) == NEW) || ...
                            ( (b(Y) == X) && (h(Y) ~= (h(X) + c(X,Y))) ) || ..
                            ( (b(Y) ~= X) && (h(Y) > (h(X) + c(X,Y))) )
                        b(Y) = X; insert(Y, h(X)+c(X,Y));
                    end
                 end
            else
                % for each neighbour Y of X
                for Y=neighbours(X)
                    if (t(Y) == NEW) || ( (b(Y) == X) && (h(Y) ~= (h(X) + c(X,Y))) )
                        b(Y) = X; insert(Y, h(X)+c(X,Y));
                    else
                        if ( (b(Y) ~= X) && (h(Y) > (h(X) + c(X,Y))) )
                            insert(X, h(X));
                        else
                            if (b(Y) ~= X) && (h(X) > (h(Y) + c(Y,X))) && ...
                                    (t(Y) == CLOSED) && h(Y) > k_old
                                insert(Y, h(Y));

                            end
                        end
                    end
                 end
            end
            
            r = 0;
            return;
        end % process_state(0

    end % methods
end % classdef
