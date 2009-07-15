function jsingu(J)

    % convert to row-echelon form
    [R, jb] = rref(J);

    depcols = setdiff( [1:numcols(J)], jb);

    fprintf('%d linearly dependent joints:\n', length(depcols));
    for d=depcols
        fprintf('  q%d depends on: ', d)
        for k=find(R(:,d))
            fprintf('q%d ', k);
        end
        fprintf('\n');
    end
