function tg = mtraj(tfunc, q0, qf, N)

    if numcols(q0) ~= numcols(qf)
        error('must be same number of columns in q0 and qf')
    end

    tg = zeros(N, numcols(q0));

    for i=1:numcols(q0)
        % for each axis
        tg(:,i) = tfunc(q0(i), qf(i), N);
    end
