function [s,Sd,Sdd] = mtraj(tfunc, q0, qf, N)

    if ~isscalar(N)
        N = length(N);
    end
    if numcols(q0) ~= numcols(qf)
        error('must be same number of columns in q0 and qf')
    end

    s = zeros(N, numcols(q0));
    sd = zeros(N, numcols(q0));
    sdd = zeros(N, numcols(q0));

    for i=1:numcols(q0)
        % for each axis
        [s(:,i),sd(:,i),sdd(:,i)] = tfunc(q0(i), qf(i), N);
    end

    if nargout > 1
        Sd = sd;
    end
    if nargout > 2
        Sdd = sdd;
    end
