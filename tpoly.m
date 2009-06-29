function [s,sd,sdd] = tpoly(q0, qf, t, qd0, qdf)

    if isscalar(t)
        t = [0:(t-1)]';
    else
        t = t(:);
    end
    if nargin < 4
        qd0 = 0;
    end
    if nargin < 5
        qdf = 0;
    end
    
    tf = max(t);
    
    % solve for the polynomial coefficients using least squares
    X = [
        0           0           0         0       0   1
        tf^5        tf^4        tf^3      tf^2    tf  1
        0           0           0         0       1   0
        5*tf^4      4*tf^3      3*tf^2    2*tf^2  tf  0
        0           0           0         1       0   0
        20*tf^3     12*tf^2     6*tf      2       0   0
    ];
    coeffs = (X \ [q0 qf qd0 qdf 0 0]')';

    % coefficients of derivatives 
    coeffs_d = [coeffs(1:5)] .* [5:-1:1];
    coeffs_dd = [coeffs_d(1:4)] .* [4:-1:1];

    % evaluate the polynomials
    p = polyval(coeffs, t);
    pd = polyval(coeffs_d, t);
    pdd = polyval(coeffs_dd, t);

    switch nargout
        case 0
            subplot(311)
            plot(t, p); grid; ylabel('s');
            subplot(312)
            plot(t, pd); grid; ylabel('sd');
            subplot(313)
            plot(t, pdd); grid;  ylabel('sdd');
            xlabel('time')
            shg
        case 1
            s = p;
        case 2
            s = p;
            sd = pd
        case 3
            s = p;
            sd = pd;
            sdd = pdd;
    end
