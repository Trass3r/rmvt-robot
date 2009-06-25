function [s,sd,sdd] = tg1(q0, qf, t, qd0, qdf)

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
    

    tscal = max(t);
    t = t / tscal;  % normalized t
    
    A = 6*(qf - q0) - 3*(qdf+qd0)*tscal;
    B = -15*(qf - q0) + (8*qd0 + 7*qdf)*tscal;
    C = 10*(qf - q0) - (6*qd0 + 4*qdf)*tscal;
    D = 0;
    E = qd0*tscal; % as the t vector has been normalized
    F = q0;

    coeffs = [A B C D E F];

    p = polyval(coeffs, t);
    pd = polyval([5*A 4*B 3*C 2*D E], t) / tscal;
    pdd = polyval([20*A 12*B 6*C 2*D], t) / tscal^2;

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
