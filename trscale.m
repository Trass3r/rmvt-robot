%TRSCALE

function t = trscale(sx, sy, sz)

    if length(sx) > 1
        s = sx;
    else
        if nargin == 1,
            s = [sx sx sx];
        else
            s = [sx sy sz];
        end
    end
    t = [diag(s) [0 0 0]'; 0 0 0 1];
