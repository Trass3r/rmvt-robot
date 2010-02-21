function tranimate(T, varargin)
    hg = trplot(eye(4,4), varargin{:});

    if size(T,3) > 1
        Ttraj = T;
    else
        if isrot(T)
            T = r2t(T);
        end
        Ttraj = ctraj(eye(4,4), T, 100);
    end

    for i=1:size(Ttraj,3)
        T = Ttraj(:,:,i);
        if isrot(T)
            T = r2t(T);
        end
        set(hg, 'Matrix', T);
        pause(0.05);
    end
