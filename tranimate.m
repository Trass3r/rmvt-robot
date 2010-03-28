% tranimate(Ts)
% tranimate(Rs)
% tranimate(T)
% tranimate(T1, T2)
% tranimate(R)
% tranimate(R1, R2)
function tranimate(T2, varargin)
    if size(T2,3) > 1
        % tranimate(Ts)
        Ttraj = T2;
    else
        if isrot(T2)
            T2 = r2t(T2);
        end
        T1 = eye(4,4);

        % check if a second 4x4 or 3x3 was passed in
        if length(varargin) > 0
            if ishomog(varargin{1}) || isrot(varargin{1})
                T1 = T2;
                T2 = varargin{1};
                if isrot(T2)
                    T2 = r2t(T2);
                end
                varargin = varargin(2:end);
            end
        end
        Ttraj = ctraj(T1, T2, 100);
    end

    hg = trplot(T1, varargin{:});

    for i=1:size(Ttraj,3)
        T = Ttraj(:,:,i);
        if isrot(T)
            T = r2t(T);
        end
        set(hg, 'Matrix', T);
        pause(0.05);
    end
