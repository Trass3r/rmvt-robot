function out = trprint(T, varargin)

    opt.fmt = '%g ';
    opt.mode = {'rpy', 'euler', 'angvec'};
    opt.radian = false;

    opt = tb_optparse(opt, varargin);

    s = '';

    for i=1:size(T,3)

        % for each 4x4 transform in a possible 3D matrix
        TT = T(:,:,i);

        % print the translational part if it exists
        if ~isrot(TT)
            st = sprintf('t=(%s), ', num2str(transl(TT)', opt.fmt));
        end

        % print the angular part in various representations
        switch (opt.mode)
        case {'rpy', 'euler'}
            % angle as a 3-vector
            if strcmp(opt.mode, 'rpy')
                ang = tr2rpy(TT);
            else
                ang = tr2eul(TT);
            end
            if opt.radian
                st = strcat(st, sprintf('R=(%s) rad\n', num2str(ang, opt.fmt)) );
             else
                st = strcat(st, sprintf('R=(%s) deg\n', num2str(ang*180.0/pi, opt.fmt)) );
             end
        case 'angvec'
            % as a vector and angle
            [th,v] = tr2angvec(TT);
            if opt.radian
                st = strcat(st, sprintf('R=(%sdeg | %s)\n', ...
                    num2str(th, opt.fmt), num2str(v, opt.fmt)) );
             else
                st = strcat(st, sprintf('R=(%sdeg | %s)\n', ...
                    num2str(th*180.0/pi, opt.fmt), num2str(v, opt.fmt)) );
             end
        end

        s = strvcat(s, st);
    end

    % if no output provided then display it
    if nargout == 0
        fprintf('%s\n', s);
    else
        out = s;
    end
