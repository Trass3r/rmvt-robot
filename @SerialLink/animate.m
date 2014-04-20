%SerialLink.animate   Update a robot animation
%
% R.animate(q) updates an existing animation for the robot R.  This will have
% been created using R.plot().
%
% Updates graphical instances of this robot in all figures.
%
% Notes::
% - Not a general purpose method, used for Simulink robot animation.
%
% See also SerialLink.plot.

function animate(robot, qq)

    if nargin < 3
        handles = findobj('Tag', robot.name);
    end
    
    links = robot.links;
    N = robot.n;
    
    % get handle of any existing graphical robots of same name
    %  one may have just been created above
    handles = findobj('Tag', robot.name);
    
    % MAIN DISPLAY/ANIMATION LOOP
    while true
        % animate over all instances of this robot in different axes
        
        for q=qq'  % for all configurations in trajectory
            q = q';
            for handle=handles'
%                 h = get(handle, 'UserData');
%                 h.q = q';
%                 set(handle, 'UserData', h);
                
                  group = findobj('Tag', robot.name);
                  h = get(group, 'UserData');
                  
                % now draw it for a pose q
                if robot.mdh
                    % modified DH case
                    T = robot.base;
                    vert = transl(T)';
                    
                    for L=1:N
                        set(h.link(L), 'Matrix', T);
                        
                        T = T * links(L).A(q(L));
                        vert = [vert; transl(T)']; % a stack of joint origin coordinates
                    end
                else
                    % standard DH case
                    T = robot.base;
                    vert = transl(T)';
                    
                    for L=1:N
                        % for all N+1 links
                        if robot.links(L).isprismatic()
                            %set(h.joint(j), 'Matrix', diag([1 1 q(j) 1])*T);
                            
                            set(h.pjoint(L), 'Matrix', T*trotz(q(L))*diag([1 1 q(L) 1]));
                        end
                        if h.link(L) ~= 0
                            set(h.link(L), 'Matrix', T);
                        end
                        
                        T = T * links(L).A(q(L));
                        vert = [vert; transl(T)'];
                    end
                    % update the transform for link N+1 (the tool)
                    if length(h.link) > N
                        set(h.link(N+1), 'Matrix', T);
                    end
                end
                
                % now draw the shadow
                if ~isempty(robot.tool)
                    t = transl(T*robot.tool);
                    vl = vert(end,:);
                    if t(1) ~= 0
                        vert = [vert; [t(1) vl(2) vl(3)]];
                    end
                    if t(2) ~= 0
                        vert = [vert; [t(1) t(2) vl(3)]];
                    end
                    if t(3) ~= 0
                        vert = [vert; t'];
                    end
                end
                if isfield(h, 'shadow')
                    set(h.shadow, 'Xdata', vert(:,1), 'Ydata', vert(:,2), 'Zdata', h.floorlevel*ones(size(vert(:,1))));
                end
                
                T = T * robot.tool;
                vert = [vert; transl(T)'];
                
                if ~isempty(h.wrist)
                    trplot(h.wrist, T);
                end
                
                % add a frame to the movie
                if ~isempty(h.robot.framenum)
                    % write the frame to the movie folder
                    print( '-dpng', fullfile(h.robot.moviepath, sprintf('%04d.png', h.robot.framenum)) );
                    h.robot.framenum = h.robot.framenum+1;
                end
                
                if h.robot.delay > 0
                    pause(h.robot.delay);
                    drawnow
                end
            end
        end
        
        if ~h.robot.loop
            break;
        end        
    end
    
    h.q = q;
    set(group, 'UserData', h);