classdef Bug2 < Navigation

    properties
        H       % hit points
        j       % number of hit points
        mline   % line from starting position to goal
        step    % state, in step 1 or step 2 of algorithm
        edge    % edge list
        k       % edge index
    end

    methods

        function bug = Bug2(world)

            % invoke the superclass constructor
            bug = bug@Navigation(world);


            bug.H = [];
            bug.j = 1;
            bug.step = 1;
        end

        function s = char(bug)
            s = sprintf('bug2 object: goal=%d,%d\n', bug.goal(1), bug.goal(2));
        end
        
        % null planning for the bug!
        function plan(bug, goal)
            bug.goal_set(goal(:)');
        end

        function navigate_init(bug, robot)

            % parameters of the M-line, direct from initial position to goal
            % as a vector mline, such that [robot 1]*mline = 0
            dims = axis;
            xmin = dims(1); xmax = dims(2);
            ymin = dims(3); ymax = dims(4);

            % create homogeneous representation of the line
            %  line*[x y 1]' = 0
            bug.mline = homline(robot(1), robot(2), ...
                bug.goal(1), bug.goal(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));
            if bug.mline(2) == 0
                % handle the case that the line is vertical
                plot([robot(1) robot(1)], [ymin ymax], 'k--');
            else
                x = [xmin xmax]';
                y = -[x [1;1]] * [bug.mline(1); bug.mline(3)] / bug.mline(2);
                plot(x, y, 'k--');
            end
        end

        function n = next(bug, robot)
            
            % these are coordinates (x,y)
          
            if bug.step == 1
                % Step 1.  Move along the M-line toward the goal

                if norm2(bug.goal - robot) == 0 % are we there yet?
                    n = [];
                    return
                end

                % motion on line toward goal
                d = bug.goal-robot;
                dx = sign(d(1));
                dy = sign(d(2));

                % detect if next step is an obstacle
                if bug.occgrid(robot(2)+dy, robot(1)+dx)
                    disp('Hit!');
                    bug.H(bug.j,:) = robot; % define hit point
                    bug.step = 2;
                    % get a list of all the points around the obstacle
                    bug.edge = edgelist(bug.occgrid==0, robot);
                    bug.k = 2;  % skip the first edge point, we are already there
                else
                    n = robot + [dx; dy];
                end
            end % step 1

            if bug.step == 2
                % Step 2.  Move around the obstacle until we reach a point
                % on the M-line closer than when we started.
                if norm2(bug.goal-robot) == 0 % are we there yet?
                    n = [];
                    return
                end

                if bug.k <= numrows(bug.edge)
                    n = bug.edge(bug.k,:)';  % next edge point
                else
                    % we are at the end of the list of edge points, we
                    % are back where we started.  Step 2.c test.
                    disp('robot is trapped')
                    n = [];
                    return;
                end

                % are we on the M-line now ?
                if abs( [robot' 1]*bug.mline') <= 0.5
                    disp('on the M-line');
                    % are closer than when we encountered the obstacle?
                    if norm2(robot-bug.goal) < norm2(bug.H(bug.j,:)'-bug.goal)
                        % back to moving along the M-line
                        disp('move on the m-line');
                        bug.j = bug.j + 1;
                        bug.step = 1;
                        return;
                    end
                end
                % no, keep going around
                %disp('keep moving around')
                bug.k = bug.k+1;
            end % step 2
        end % while
    end
end
