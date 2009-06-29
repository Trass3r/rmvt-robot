%MSTRAJ  Multisegment trajectory
%
%   traj = mstraj(segments, qdmax, q, dt, tacc)
%
% Create a multisegment trajectory based on via points and velocity/acceleration
% limits.  The path is linear segments with polynomial blends.
%
% The output is a trajectory MxN matrix, with one row per time step, and
% one column per axis.
%
%   segments is a NxM matrix of via points, 1 row per via point, one column per axis
%   qdmax is a row N-vector of axis velocity limits, or a column M-vector of segment times
%   q is a N-vector of initial axis coordinates
%   dt is the time step
%   tacc is the acceleration time.
%
%   traj = mstraj(segments, qdmax, q, dt, tacc, qd0, qdf)
%
% Optionally specify initial and final velocity.
%
% See also: CTRAJ.

% Copyright (C) 1993-2008, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

function [TG, taxis]  = mstraj(segments, qdmax, q, dt, Tacc, qd0, qdf)

    ns = numrows(segments);
    nj = numcols(segments);

    debug = false;

    if nargin < 6,
        qd0 = zeros(1, nj);
    end
    if nargin < 7,
        qdf = zeros(1, nj);
    end

    % set the initial conditions
    q_prev = q;
    qd_prev = qd0;

    clock = 0;      % keep track of time
    arrive = [];    % record planned time of arrival at via points

    tg = [];
    taxis = [];

    for seg=1:ns
        if debug
            fprintf('------------------- segment %d\n', seg);
        end

        % set the blend time, just half an interval for the first segment

        if length(Tacc) > 1
            tacc = Tacc(seg);
        else
            tacc = Tacc;
        end

        tacc = ceil(tacc/dt)*dt;
        tacc2 = ceil(tacc/2/dt) * dt;
        if seg == 1
            taccx = tacc2;
        else
            taccx = tacc;
        end

        % estimate travel time
        %    could better estimate distance travelled during the blend
        q_next = segments(seg,:);    % current target
        dq = q_next - q_prev;    % total distance to move this segment

        %% probably should iterate over the next section to get qb right...
        % while 1
        %   qd_next = (qnextnext - qnext)
        %   tb = abs(qd_next - qd) ./ qddmax;
        %   qb = f(tb, max acceleration)
        %   dq = q_next - q_prev - qb
        %   tl = abs(dq) ./ qdmax;

        if size(qdmax, 1) == 1
            % qdmax is specified, compute slowest axis

            qb = taccx * qdmax / 2;        % distance moved during blend
            tb = taccx;

            % convert to time
            tl = abs(dq) ./ qdmax;
            %tl = abs(dq - qb) ./ qdmax;
            tl = ceil(tl/dt) * dt;

            % find the total time and slowest axis
            tt = tb + tl;
            [tseg,slowest] = max(tt);
            taxis(seg,:) = tt;

            % best if there is some linear motion component
            if tseg <= 2*tacc
                tseg = 2 * tacc;
            end
        else
            % segment time specified, use that
            tseg = qdmax(seg);
        end

        % log the planned arrival time
        arrive(seg) = clock + tseg;
        if seg > 1
            arrive(seg) = arrive(seg) + tacc2;
        end

        if debug
            fprintf('seg %d, slowest axis %d, time required %.4g\n', ...
                seg, slowest, tseg);
        end

        %% create the trajectories for this segment

        % linear velocity from qprev to qnext
        qd = dq / tseg;

        % add the blend polynomial
        qb = jtraj(q, q_prev+tacc2*qd, [0:dt:taccx], qd_prev, qd);
        tg = [tg; qb(2:end,:)];

        clock = clock + taccx;     % update the clock

        % add the linear part, from tacc/2+dt to tseg-tacc/2
        for t=[tacc2+dt:dt:tseg-tacc2]
            s = t/tseg;
            q = (1-s) * q_prev + s * q_next;       % linear step
            tg = [tg; q];
            clock = clock + dt;
        end

        q_prev = q_next;    % next target becomes previous target
        qd_prev = qd;
    end
    % add the final blend
    qb = jtraj(q, q_next, [0:dt:tacc2], qd_prev, qdf);
    tg = [tg; qb(2:end,:)];

    % plot a graph if no output argument
    if nargout == 0
        t = [0:numrows(tg)-1]'*dt;
        plot(t, tg, '-o');
        hold on
        plot(arrive, segments, 'bo', 'MarkerFaceColor', 'k');
        hold off
        grid
        xlabel('time');
    else 
        TG = tg;
    end
