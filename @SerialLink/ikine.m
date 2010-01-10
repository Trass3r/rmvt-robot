%IKINE Inverse manipulator kinematics
%
%	Q = IKINE(ROBOT, T)
%	Q = IKINE(ROBOT, T, Q)
%	Q = IKINE(ROBOT, T, Q, M)
%
% Returns the joint coordinates corresponding to the end-effector transform T.
% Note that the inverse kinematic solution is generally not unique, and 
% depends on the initial guess Q (which defaults to 0).
%
%	QT = IKINE(ROBOT, TG)
%	QT = IKINE(ROBOT, TG, Q)
%	QT = IKINE(ROBOT, TG, Q, M)
%
% Returns the joint coordinates corresponding to each of the transforms in 
% the 4x4xN trajectory TG.
% Returns one row of QT for each input transform.  The initial estimate 
% of QT for each time step is taken as the solution from the previous 
% time step.
%
% If the manipulator has fewer than 6 DOF then this method of solution
% will fail, since the solution space has more dimensions than can
% be spanned by the manipulator joint coordinates.  In such a case
% it is necessary to provide a mask matrix, M, which specifies the 
% Cartesian DOF (in the wrist coordinate frame) that will be ignored
% in reaching a solution.  The mask matrix has six elements that
% correspond to translation in X, Y and Z, and rotation about X, Y and
% Z respectively.  The value should be 0 (for ignore) or 1.  The number
% of non-zero elements should equal the number of manipulator DOF.
%
% Solution is computed iteratively using the pseudo-inverse of the
% manipulator Jacobian.
%
% Such a solution is completely general, though much less efficient 
% than specific inverse kinematic solutions derived symbolically.
% 
% This approach allows a solution to obtained at a singularity, but 
% the joint angles within the null space are arbitrarily assigned.
%
% For instance with a typical 5 DOF manipulator one would ignore
% rotation about the wrist axis, that is, M = [1 1 1 1 1 0].
%
%
% See also: FKINE, TR2DIFF, JACOB0, IKINE6S.
 
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

function qt = ikine(robot, tr, q, m, newopt)
	%  set default parameters for solution
	opt.ilimit = 1000;
	opt.tol = 1e-6;
    opt.debug = false;
    opt.lambda = 0.4;
    % 0.4 for Puma
    % 10 for leg
    opt.useInverse = false;

    % with no arguments, return the default parameters as a struct
    if nargin == 0
        qt = opt;
        return
    end

    % if a parameter struct is provided, use it instead
    if nargin == 5
        opt = newopt;
    end

    % OK, looks like we are doing real stuff
	n = robot.n;

    % check sanity of arguments
	if nargin < 3,
		q = zeros(n, 1);
	else
        if isempty(q)
            q = zeros(n, 1);
        else
            q = q(:);
        end
	end
	if nargin == 4,
		m = m(:);
		if length(m) ~= 6,
			error('Mask matrix should have 6 elements');
		end
		if length(find(m)) ~= robot.n 
			error('Mask matrix must have same number of 1s as robot DOF')
		end
	else
		if n < 6,
			error('For a manipulator with fewer than 6DOF a mask matrix argument must be specified');
		end
		m = ones(6, 1);
	end
		
    % make this a logical array so we can index with it
    m = logical(m);

    npoints = size(tr,3);    % number of points
    qt = zeros(npoints, n);  % preallocate space for results
    tcount = 0;              % total iteration count

    for i=1:npoints
        T = tr(:,:,i);
        nm = Inf;
        count = 0;
        while nm > opt.tol,
            e = tr2delta(fkine(robot, q'), T);

            % compute the Jacobian
            J = jacob0(robot, q);

            % error is based on the square sub-Jacobian
            if opt.useInverse
                dq = opt.lambda * pinv( J(m,:) ) * e(m);
            else
                dq = opt.lambda *  J(m,:)' * e(m);
            end

            if opt.debug
                fprintf('%d/%d: e =  ', i, count); disp(e')
                fprintf('      dq = '); disp(dq');
            end

            % update the estimated solution
            q = q + dq;
            nm = norm(dq);
            count = count+1;
            if count > opt.ilimit,
                fprintf('i=%d, nm=%f\n', i, nm);
                warning( sprintf('Solution wouldn''t converge, final error %.4g', nm) )
                break
            end
        end
        qt(i,:) = q';
        tcount = tcount + count;
    end
end


