%JACOB0 Compute manipulator Jacobian in world coordinates
%
%	J0 = JACOB0(ROBOT, Q)
%
% Returns a Jacobian matrix for the robot ROBOT in pose Q.
%
% The manipulator Jacobian matrix maps differential changes in joint space
% to differential Cartesian motion (world coord frame) of the end-effector.
% 		dX = J dQ
%
% For an n-axis manipulator the Jacobian is a 6 x n matrix.
%
%	J0_A = JACOB0(ROBOT, Q, ang)
%
%  Returns the analytical Jacobian where the angular rates are expressed in
% either RPY or Euler angle rates rather than angular velocity.  The string
% ang is either 'rpy' or 'eul'.
%
% See also: JACOBN, DIFF2TR, TR2DIFF.


% Copyright (C) 1999-2008, by Peter I. Corke
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

function J0 = jacob0(robot, q, ang)
	%
	%   dX_tn = Jn dq
	%
	Jn = jacobn(robot, q);	% Jacobian from joint to wrist space

	%
	%  convert to Jacobian in base coordinates
	%
	Tn = fkine(robot, q);	% end-effector transformation
	R = t2r(Tn);
	J0 = [R zeros(3,3); zeros(3,3) R] * Jn;

    if nargin == 3,
        switch ang,
        case 'rpy',
            rpy = tr2rpy( fkine(robot, q) );
            B = rpy2jac(rpy);
            if rcond(B) < eps,
                error('Representational singularity');
            end
            J0 = blkdiag( eye(3,3), inv(B) ) * J0;
        case 'eul'
            eul = tr2eul( fkine(robot, q) );
            B = eul2jac(eul);
            if rcond(B) < eps,
                error('Representational singularity');
            end
            J0 = blkdiag( eye(3,3), inv(B) ) * J0;
        end
    end
