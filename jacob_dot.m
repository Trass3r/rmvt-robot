%JACOBN Compute manipulator Jacobian in end-effector frame
%
%	JN = JACOBN(ROBOT, Q)
%
% Returns a Jacobian matrix for the robot ROBOT in pose Q.
%
% The manipulator Jacobian matrix maps differential changes in joint space
% to differential Cartesian motion of the end-effector (end-effector coords).
% 		dX = J dQ
%
% This function uses the technique of
% 	Paul, Shimano, Mayer
% 	Differential Kinematic Control Equations for Simple Manipulators
% 	IEEE SMC 11(6) 1981
% 	pp. 456-460
%
% For an n-axis manipulator the Jacobian is a 6 x n matrix.
%
% See also: JACOB0, DIFF2TR, TR2DIFF

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

function Jdot = jacob2(robot, q, qd)

	n = robot.n;
	L = robot.link;		% get the links

    Tj = robot.base;     % this is cumulative transform from base
    w = [0;0;0];
    v = [0;0;0];
    for j=1:n,
        link = robot.link{j};
        Aj = link(q(j));
        Tj = Tj * Aj;
        R{j} = t2r(Aj);
        T{j} = t2r(Tj);
        p(:,j) = transl(Tj);    % origin of link j

        if j>1,
            z(:,j) = T{j-1} * [0 0 1]'; % in world frame
            %w = R{j}'*( w + z(:,j) * qd(j));
            %v = v + cross(R{j}*w, R{j-1}*p(:,j));
            v = R{j-1}'*v + cross(w, p(:,j));
            w = R{j-1}'* w + z(:,j) * qd(j);
        else
            z(:,j) = [0 0 1]';
            v = [0 0 0]';
            w = [0 0 0]';
        end

        vel(:,j) = v;       % velocity of origin of link j

        omega(:,j) = w;         % omega of link j in link j frame
    end

    J = [];
    Jdot = [];
    for j=1:n,
        if j>1,
            t = p(:,n) - p(:,j-1);
            td = vel(:,n) - vel(:,j-1);
        else
            t = p(:,n);
            td = vel(:,n);
        end
        if L{j}.RP == 'R',
            J_col = [cross(z(:,j), t); z(:,j)];
            Jdot_col = [cross(cross(omega(:,j), z(:,j)), t) + cross(z(:,j), td) ; cross(omega(:,j),  z(:,j))];
        else
            J_col = [z(:,j); 0;0;0];
            Jdot_col = zeros(6,1);
        end

        J = [J J_col];
        Jdot = [Jdot Jdot_col];

	end
