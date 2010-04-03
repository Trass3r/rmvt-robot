%CORIOLIS Compute the manipulator Coriolis matrix
%
% 	C = CORIOLIS(ROBOT, Q, QD)
%
% Returns the n element Coriolis/centripetal torque vector at the specified 
% pose and velocity.
% ROBOT is a robot object and describes the manipulator dynamics and 
% kinematics.
%
% If Q and QD are row vectors, CORIOLIS(ROBOT,Q,QD) is a row vector 
% of joint torques.
% If Q and QD are matrices, each row is interpretted as a joint state 
% vector, and CORIOLIS(ROBOT,Q,QD) is a matrix each row being the 
% corresponding joint %	torques.
%
% See also: ROBOT, RNE, ITORQUE, GRAVLOAD.


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

function C = coriolis(robot, q, qd)

    % we need to create a clone robot with no friciton, since friction
    % is also proportional to joint velocity
    robot2 = robot.nofriction('all');

    N = robot2.n;
    C = zeros(N,N);
    Csq = zeros(N,N);

    % find the torques that depend on a single finite joint speed,
    % these are due to the squared (centripetal) terms
    %
    %  set QD = [1 0 0 ...] then resulting torque is due to qd_1^2
    for j=1:N
        QD = zeros(1,N);
        QD(j) = 1;
        tau = robot2.rne(q, QD, zeros(size(q)), [0 0 0]');
        Csq(:,j) = Csq(:,j) + tau';
    end

    % find the torques that depend on a pair of finite joint speeds,
    % these are due to the product (Coridolis) terms
    %  set QD = [1 1 0 ...] then resulting torque is due to 
    %    qd_1 qd_2 + qd_1^2 + qd_2^2
    for j=1:N
        for k=j+1:N
            % find a product term  qd_j * qd_k
            QD = zeros(1,N);
            QD(j) = 1;
            QD(k) = 1;
            tau = robot2.rne(q, QD, zeros(size(q)), [0 0 0]');
            C(:,k) = C(:,k) + (tau' - Csq(:,k) - Csq(:,j)) * qd(j);
        end
    end
    C = C + Csq * diag(qd);
