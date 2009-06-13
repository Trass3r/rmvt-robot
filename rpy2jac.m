%RPY2JAC Compute the Jacobian from RPY angle rates to angular velocity
%
%	J = RPY2JAC(RPY)
%	J = RPY2JAC(R, P, Y)
%
% Returns a 3x3 Jacobian matrix to map roll-pitch-yaw rates to angular
% velocity.  Used in the creation of the analytical Jacobian.
%
% See also: EUL2JAC, JACOB0, JACOBN

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

function J = rpy2jac(r, p, y)

    if length(r) == 3,
        p = r(2);
        y = r(3);
        r = r(1);
    end
	J = [	
        1  0       sin(p)
        0  cos(r)  -cos(p)*sin(r)
        0  sin(r)  cos(p)*cos(r)
        ];
		
