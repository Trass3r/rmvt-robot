%EUL2TR Convert Euler angles to homogeneous transformation
%
% 	TR = EUL2TR([PHI THETA PSI])
% 	TR = EUL2TR(PHI, THETA, PSI)
%
% Returns a homogeneous tranformation for the specified Euler angles.  These 
% correspond to rotations about the Z, X, Z axes respectively.
%
% See also: TR2EUL, RPY2TR

% $Log: not supported by cvs2svn $
% $Revision: 1.3 $
% Copyright (C) 1993-2002, by Peter I. Corke

function r = eul2tr(phi, theta, psi)
        if length(phi) == 3,
                r = rotz(phi(1)) * roty(phi(2)) * rotz(phi(3));
        else
                r = rotz(phi) * roty(theta) * rotz(psi);
        end
