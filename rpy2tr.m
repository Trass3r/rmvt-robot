%RPY2TR Roll/pitch/yaw to homogenous transform
%
% 	TR = RPY2TR([R P Y])
%	TR = RPY2TR(R,P,Y)
%
% Returns a homogeneous tranformation for the specified roll/pitch/yaw angles.
% These correspond to rotations about the Z, X, Y axes respectively.
%
% See also: TR2RPY, EUL2TR

% $Log: not supported by cvs2svn $
% $Revision: 1.2 $
% Copyright (C) 1993-2002, by Peter I. Corke

function r = rpy2tr(roll, pitch, yaw)
        if length(roll) == 3,
                r = rotz(roll(1)) * roty(roll(2)) * rotx(roll(3));
        else
                r = rotz(roll) * roty(pitch) * rotx(yaw);
        end
