%RNE	Compute inverse dynamics via recursive Newton-Euler formulation
%
%	TAU = RNE(ROBOT, Q, QD, QDD)
%	TAU = RNE(ROBOT, [Q QD QDD])
%
%	Returns the joint torque required to achieve the specified joint position,
%	velocity and acceleration state.
%
%	Gravity vector is an attribute of the robot object but this may be 
%	overriden by providing a gravity acceleration	vector [gx gy gz].
%
%	TAU = RNE(ROBOT, Q, QD, QDD, GRAV)
%	TAU = RNE(ROBOT, [Q QD QDD], GRAV)
%
%	An external force/moment acting on the end of the manipulator may also be
%	specified by a 6-element vector [Fx Fy Fz Mx My Mz].
%
%	TAU = RNE(ROBOT, Q, QD, QDD, GRAV, FEXT)
%	TAU = RNE(ROBOT, [Q QD QDD], GRAV, FEXT)
%
%	where	Q, QD and QDD are row vectors of the manipulator state; pos, vel, and accel.
%
%	The torque computed also contains a contribution due to armature
%	inertia.
%
%	See also ROBOT, FROBOT, ACCEL, GRAVLOAD, INERTIA.
%
%	Should be a MEX file.

% MOD.HISTORY
%	12/01	changed into wrapper for DH or MDH conventions
%
%	Copyright (C) Peter Corke 1999

function tau = rne(robot, varargin)
	if robot.mdh == 0,
		tau = rne_dh(robot, varargin{:});
	else
		tau = rne_mdh(robot, varargin{:});
	end
