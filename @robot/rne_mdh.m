%MRNE	Compute inverse dynamics via recursive Newton-Euler formulation
%
%	TAU = MRNE(ROBOT, Q, QD, QDD)
%	TAU = MRNE(ROBOT, [Q QD QDD])
%
%	Returns the joint torque required to achieve the specified joint position,
%	velocity and acceleration state.
%
%	Gravity vector is an attribute of the robot object but this may be 
%	overriden by providing a gravity acceleration	vector [gx gy gz].
%
%	TAU = MRNE(ROBOT, Q, QD, QDD, GRAV)
%	TAU = MRNE(ROBOT, [Q QD QDD], GRAV)
%
%	An external force/moment acting on the end of the manipulator may also be
%	specified by a 6-element vector [Fx Fy Fz Mx My Mz].
%
%	TAU = MRNE(ROBOT, Q, QD, QDD, GRAV, FEXT)
%	TAU = MRNE(ROBOT, [Q QD QDD], GRAV, FEXT)
%
%	where	Q, QD and QDD are row vectors of the manipulator state; pos, vel, and accel.
%
%	The torque computed also contains a contribution due to armature
%	inertia.
%
%	See also ROBOT, FROBOT, ACCEL, GRAVLOAD, INERTIA.
%
%	Should be a MEX file.
%  Uses notation similar to "Introduction to Robotics", J.J. Craig.
%

%	Copyright (C) 1995 Peter Corke


% MOD.HISTORY
%       6/95    make use of passed in FEXT 
%       4/95    fix bug in use of passed FEXT 
%	11/96	bug for prismatic case 
%	3/99	uses objects
%
%	Copyright (C) Peter Corke 1999

function tau = rne_mdh(robot, a1, a2, a3, a4, a5)

	z0 = [0;0;1];
	grav = robot.gravity;	% default gravity from the object
	fext = zeros(6, 1);
	debug = 1;

	n = robot.n;
	if numcols(a1) == 3*n,
		Q = a1(:,1:n);
		Qd = a1(:,n+1:2*n);
		Qdd = a1(:,2*n+1:3*n);
		np = numrows(Q);
		if nargin >= 3,	
			grav = a2(:);
		end
		if nargin == 4,
			fext = a3;
		end
	else
		np = numrows(a1);
		Q = a1;
		Qd = a2;
		Qdd = a3;
		if numcols(a1) ~= n | numcols(Qd) ~= n | numcols(Qdd) ~= n | ...
			numrows(Qd) ~= np | numrows(Qdd) ~= np,
			error('bad data');
		end
		if nargin >= 5,	
			grav = a4(:);
		end
		if nargin == 6,
			fext = a5;
		end
	end
	
	tau = zeros(np,n);

	for p=1:np,
		q = Q(p,:)';
		qd = Qd(p,:)';
		qdd = Qdd(p,:)';
	
		Fm = [];
		Nm = [];
		pstarm = [];
		Rm = [];
		w = zeros(3,1);
		wd = zeros(3,1);
		v = zeros(3,1);
		vd = grav;

	%
	% init some variables, compute the link rotation matrices
	%
		for j=1:n,
			link = robot.link{j};
			Tj = link(q(j));
			Rm{j} = tr2rot(Tj);
			if link.RP == 'R',
				D = link.D;
			else
				D = q(j);
			end
			alpha = link.alpha;
			Pm(:,j) = [link.A; -D*sin(alpha); D*cos(alpha)];	% (i-1) P i
		end

	%
	%  the forward recursion
	%
		for j=1:n,
			link = robot.link{j};

			R = Rm{j}';	% transpose!!
			P = Pm(:,j);
			Pc = link.r;

			%
			% trailing underscore means new value
			%
			if link.RP == 'R',
				% revolute axis
				w_ = R*w + z0*qd(j);
				wd_ = R*wd + cross(R*w,z0*qd(j)) + z0*qdd(j);
				%v = cross(w,P) + R*v;
				vd_ = R * (cross(wd,P) + ...
					cross(w, cross(w,P)) + vd);

			else
				% prismatic axis
				w_ = R*w;
				wd_ = R*wd;
				%v = R*(z0*qd(j) + v) + cross(w,P);
				vd_ = R*(cross(wd,P) + ...
					cross(w, cross(w,P)) + vd ...
				      ) + 2*cross(R*w,z0*qd(j)) + z0*qdd(j);
			end
			% update variables
			w = w_;
			wd = wd_;
			vd = vd_;

			vdC = cross(wd,Pc) + ...
				cross(w,cross(w,Pc)) + vd;
			F = link.m*vdC;
			N = link.I*wd + cross(w,link.I*w);
			Fm = [Fm F];
			Nm = [Nm N];
			if debug,
				fprintf('w: '); fprintf('%.3f ', w)
				fprintf('\nwd: '); fprintf('%.3f ', wd)
				fprintf('\nvd: '); fprintf('%.3f ', vd)
				fprintf('\nvdbar: '); fprintf('%.3f ', vdC)
				fprintf('\n');
			end
		end

	%
	%  the backward recursion
	%

		f = fext(1:3);		% force/moments on end of arm
		nn = fext(4:6);

		for j=n:-1:1,
			
			%
			% order of these statements is important, since both
			% nn and f are functions of previous f.
			%
			link = robot.link{j};
			
			if j == n,
				R = eye(3,3);
				P = [0;0;0];
			else
				R = Rm{j+1};
				P = Pm(:,j+1);		% i/P/(i+1)
			end
			Pc = link.r;
			
			f_ = R*f + Fm(:,j);
			nn_ = Nm(:,j) + R*nn + cross(Pc,Fm(:,j)) + ...
				cross(P,R*f);
			
			f = f_;
			nn = nn_;

			if debug,
				fprintf('f: '); fprintf('%.3f ', f)
				fprintf('\nn: '); fprintf('%.3f ', nn)
				fprintf('\n');
			end
			if link.RP == 'R',
				% revolute
				tau(p,j) = nn'*z0 + ...
					link.G^2 * ( link.Jm*qdd(j) + ...
						friction(link, qd(j)) ...
					);
			else
				% prismatic
				tau(p,j) = f'*z0 + ...
					link.G^2 * ( link.Jm*qdd(j) + ...
						friction(link, qd(j)) ...
					);
			end
		end
	end
