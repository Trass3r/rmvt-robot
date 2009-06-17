%LINK create a new LINK object
%
% A LINK object holds all information related to a robot link such as
% kinematics of the joint, rigid-body inertial parameters, motor and
% transmission parameters.
%
%	L = link
%	L = link(link)
%
% Create a default link, or a clone of the passed link.
%
%	L = link([alpha a theta d] [, CONVENTION])
% 	L = link([alpha a theta d sigma]) [, CONVENTION])
% 	L = link([alpha a theta d sigma offset] [, CONVENTION]
%
% If sigma or offset are not provided they default to zero.  Offset is a
% constant amount added to the joint angle variable before forward kinematics
% and is useful if you want the robot to adopt a 'sensible' pose for zero
% joint angle configuration.
%
% The optional CONVENTION argument is 'standard' for standard D&H parameters 
% or 'modified' for modified D&H parameters.  If not specified the default
% 'standard'.
%
% Object methods
%	L.A(q)         return link transform (A) matrix
%	L.islimit(q)   return vector of exceed flags (-1, 0, +1) corresponding to q
%
% Object properties (read/write)
%
%	L.alpha	       kinematic: link twist
%	L.a            kinematic: link twist
%	L.theta        kinematic: link twist
%	L.d            kinematic: link twist
%	L.sigma	       kinematic: 0 if revolute, 1 if prismatic
%	L.mdh	       kinematic 0 if standard D&H, else 1
%	L.offset	   kinematic: joint variable offset
%	L.qlim	       kinematic: joint variable limits [min max]
%
%	L.m            dynamic: link mass
%	L.r            dynamic: link COG wrt link coordinate frame 3x1
%	L.I            dynamic: link inertia matrix, symmetric 3x3, about link COG
%                      Maybe be assigned as:
%                         3x3 matrix (symmetry check enforced)
%                         3x1 vector [Ixx Iyy Izz]
%                         6x1 vector [Ixx Iyy Izz Ixy Iyz Ixz]
%	L.B            dynamic: link viscous friction (motor referred)
%	L.Tc           dynamic: link Coulomb friction [Tc_pos Tc_neg]
%                      Maybe be assigned as:
%                          2x1 vector
%                          scalar -> [Tc -Tc]
%
%	L.G            actuator: gear ratio
%	L.J            actuator: motor inertia (motor referred)
%
% Object properties (read only)
%
%	L.RP		return 'R' or 'P'
%	L.dh		return legacy DH row
%	L.dyn	return legacy DYN row
%
% For robot models prior to Toolbox release 5 (pre Matlab objects) the
% following object constructors are provided.
%
% 	L = LINK(DYN_ROW)		create from row of legacy DYN matrix
%	L = LINK(DYN_ROW, CONVENTION)	create from row of legacy DYN matrix
%

classdef link
    properties
        % kinematic parameters
        theta
        d
        alpha
        a
        sigma
        mdh
        offset
        name

        % dynamic parameters
        m
        r
        I
        Jm
        B

        Tc
        G
        qlim
    end

    properties (Dependent = true, SetAccess = private)
        dh
        RP
        dyn
    end


    methods
        function l = link(dh, convention)

            if nargin == 0,
                % create an 'empty' link
                l.alpha = 0;
                l.a = 0;
                l.theta = 0;
                l.d = 0;
                l.sigma = 0;
                l.mdh = 0;
                l.offset = 0;
                
                % it's a legacy DYN matrix
                l.m = [];
                l.r = [];
                v = [];
                l.I = [];
                l.Jm = [];
                l.G = [];
                l.B = 0;
                l.Tc = [0 0];
                l.qlim = [];

            elseif isa(dh, 'link')
                % clone passed link
                l = dh;
            elseif length(dh) <= 6,
                % legacy DH matrix
                % link([alpha A theta D sigma])

                if length(dh) < 4,
                        error('must provide <alpha A theta D> params');
                end
                l.alpha = dh(1);
                l.a = dh(2);
                l.theta = dh(3);
                l.d = dh(4);
                l.sigma = 0;
                if length(dh) >= 5,
                    l.sigma = dh(5);
                end
                if nargin > 1
                    if strncmp(convention, 'mod', 3) == 1,
                        l.mdh = 1;
                    elseif strncmp(convention, 'sta', 3) == 1,
                        l.mdh = 0;
                    else
                        error('convention must be modified or standard');
                    end
                else
                        l.mdh = 0;	% default to standard D&H
                end
                l.offset = 0;
                if length(dh) >= 6,
                    l.offset = dh(6);
                end

                % we know nothing about the dynamics
                l.m = [];
                l.r = [];
                v = [];
                l.I = [];
                l.Jm = [];
                l.G = [];
                l.B = 0;
                l.Tc = [0 0];
                l.qlim = [];

            else
                % legacy DYN matrix

                l.alpha = dh(1);
                l.a = dh(2);
                l.theta = dh(3);
                l.a = dh(4);
                if length(dh) == 4,
                    l.sigma = 0;
                else
                    l.sigma = dh(5);
                end
                l.mdh = 0;	% default to standard D&H
                l.offset = 0;
                
                % it's a legacy DYN matrix
                l.m = dh(6);
                l.r = dh(7:9)';		% a column vector
                v = dh(10:15);
                l.I = [	v(1) v(4) v(6)
                    v(4) v(2) v(5)
                    v(6) v(5) v(3)];
                if length(dh) > 15,
                    l.Jm = dh(16);
                end
                if length(dh) > 16,
                    l.G = dh(17);
                else
                    l.G = 1;
                end
                if length(dh) > 17,
                    l.B = dh(18);
                else
                    l.B = 0.0;
                end
                if length(dh) > 18,
                    l.Tc = dh(19:20);
                else
                    l.Tc = [0 0];
                end
                l.qlim = [];
                if nargin > 1
                    if strncmp(convention, 'mod', 3) == 1,
                        l.mdh = 1;
                    elseif strncmp(convention, 'sta', 3) == 1,
                        l.mdh = 0;
                    else
                        error('convention must be modified or standard');
                    end
                else
                        l.mdh = 0;	% default to standard D&H
                end
            end
        end % link()

        function  tau = friction(l, qd)
            tau = 0.0;

            qd = qd(:);
            tau = l.B * qd;

            tau = tau + (qd > 0) * l.Tc(1) + (qd < 0) * l.Tc(2);
        end % friction()

        %NOFRICTION blah
        function  l2 = nofriction(l, only)

            l2 = link(l);

            if (nargin == 2) & strcmpi(only(1:3), 'all'),
                l2.B = 0;
            end
            l2.Tc = [0 0];
        end

        function v = get.RP(l)
			if l.sigma == 0,
				v = 'R';
			else
				v = 'P';
			end
        end % RP()

		function v = get.dh(l)
			v = [l.alpha l.a l.theta l.d l.sigma];
        end % get.dh()

		function v = get.dyn(l)
            try
                v = [l.alpha l.a l.theta l.d l.sigma l.m l.r(:)' diag(l.I)' l.I(2,1) l.I(2,3) l.I(1,3) l.Jm l.G l.B l.Tc(:)'];
            catch
                error('some dynamic params not set');
            end
        end %  get.dyn()

        function l = set.Tc(l, v)
            if isempty(v),
                return;
            end
            if length(v) == 1,
                l.Tc = [v -v];
            elseif length(v) == 2,
                l.Tc = v;
            else
                error('Coulomb friction vector can have 1 (symmetric) or 2 (asymmetric) elements only')
            end
        end % set.Tc()

        function l = set.I(l, v)
            if isempty(v),
                return;
            end
            if all(size(v) == [3 3])
                if norm(v-v') > eps,
                    error('inertia matrix must be symmetric');
                end
                l.I = v;
            elseif length(v) == 3,
                l.I = diag(v);
            elseif length(v) == 6,
                l.I = [	v(1) v(4) v(6)
                    v(4) v(2) v(5)
                    v(6) v(5) v(3)];
            end
        end % set.I()

        function v = islimit(l, q)
			if isempty(l.qlim)
				error('no limits assigned to link')
			end
			v = (q > l.qlim(2)) - (q < l.qlim(1));
        end % islimit()

        function T = A(L, q)

            if L.mdh == 0,
                T = linktran([L.alpha L.a L.theta L.d L.sigma], ...
                    q+L.offset);
            else
                T = mlinktran([L.alpha L.a L.theta L.d L.sigma], ...
                    q+L.offset);
            end
        end % A()

        function display(l)
            disp(' ');
            disp([inputname(1), ' = '])
            disp(' ');
            disp( char(l) );
        end % display()

        function s = char(links)


            s = '';
            for j=1:length(links),
                l = links(j);
                if l.mdh == 0,
                    conv = 'std';
                else
                    conv = 'mod';
                end
                if l.sigma == 1,
                    js = sprintf('%11.4g %11.4g %11.4g %11s (%s)', l.alpha, l.a, l.theta, sprintf('q%d', j), conv);
                else
                    js = sprintf('%11.4g %11.4g %11s %11.4g (%s)', l.alpha, l.a, sprintf('q%d', j), l.d, conv);
                end
                s = strvcat(s, js);
            end
        end % char()

        function show(l)

            if length(l) > 1,
                for ll=l,
                    show(ll);
                end
                return;
            end

            display(l);
            if ~isempty(l.m)
                fprintf('  m    = %f\n', l.m)
            end
            if ~isempty(l.r)
                fprintf('  r    = %f %f %f\n', l.r);
            end
            if ~isempty(l.I)
                fprintf('  I    = | %f %f %f |\n', l.I(1,:));
                fprintf('         | %f %f %f |\n', l.I(2,:));
                fprintf('         | %f %f %f |\n', l.I(3,:));
            end
            if ~isempty(l.Jm)
                fprintf('  Jm   = %f\n', l.Jm);
            end
            if ~isempty(l.B)
                fprintf('  B    = %f\n', l.B);
            end
            if ~isempty(l.Tc)
                fprintf('  Tc   = %f(+) %f(-)\n', l.Tc(1), l.Tc(2));
            end
            if ~isempty(l.G)
                fprintf('  G    = %f\n', l.G);
            end
            if ~isempty(l.qlim)
                fprintf('  qlim = %f to %f\n', l.qlim(1), l.qlim(2));
            end
        end % show()


    end % methods
end % class


%LINKTRAN	Compute the link transform from kinematic parameters
%
%	LINKTRAN(alpha, an, theta, dn)
%	LINKTRAN(DH, q) is a homogeneous 
%	transformation between link coordinate frames.
%
%	alpha is the link twist angle
%	an is the link length
%	theta is the link rotation angle
%	dn is the link offset
%	sigma is 0 for a revolute joint, non-zero for prismatic
%
%	In the second case, q is substitued for theta or dn according to sigma.
%
%	Based on the standard Denavit and Hartenberg notation.

%	Copyright (C) Peter Corke 1993
function t = linktran(a, b, c, d)

	if nargin == 4,
		alpha = a;
		an = b;
		theta = c;
		dn = d;
	else
		if numcols(a) < 4,
			error('too few columns in DH matrix');
		end
		alpha = a(1);
		an = a(2);
		if numcols(a) > 4,
			if a(5) == 0,	% revolute
				theta = b;
				dn = a(4);
			else		% prismatic
				theta = a(3);
				dn = b;
			end
		else
			theta = b;	% assume revolute if sigma not given
			dn = a(4);
		end
	end
	sa = sin(alpha); ca = cos(alpha);
	st = sin(theta); ct = cos(theta);

	t =    [	ct	-st*ca	st*sa	an*ct
			st	ct*ca	-ct*sa	an*st
			0	sa	ca	dn
			0	0	0	1];
end

%MLINKTRAN	Compute the link transform from kinematic parameters
%
%	MLINKTRAN(alpha, an, theta, dn)
%	MLINKTRAN(DH, q) is a homogeneous 
%	transformation between link coordinate frames.
%
%	alpha is the link twist angle
%	an is the link length
%	theta is the link rotation angle
%	dn is the link offset
%	sigma is 0 for a revolute joint, non-zero for prismatic
%
%	In the second case, q is substitued for theta or dn according to sigma.
%
%	Based on the modified Denavit and Hartenberg notation.

%	Copyright (C) Peter Corke 1993
function t = mlinktran(a, b, c, d)

	if nargin == 4,
		alpha = a;
		an = b;
		theta = c;
		dn = d;
	else
		if numcols(a) < 4,
			error('too few columns in DH matrix');
		end
		alpha = a(1);
		an = a(2);
		if numcols(a) > 4,
			if a(5) == 0,	% revolute
				theta = b;
				dn = a(4);
			else		% prismatic
				theta = a(3);
				dn = b;
			end
		else
			theta = b;	% assume revolute if no sigma given
			dn = a(4);
		end
	end
	sa = sin(alpha); ca = cos(alpha);
	st = sin(theta); ct = cos(theta);

	t =    [	ct	-st	0	an
			st*ca	ct*ca	-sa	-sa*dn
			st*sa	ct*sa	ca	ca*dn
			0	0	0	1];
end

