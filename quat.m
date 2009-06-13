classdef quaternion

    properties (Dependent = true)
        r
        t
        d
    end

    properties
        s       % scalar part
        v       % vector part
        data    % used only for a quaternion array.  4xNxM
    end

    methods

        %QUATERNION Constructor for quaternion objects
        % 
        %	Q = QUATERNION([s v1 v2 v3])	from 4 elements
        %	Q = QUATERNION([s v1 v2 v3])	from 4 elements
        %	Q = QUATERNION(v, theta)	from vector plus angle
        %	Q = QUATERNION(R)		from a 3x3 or 4x4 matrix
        %	Q = QUATERNION(q)		from another quaternion
        %	Q = QUATERNION(s)		from a scalar
        %	Q = QUATERNION(v)		from a vector
        %
        % All versions, except the first, are guaranteed to return a unit quaternion.
        %
        % A quaternion is a compact method of representing a 3D rotation that has
        % computational advantages including speed and numerical robustness.
        %
        % A quaternion has 2 parts, a scalar s, and a vector v and is typically written
        %
        %	q = s <vx vy vz>
        %
        % A unit quaternion is one for which s^2+vx^2+vy^2+vz^2 = 1.
        %
        % A quaternion can be considered as a rotation about a vector in space where
        %	q = cos (theta/2) sin(theta/2) <vx vy vz>
        % where <vx vy vz> is a unit vector.
        %
        % Various functions such as INV, NORM, UNIT and PLOT are overloaded for
        % quaternion objects.
        %
        % Arithmetic operators are also overloaded to allow quaternion multiplication,
        % division, exponentiaton, and quaternion-vector multiplication (rotation).
        %
        % SEE ALSO: QUATERNION/SUBSREF, QUATERNION/PLOT

        function q = quaternion(a1, a2)

            'constructor'
            if nargin == 0,
                q.v = [0,0,0];
                q.s = 1;
            elseif isa(a1, 'quaternion')
        %	Q = QUATERNION(q)		from another quaternion
                q = a1;
            elseif nargin == 1
                if all(size(a1) == [1 4]) | all(size(a1) == [4 1])
        %	Q = QUATERNION([s v1 v2 v3])	from 4 elements
                    a1 = a1(:);
                    q.s = a1(1);
                    q.v = a1(2:4)';
                elseif all(size(a1) == [3 3])
        %	Q = QUATERNION(R)		from a 3x3 or 4x4 matrix
                    q = quaternion( tr2q(a1) );
                elseif all(size(a1) == [4 4])
                    q = quaternion( tr2q(a1(1:3,1:3)) );

                elseif length(a1) == 3,
        %	Q = QUATERNION(v)		from a vector

                    q.s = 0;
                    q.v = a1(:)';
                elseif length(a1) == 1,
        %	Q = QUATERNION(s)		from a scalar
                    q.s = a1(1);
                    q.v = [0 0 0];
                else
                    error('unknown dimension of input');
                end
            elseif nargin == 2
                q.s = a1;
                q.v = a2;
        %	Q = QUATERNION(v, theta)	from vector plus angle
                %q = unit( quaternion( [cos(a2/2) sin(a2/2)*unit(a1(:).')]) );
            end

        end


        %CHAR Create string representation of quaternion object
        function s = char(q)

            s = [num2str(q.s), ' <' num2str(q.v(1)) ', ' num2str(q.v(2)) ', '   num2str(q.v(3)) '>'];
        end

        %DISPLAY Display the value of a quaternion object

        function display(q)

            disp(' ');
            disp([inputname(1), ' = '])
            disp(' ');
            disp(['  ' char(q)])
            disp(' ');
        end

        %DOUBLE Convert a quaternion object to a 4-element vector

        function v = double(q)

            v = [q.s q.v];
        end

        %INV Invert a unit-quaternion
        %
        %	QI = inv(Q)
        %
        % Return the inverse of the unit-quaternion Q.
        %

        function qi = inv(q)

            qi = quaternion([q.s -q.v]);
        end

        %UNIT Unitize a quaternion
        %
        %	QU = UNIT(Q)
        %
        % Returns a unit quaternion.
        function qu = unit(q)
            qu = q / norm(q);
        end

        %NORM Compute the norm of a quaternion
        %
        %	QN = norm(Q)
        %
        % Return a unit-quaternion corresponding to the quaternion Q.
        %
        function n = norm(q)

            n = norm(double(q));
        end


        %QINTERP Interpolate rotations expressed by quaternion objects
        %
        %	QI = qinterp(Q1, Q2, R)
        %
        % Return a unit-quaternion that interpolates between Q1 and Q2 as R moves
        % from 0 to 1.  This is a spherical linear interpolation (slerp) that can
        % be interpretted as interpolation along a great circle arc on a sphere.
        %
        % If r is a vector, QI, is a cell array of quaternions, each element
        % corresponding to sequential elements of R.
        %
        % See also: CTRAJ, QUATERNION.
        function q = qinterp(Q1, Q2, r)

            q1 = double(Q1);
            q2 = double(Q2);

            if (r<0) | (r>1),
                error('R out of range');
            end

            theta = acos(q1*q2');
            q = {};
            count = 1;

            if length(r) == 1,
                if theta == 0,
                    q = Q1;
                else
                    q = quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) );
                end
            else
                for R=r(:)',
                    if theta == 0,
                        qq = Q1;
                    else
                        qq = quaternion( (sin((1-R)*theta) * q1 + sin(R*theta) * q2) / sin(theta) );
                    end
                    q{count} = qq;
                    count = count + 1;
                end
            end
        end


        %PLUS Add two quaternion objects
        %
        % Invoked by the + operator
        %
        % q1+q2	standard quaternion addition
        function qp = plus(q1, q2)

            if isa(q1, 'quaternion') & isa(q2, 'quaternion')

                qp = quaternion(double(q1) + double(q2));
            end
        end


        %MINUS Subtract two quaternion objects
        %
        % Invoked by the - operator
        %
        % q1-q2	standard quaternion subtraction
        function qp = minus(q1, q2)

            if isa(q1, 'quaternion') & isa(q2, 'quaternion')

                qp = quaternion(double(q1) - double(q2));
            end
        end

        %MTIMES Multiply two quaternion objects
        %
        % Invoked by the * operator, handle two cases:
        %
        % q1*q2	standard quaternion multiplication
        % q1*v	rotate vector v by quaternion
        % q1*s	multiply vector v by scalar
        function qp = mtimes(q1, q2)

            if isa(q1, 'quaternion') & isa(q2, 'quaternion')
            %QQMUL	Multiply unit-quaternion by unit-quaternion
            %
            %	QQ = qqmul(Q1, Q2)
            %
            %	Return a product of unit-quaternions.
            %
            %	See also: TR2Q

            %	Copyright (C) 1993 Peter Corke

                % decompose into scalar and vector components
                s1 = q1.s;	v1 = q1.v;
                s2 = q2.s;	v2 = q2.v;

                % form the product
                qp = quaternion([s1*s2-v1*v2' s1*v2+s2*v1+cross(v1,v2)]);

            elseif isa(q1, 'quaternion') & isa(q2, 'double'),

            %QVMUL	Multiply vector by unit-quaternion
            %
            %	VT = qvmul(Q, V)
            %
            %	Rotate the vector V by the unit-quaternion Q.
            %
            %	See also: QQMUL, QINV

                if length(q2) == 3,
                    qp = q1 * quaternion([0 q2(:)']) * inv(q1);
                    qp = qp.v;
                elseif length(q2) == 1,
                    qp = quaternion( double(q1)*q2);
                else
                    error('quaternion-vector product: must be a 3-vector or scalar');
                end

            elseif isa(q2, 'quaternion') & isa(q1, 'double'),
                if length(q1) == 3,
                    qp = q2 * quaternion([0 q1(:)']) * inv(q2);
                    qp = qp.v;
                elseif length(q1) == 1,
                    qp = quaternion( double(q2)*q1);
                else
                    error('quaternion-vector product: must be a 3-vector or scalar');
                end
            end
        end

        %MPOWER Raise quaternion to integer power
        %
        % Compound the quaternion with itself.  Invoked by means of the caret
        % operator.
        function qp = mpower(q, p)

            % check that exponent is an integer
            if (p - floor(p)) ~= 0,
                error('quaternion exponent must be integer');
            end

            qp = q;

            % multiply by itself so many times
            for i = 2:abs(p),
                qp = qp * q;
            end

            % if exponent was negative, invert it
            if p<0,
                qp = inv(qp);
            end
        end

        %MRDIVIDE Compute quaternion quotient.
        %
        % Invoked on the / operator, handle two cases:
        % q1/q2  	multiply one quaternion by inverse of the second.
        % q1/s		result is non-unit quaternion, all elements divided by s
        function qq = mrdivide(q1, q2)

            if isa(q2, 'quaternion'),
                % qq = q1 / q2
                %    = q1 * qinv(q2)

                qq = q1 * inv(q2);
            elseif isa(q2, 'double'),
                qq = quaternion( double(q1) / q2 );
            end
        end


        %PLOT Plot a quaternion object 
        %
        %	PLOT(Q)
        %
        % Display the quaternion as a rotated coordinate frame.
        %

        function plot(Q, off, fmt, color)
            %axis([-1 1 -1 1 -1 1])

            if nargin < 2,
                off = [0 0 0];
            end
            if nargin < 3,
                fmt = '%c';
            end
            if nargin < 4,
                color = 'b';
            end
            % create unit vectors
            o = [0 0 0]';
            x1 = Q*[1 0 0]';
            y1 = Q*[0 1 0]';
            z1 = Q*[0 0 1]';

            get(gca, 'Tag')
            if strcmp(get(gca, 'Tag'), 'trplot') == 0,
                fprintf('No tag\n');
                clf
                axes
                set(gca, 'Tag', 'trplot')
                fprintf('set tag\n');
                xlabel( 'X');
                ylabel( 'Y');
                zlabel( 'Z');
            end
            ih = ishold;
            hold on
            plot3([0;x1(1)]+off(1), [0; x1(2)]+off(2), [0; x1(3)]+off(3), color);
            h = text(off(1)+x1(1), off(2)+x1(2), off(3)+x1(3), sprintf(fmt, 'X'));
            set(h, 'Color', color);

            plot3([0;y1(1)]+off(1), [0; y1(2)]+off(2), [0; y1(3)]+off(3), color);
            h = text(off(1)+y1(1), off(2)+y1(2), off(3)+y1(3), sprintf(fmt, 'Y'));
            set(h, 'Color', color);

            plot3([0;z1(1)]+off(1), [0; z1(2)]+off(2), [0; z1(3)]+off(3), color);
            h = text(off(1)+z1(1), off(2)+z1(2), off(3)+z1(3), sprintf(fmt, 'Z'));
            set(h, 'Color', color);
            grid on
            if ~ishold,
                hold off
            end
            axis equal
        end

        %TR2Q	Convert homogeneous transform to a unit-quaternion
        %
        %	Q = tr2q(T)
        %
        %	Return a unit quaternion corresponding to the rotational part of the
        %	homogeneous transform T.
        %
        %	See also: Q2TR

        %	Copyright (C) 1993 Peter Corke
        function q = tr2q(t)
            qs = sqrt(trace(t)+1)/2.0;
            kx = t(3,2) - t(2,3);	% Oz - Ay
            ky = t(1,3) - t(3,1);	% Ax - Nz
            kz = t(2,1) - t(1,2);	% Ny - Ox

            if (t(1,1) >= t(2,2)) & (t(1,1) >= t(3,3)) 
                kx1 = t(1,1) - t(2,2) - t(3,3) + 1;	% Nx - Oy - Az + 1
                ky1 = t(2,1) + t(1,2);			% Ny + Ox
                kz1 = t(3,1) + t(1,3);			% Nz + Ax
                add = (kx >= 0);
            elseif (t(2,2) >= t(3,3))
                kx1 = t(2,1) + t(1,2);			% Ny + Ox
                ky1 = t(2,2) - t(1,1) - t(3,3) + 1;	% Oy - Nx - Az + 1
                kz1 = t(3,2) + t(2,3);			% Oz + Ay
                add = (ky >= 0);
            else
                kx1 = t(3,1) + t(1,3);			% Nz + Ax
                ky1 = t(3,2) + t(2,3);			% Oz + Ay
                kz1 = t(3,3) - t(1,1) - t(2,2) + 1;	% Az - Nx - Oy + 1
                add = (kz >= 0);
            end

            if add
                kx = kx + kx1;
                ky = ky + ky1;
                kz = kz + kz1;
            else
                kx = kx - kx1;
                ky = ky - ky1;
                kz = kz - kz1;
            end
            nm = norm([kx ky kz]);
            if nm == 0,
                q = quaternion([1 0 0 0]);
            else
                s = sqrt(1 - qs^2) / nm;
                qv = s*[kx ky kz];

                q = quaternion([qs qv]);

            end
        end


        %Q2TR	Convert unit-quaternion to homogeneous transform
        %
        %	T = q2tr(Q)
        %
        %	Return the rotational homogeneous transform corresponding to the unit
        %	quaternion Q.
        %
        %	See also: TR2Q

        function t = q2tr(q)

            q = double(q);
            s = q(1);
            x = q(2);
            y = q(3);
            z = q(4);

            r = [	1-2*(y^2+z^2)	2*(x*y-s*z)	2*(x*z+s*y)
                2*(x*y+s*z)	1-2*(x^2+z^2)	2*(y*z-s*x)
                2*(x*z-s*y)	2*(y*z+s*x)	1-2*(x^2+y^2)	];
            t = eye(4,4);
            t(1:3,1:3) = r;
            t(4,4) = 1;
        end

        function ind = subsindex(q)
            fprintf('subsindex');
            ind = 0;
        end

        function qs = subsref(q, S)
            fprintf('subsref: %s', S.type);
            if S.type == '()',
                i = S.subs{1};
                i
                if i > numrows(q.s),
                    error('subscript out of bounds');
                end
                qs = quaternion([ q.s(i) q.v(i,:)]);
            else
                error('bad quaternion subscript reference')
            end
        end

        function x = subsasgn(q, S, b)
            if S.type == '()',
                i = S.subs{1};
                i
                b.s
                b.v
                q.s(i) = b.s;
                q.v(i,:) = b.v;
                x = quaternion(q.s, q.v);
            else
                error('bad quaternion assignment')
            end
        end

        function s = get.s(q)
            s = q.s;
        end

        function v = get.v(q)
            v = q.v;
        end

        function d = get.d(q)
            d = double(q);
        end

        function r = get.r(q)
            r = t2r( q2tr(q) );
        end

        function t = get.t(q)
            t = q2tr(q);
        end
    end % methods
end % classdef
