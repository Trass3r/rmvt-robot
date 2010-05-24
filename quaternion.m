%Quaternion Constructor for Quaternion objects
% 
%   Q = Quaternion               identitity quaternion 1<0,0,0>
%   Q = Quaternion(q)            from another quaternion
%
%   Q = Quaternion([s v1 v2 v3]) from 4 elements
%   Q = Quaternion(s)            from a scalar 
%   Q = Quaternion(v)            from a vector (pure quaternion)
%
%   Q = Quaternion(th, v)        unit quaternion from vector plus angle
%   Q = Quaternion(R)            unit quaternion from a 3x3 or 4x4 matrix
%
% A quaternion is a compact method of representing a 3D rotation that has
% computational advantages including speed and numerical robustness.
%
% A quaternion has 2 parts, a scalar s, and a vector v and is typically written
%
%   q = s <vx vy vz>
%
% A unit quaternion is one for which s^2+vx^2+vy^2+vz^2 = 1.
%
% A quaternion can be considered as a rotation about a vector in space where
%   q = cos (theta/2) sin(theta/2) <vx vy vz>
% where <vx vy vz> is a unit vector.
%
% Object methods
%
%   q.inv             return inverse of quaterion
%   q.norm            return norm of quaternion
%   q.unit            return unit quaternion
%   q.unitize         unitize this quaternin
%   q.plot            same options as trplot()
%   q.interp(q2, r)   interpolation (slerp) between q and q2, 0<=r<=1
%   q.scale(r)        interpolation (slerp) between identity and q, 0<=r<=1
%
% Object operators
%
%   Arithmetic operators are overloaded.
%
%   q+q2              return elementwise sum of quaternions
%   q-q2              return elementwise difference of quaternions
%   q*q2              return quaternion product
%   q*v               rotate vector by quaternion, v is 3x1
%   q/q2              return q*q2.inv
%   q^n               return q to power n (integer only)
%
% Object properties (read only)
%
%   q.s               real part
%   q.v               vector part
%   q.r               3x3 rotation matrix
%   q.t               4x4 homogeneous transform matrix
%   q.dot(w)          derivative of quaternion with angular velocity w

% TODO
%  constructor handles R, T trajectory and returns vector
%  .r, .t on a quaternion vector??

classdef Quaternion

    properties (Dependent = true)
        r   % rotation matrix
        t   % translation matrix
    end

    properties (SetAccess = private)
        s       % scalar part
        v       % vector part
    end

    methods

        function q = Quaternion(a1, a2)

            if nargin == 0,
                q.v = [0,0,0];
                q.s = 1;
            elseif isa(a1, 'Quaternion')
            %   Q = Quaternion(q)       from another quaternion
                q = a1;
            elseif nargin == 1
                if all(size(a1) == [1 4]) || all(size(a1) == [4 1])
            %   Q = Quaternion([s v1 v2 v3])    from 4 elements
                    a1 = a1(:);
                    q.s = a1(1);
                    q.v = a1(2:4)';
                elseif all(size(a1) == [3 3])
            %   Q = Quaternion(R)       from a 3x3 or 4x4 matrix
                    q = Quaternion( tr2q(a1) );
                elseif all(size(a1) == [4 4])
                    q = Quaternion( tr2q(a1(1:3,1:3)) );

                elseif length(a1) == 3,
            %   Q = Quaternion(v)       from a vector

                    q.s = 0;
                    q.v = a1(:)';
                elseif length(a1) == 1,
            %   Q = Quaternion(s)       from a scalar
                    q.s = a1(1);
                    q.v = [0 0 0];
                else
                    error('unknown dimension of input');
                end
            elseif nargin == 2
                if isvector(a1) && isscalar(a2),
                %   Q = Quaternion(theta, v)    from vector plus angle
                    q.s = cos(a1/2);
                    q.v = sin(a1/2)*unit(a2(:)');
                end
            end

        end


        %CHAR Create string representation of quaternion object
        function s = char(q)

            if length(q) > 1,
                s = '';
                for qq = q;
                    s = strvcat(s, char(qq));
                end
                return
            end
            s = [num2str(q.s), ' < ' ...
                num2str(q.v(1)) ', ' num2str(q.v(2)) ', '   num2str(q.v(3)) ' >'];
        end

        %DISPLAY Display the value of a quaternion object

        function display(q)

            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            if loose
                disp(' ');
            end
            disp(char(q))
            if loose
                disp(' ');
            end
        end

        %DOUBLE Convert a quaternion object to a 4-element vector

        function v = double(q)

            v = [q.s q.v];
        end

        %INV Invert a unit-quaternion
        %
        %   QI = inv(Q)
        %
        % Return the inverse of the unit-quaternion Q.
        %

        function qi = inv(q)

            qi = Quaternion([q.s -q.v]);
        end

        %UNIT Unitize a quaternion
        %
        %   QU = UNIT(Q)
        %
        % Returns a unit quaternion.
        function qu = unit(q)
            qu = q / norm(q);
        end

        %NORM Compute the norm of a quaternion
        %
        %   QN = norm(Q)
        %
        % Return a unit-quaternion corresponding to the quaternion Q.
        %
        function n = norm(q)

            n = norm(double(q));
        end


        %INTERP Interpolate rotations expressed by quaternion objects
        %
        %   QI = interp(Q1, Q2, R)
        %
        % Return a unit-quaternion that interpolates between Q1 and Q2 as R moves
        % from 0 to 1.  This is a spherical linear interpolation (slerp) that can
        % be interpretted as interpolation along a great circle arc on a sphere.
        %
        % If r is a vector, QI, is a vector of quaternions, each element
        % corresponding to sequential elements of R.
        %
        % See also: CTRAJ, quaternion.
        function q = interp(Q1, Q2, r)

            q1 = double(Q1);
            q2 = double(Q2);

            if any(r<0) || any(r>1),
                error('R out of range');
            end

            theta = acos(q1*q2');
            count = 1;

            if length(r) == 1,
                if theta == 0,
                    q = Q1;
                else
                    q = Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) );
                end
            else
                for R=r(:)',
                    if theta == 0,
                        qq = Q1;
                    else
                        qq = Quaternion( (sin((1-R)*theta) * q1 + sin(R*theta) * q2) / sin(theta) );
                    end
                    q(count) = qq;
                    count = count + 1;
                end
            end
        end
        
        
        %SCALE Interpolate rotations expressed by quaternion objects
        %
        %   QI = scale(Q1, R)
        %
        % Return a unit-quaternion that interpolates between identiy and Q1 as R moves
        % from 0 to 1.  This is a spherical linear interpolation (slerp) that can
        % be interpretted as interpolation along a great circle arc on a sphere.
        %
        % If r is a vector, QI, is a cell array of quaternions, each element
        % corresponding to sequential elements of R.
        %
        % See also: CTRAJ, quaternion.

        function q = scale(Q, r)

            q2 = double(Q);

            if any(r<0) || (r>1),
                error('r out of range');
            end
            q1 = [1 0 0 0];         % identity quaternion
            theta = acos(q1*q2');

            if length(r) == 1,
                if theta == 0,
                    q = Q;
                else
                    q = Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) ).unit;
                end
            else
                count = 1;
                for R=r(:)',
                    if theta == 0,
                        qq = Q;
                    else
                        qq = Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) ).unit;
                    end
                    q(count) = qq;
                    count = count + 1;
                end
            end
        end


        %PLUS Add two quaternion objects
        %
        % Invoked by the + operator
        %
        % q1+q2 standard quaternion addition
        function qp = plus(q1, q2)

            if isa(q1, 'Quaternion') & isa(q2, 'Quaternion')

                qp = Quaternion(double(q1) + double(q2));
            end
        end


        %MINUS Subtract two quaternion objects
        %
        % Invoked by the - operator
        %
        % q1-q2 standard quaternion subtraction
        function qp = minus(q1, q2)

            if isa(q1, 'Quaternion') & isa(q2, 'Quaternion')

                qp = Quaternion(double(q1) - double(q2));
            end
        end

        %MTIMES Multiply two quaternion objects
        %
        % Invoked by the * operator, handle two cases:
        %
        % q1*q2 standard quaternion multiplication
        % q1*v  rotate vector v by quaternion
        % q1*s  multiply vector v by scalar
        function qp = mtimes(q1, q2)

            if isa(q1, 'Quaternion') & isa(q2, 'Quaternion')
            %QQMUL  Multiply unit-quaternion by unit-quaternion
            %
            %   QQ = qqmul(Q1, Q2)
            %
            %   Return a product of unit-quaternions.
            %
            %   See also: TR2Q

            %   Copyright (C) 1993 Peter Corke

                % decompose into scalar and vector components
                s1 = q1.s;  v1 = q1.v;
                s2 = q2.s;  v2 = q2.v;

                % form the product
                qp = Quaternion([s1*s2-v1*v2' s1*v2+s2*v1+cross(v1,v2)]);

            elseif isa(q1, 'Quaternion') & isa(q2, 'double'),

            %QVMUL  Multiply vector by unit-quaternion
            %
            %   VT = qvmul(Q, V)
            %
            %   Rotate the vector V by the unit-quaternion Q.
            %
            %   See also: QQMUL, QINV

                if length(q2) == 3,
                    qp = q1 * Quaternion([0 q2(:)']) * inv(q1);
                    qp = qp.v(:);
                elseif length(q2) == 1,
                    qp = Quaternion( double(q1)*q2);
                else
                    error('quaternion-vector product: must be a 3-vector or scalar');
                end

            elseif isa(q2, 'Quaternion') & isa(q1, 'double'),
                if length(q1) == 3,
                    qp = q2 * Quaternion([0 q1(:)']) * inv(q2);
                    qp = qp.v;
                elseif length(q1) == 1,
                    qp = Quaternion( double(q2)*q1);
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
        % q1/q2     multiply one quaternion by inverse of the second.
        % q1/s      result is non-unit quaternion, all elements divided by s
        function qq = mrdivide(q1, q2)

            if isa(q2, 'Quaternion'),
                % qq = q1 / q2
                %    = q1 * qinv(q2)

                qq = q1 * inv(q2);
            elseif isa(q2, 'double'),
                qq = Quaternion( double(q1) / q2 );
            end
        end


        %PLOT Plot a quaternion object 
        %
        %   PLOT(Q)
        %
        % Display the quaternion as a rotated coordinate frame.
        %

        function plot(Q, varargin)
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

            trplot( Q.r, varargin{:});
            drawnow
        end

        function r = get.r(q)
            if length(q) > 1
                r = [];
                for Q=q
                    rr = t2r( q2tr(Q) );
                    r = cat(3, r, rr);
                end
            else
                r = t2r( q2tr(q) );
            end
        end

        function t = get.t(q)
            if length(q) > 1
                t = [];
                for Q=q
                    t = cat(3, t, q2tr(Q));
                end
            else
                t = q2tr(q);
            end
        end

        function qd = dot(q, omega)
            E = q.s*eye(3,3) - skew(q.v);
            omega = omega(:);
            qd = Quaternion(-0.5*q.v*omega, 0.5*E*omega);
        end
    end % methods
end % classdef

%TR2Q   Convert homogeneous transform to a unit-quaternion
%
%   Q = tr2q(T)
%
%   Return a unit quaternion corresponding to the rotational part of the
%   homogeneous transform T.
%
%   See also: Q2TR

%   Copyright (C) 1993 Peter Corke
function q = tr2q(t)
    qs = sqrt(trace(t)+1)/2.0;
    kx = t(3,2) - t(2,3);   % Oz - Ay
    ky = t(1,3) - t(3,1);   % Ax - Nz
    kz = t(2,1) - t(1,2);   % Ny - Ox

    if (t(1,1) >= t(2,2)) & (t(1,1) >= t(3,3)) 
        kx1 = t(1,1) - t(2,2) - t(3,3) + 1; % Nx - Oy - Az + 1
        ky1 = t(2,1) + t(1,2);          % Ny + Ox
        kz1 = t(3,1) + t(1,3);          % Nz + Ax
        add = (kx >= 0);
    elseif (t(2,2) >= t(3,3))
        kx1 = t(2,1) + t(1,2);          % Ny + Ox
        ky1 = t(2,2) - t(1,1) - t(3,3) + 1; % Oy - Nx - Az + 1
        kz1 = t(3,2) + t(2,3);          % Oz + Ay
        add = (ky >= 0);
    else
        kx1 = t(3,1) + t(1,3);          % Nz + Ax
        ky1 = t(3,2) + t(2,3);          % Oz + Ay
        kz1 = t(3,3) - t(1,1) - t(2,2) + 1; % Az - Nx - Oy + 1
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
        q = Quaternion([1 0 0 0]);
    else
        s = sqrt(1 - qs^2) / nm;
        qv = s*[kx ky kz];

        q = Quaternion([qs qv]);

    end
end


%Q2TR   Convert unit-quaternion to homogeneous transform
%
%   T = q2tr(Q)
%
%   Return the rotational homogeneous transform corresponding to the unit
%   quaternion Q.
%
%   See also: TR2Q

function t = q2tr(q)

    q = double(q);
    s = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    r = [   1-2*(y^2+z^2)   2*(x*y-s*z) 2*(x*z+s*y)
        2*(x*y+s*z) 1-2*(x^2+z^2)   2*(y*z-s*x)
        2*(x*z-s*y) 2*(y*z+s*x) 1-2*(x^2+y^2)   ];
    t = eye(4,4);
    t(1:3,1:3) = r;
    t(4,4) = 1;
end
