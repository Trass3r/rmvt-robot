%MINUS Subtract two quaternion objects
%
% Invoked by the - operator
%
% q1-q2	standard quaternion subtraction

function qp = plus(q1, q2)

	if isa(q1, 'quaternion') & isa(q2, 'quaternion')

		qp = quaternion(double(q1) - double(q2));
	end
