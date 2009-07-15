%LSPB  Linear segment with parabolic blend
%
% Compute a function, q(t), that varies smoothly from q0 to q1, using
% a constant velocity segment and parabolic blends (trapezoidal path).
%
% q = lspb(q0, q1, N)
%
%   Return path for N steps.
%
% q = lspb(q0, q1, t)
%
%   Return path over the time vector t.
%
% q = lspb(q0, q1, t, V)
%
%   Return path over the time vector t, but setting the velocity of the
%   linear segment.
%
% t = lspb(q0, q1, [], V)
%
%   Return the time to complete the segment at specified velocity. 

function [s,sd,sdd] = lspb(q0, q1, t, V)

	if size(t) == [1 1],
		t = [0:t-1]';
	end

	tf = max(t(:));

	if nargin < 4,
        % if velocity not specified, compute it
		V = (q1-q0)/tf * 1.5;
	else
		if V < (q1-q0)/tf,
			error('V too small\n');
		elseif V > 2*(q1-q0)/tf,
			error('V too big\n');
		end
    end

    if isempty(t)
        % return the time to complete, not the path
        return
    end
	tb = (q0 - q1 + V*tf)./V;
	a = V./tb;

    p = zeros(length(t), 1);
    pd = p;
    pdd = p;
    
	for i = 1:length(t),
		tt = t(i);

		if tt <= tb,
            % initial blend
			p(i) = q0 + a/2*tt^2;
            pd(i) = a*tt;
            pdd(i) = a;
		elseif tt <= (tf-tb),
            % linear motion
			p(i) = (q1+q0-V*tf)/2 + V*tt;
            pd(i) = V;
            pdd(i) = 0;
        else
            % final blend
			p(i) = q1 - a/2*tf^2 + a*tf*tt - a/2*tt^2;
            pd(i) = a*tf - a*tt;
            pdd(i) = -a;
		end
	end

    switch nargout
        case 0
            clf
            subplot(311)
            hold on
            k = t<= tb;
            plot(t(k), p(k), 'r-o');
            k = (t>=tb) & (t<= (tf-tb));
            plot(t(k), p(k), 'b-o');
            k = t>= (tf-tb);
            plot(t(k), p(k), 'g-o');
            grid; ylabel('s');
            hold off

            subplot(312)
            plot(t, pd); grid; ylabel('sd');
            
            subplot(313)
            plot(t, pdd); grid; ylabel('sdd');
            xlabel('time')
            shg
        case 1
            s = p;
        case 2
            s = p;
            sd = pd;
        case 3
            s = p;
            sd = pd;
            sdd = pdd;
    end
    
	if nargout == 0,
	else
		tg = p;
	end
