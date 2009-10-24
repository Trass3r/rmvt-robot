%plot_ellipse Plot an ellipse
%
%   plot_ellipse(x, P)
%   plot_ellipse(x, P, z)
%   plot_ellipse(x, P, z, hin)
%   plot_ellipse(x, P, z, hin, ls)
%
%   returns the handle to the line object for the ellipse.  The input argument
%   hin can be used to modify an existing ellipse.
%
%   ls is the standard line styles.

% figure a way to pass in a handle to use
% Credit Paul Newman
function h = plot_ellipse(x, P, nSigma, z, hin, varargin)

    if nargin < 5
        hin = [];
    end
    if nargin < 4
        z = 0;
    end
    if nargin < 3
        nSigma = 1;
    end

    % only plot x-y part
    P = P(1:2,1:2); 
    x = x(1:2);

    if(~any(diag(P)==0))
        [V,D] = eig(P);

        % make a circle of radius nSigma
        th = [0:0.1:2*pi];
        th = [th 0];
        y = nSigma*[cos(th);sin(th)];

        % warp it into the ellipse
        el = V*sqrtm(D)*y;
        % offset it
        el = el + repmat(x(:), 1, size(el,2));
%        el = [el el(:,1)] + repmat(x,1,size(el,2)+1);
        if ishandle(hin)
            set(hin, 'Xdata', line(el(1,:), 'Ydata', el(2,:), 'Zdata', z*ones(length(el(1,:)),1))) ;
        else
            holdon = ishold
            hold on

            h = plot3(el(1,:), el(2,:), z*ones(length(el(1,:)),1), varargin{:} );

            if ~holdon
                hold off
            end
        end
    end

