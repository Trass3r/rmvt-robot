function world = makeworld(Nw)
    if nargin < 1
        Nw = 128;
    end

    world = zeros(Nw, Nw);

    imagesc(world);
    grid
    binary_cmap = [1 1 1; 1 0 0];
    colormap(binary_cmap);
    caxis([0 1]);
    figure(gcf)

    fprintf('makeworld:\n');
    fprintf('  left button, click and drag to create a rectangle\n');
    fprintf('  p - draw polygon\n');
    fprintf('  c - draw circle\n');
    fprintf('  e - erase map\n');
    fprintf('  u - undo last action\n');
    fprintf('  q - leave editing mode\n');


    while 1,
        drawnow

        k = waitforbuttonpress;
        if k == 1,
            % key pressed
            c = get(gcf, 'CurrentCharacter');
            switch c
                case 'p'
                    fprintf('click a sequence of points, <enter> when done')
                    title('click a sequence of points, <enter> when done')
                    xy = ginput;

                    fprintf('\r                                              \r')
                    title('')

                    xy = round(xy);
                    world_prev = world;
                    [X,Y] = meshgrid(1:Nw, 1:Nw);
                    world = world + inpolygon(X, Y, xy(:,1), xy(:,2));
                    
                case 'c',
                    fprintf('click a centre point')
                    title('click a centre point')
                    waitforbuttonpress;
                    point1 = get(gca,'CurrentPoint');    % button down detected
                    point1 = round(point1(1,1:2));       % extract x and y
                    hold on
                    h = plot(point1(1), point1(2), '+');
                    drawnow

                    fprintf('\rclick a circumference point')
                    title('click a circumference point')
                    waitforbuttonpress;
                    point2 = get(gca,'CurrentPoint');    % button up detected
                    point2 = round(point2(1,1:2));

                    fprintf('\r                             \r')
                    title('')
                    delete(h);
                    drawnow
                    hold off

                    r = round(norm2(point1-point2));
                    c = kcircle(r, 2*r+1);

                    world_prev = world;

                    % add the circle to the world
                    world(point1(2)-r:point1(2)+r,point1(1)-r:point1(1)+r) = ...
                    world(point1(2)-r:point1(2)+r,point1(1)-r:point1(1)+r) + c;
                    % ensure maximum value of occ.grid is 1
                    world = min(world, 1);

                case 'e'
                    world_prev = world;
                    world = zeros(Nw, Nw);

                case 'u'
                    if ~isempty(world_prev)
                        world = world_prev;
                    end
                    
                otherwise
                    break;      % key pressed
            end
        else
            % button pressed
            point1 = get(gca,'CurrentPoint');    % button down detected
            finalRect = rbbox;                   % return figure units
            point2 = get(gca,'CurrentPoint');    % button up detected

            point1 = round(point1(1,1:2));       % extract x and y
            point2 = round(point2(1,1:2));

            x1 = max(1, point1(1));
            x2 = min(Nw, point2(1));
            y1 = max(1, point1(2));
            y2 = min(Nw, point2(2));

            world_prev = world;
            world(y1:y2, x1:x2) = 1;
        end

        % ensure maximum value of occ.grid is 1
        world = min(world, 1);

        % update the world display
        set(gco, 'CData', world);
    end
