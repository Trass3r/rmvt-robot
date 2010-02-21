function eulplot(a,b,c)
    R = eye(3,3);
    trplot( R, 'unit', 'frame', '0', 'color', 'k');
    hold on
    R = R * rotz(a);
    trplot( R, 'unit', 'frame', '1', 'color', 'r');
    R = R * roty(b);
    trplot( R, 'unit', 'frame', '2', 'color', 'g');
    R = R * rotz(c);
    trplot( R, 'unit', 'frame', '3', 'color', 'b');
    hold off
