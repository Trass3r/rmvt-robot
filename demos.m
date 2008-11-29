function demos
%DEMOS   Start the demo subsystem, add the Robot Toolbox to the path

tbpath = fileparts(which('fkine'));
addpath( fullfile(tbpath, 'demos') );
disp('Robotics Toolbox for Matlab (release 8)')

% startup the standard matlab thing
demo toolbox
