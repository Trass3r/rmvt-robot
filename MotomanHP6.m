%MotomanHP6  Load kinematic data for a Motoman HP6 manipulator
%
%	MotomanHP6
%
% Defines the object 'R' in the current workspace which describes the 
% kinematic characterstics of a Motoman HP6 manipulator
% using standard DH conventions.
%
% Also define the vector q0 which corresponds to the zero position.
%
%Wynand Swart
%Mega Robots CC, P/O Box 8412, Pretoria, 0001, South Africa
%Cell: 073-1555-430
%wynand.swart@gmail.com
%30 Sep 2007
%Motoman HP robot

%##########################################################
L(1) = link([ -pi/2   0.15     0   0            0]);
L(2) = link([ -pi     0.57     0   0            0]);
L(3) = link([ -pi/2   0.155    0   0            0]);
L(4) = link([  pi/2   0        0   -0.635       0]);
L(5) = link([ -pi/2   0        0   0            0]);
L(6) = link([  pi     0        0  -0.095        0]);
%##########################################################
%Pose 0; At ZERO position
%##########################################################
q0 =[0   -pi/2   0   0   -pi/2   0];
R=robot(L, 'name', 'Motoman HP6');
%##########################################################
