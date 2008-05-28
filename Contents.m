% Robotics Toolbox.
% Version 7  April-2002
%
% What's new.
%   Readme      - New features and enhancements in this version.
%
% Homogeneous transformations
%   eul2tr      - Euler angle to transform 
%   oa2tr       - orientation and approach vector to transform 
%   rotx        - transform for rotation about X-axis 
%   roty        - transform for rotation about Y-axis 
%   rotz        - transform for rotation about Z-axis 
%   rpy2tr      - roll/pitch/yaw angles to transform 
%   tr2eul      - transform to Euler angles 
%   tr2rot      - transform to rotation submatrix
%   tr2rpy      - transform to roll/pitch/yaw angles
%   transl      - set or extract the translational component of a transform 
%   trnorm      - normalize a transform 
%   
% Quaternions
%   /           - divide quaternion by quaternion or scalar
%   *           - multiply quaternion by a quaternion or vector
%   inv         - invert a quaternion 
%   norm        - norm of a quaternion 
%   plot        - display a quaternion as a 3D rotation
%   qinterp     - interpolate quaternions
%   unit        - unitize a quaternion 
%
% Kinematics
%   diff2tr     - differential motion vector to transform 
%   fkine       - compute forward kinematics 
%   ikine       - compute inverse kinematics 
%   ikine560    - compute inverse kinematics for Puma 560 like arm
%   jacob0      - compute Jacobian in base coordinate frame
%   jacobn      - compute Jacobian in end-effector coordinate frame
%   tr2diff     - transform to differential motion vector 
%   tr2jac      - transform to Jacobian 
%   
% Dynamics
%   accel       - compute forward dynamics
%   cinertia    - compute Cartesian manipulator inertia matrix 
%   coriolis    - compute centripetal/coriolis torque 
%   friction    - joint friction
%   ftrans      - transform force/moment 
%   gravload    - compute gravity loading 
%   inertia     - compute manipulator inertia matrix 
%   itorque     - compute inertia torque 
%   nofriction  - remove friction from a robot object 
%   rne         - inverse dynamics 
%   
% Trajectory generation
%   ctraj       - Cartesian trajectory 
%   jtraj       - joint space trajectory 
%   trinterp    - interpolate transform s
%   
% Graphics
%   drivebot    - drive a graphical  robot 
%   plot        - plot/animate robot 
%   
% Other
%   manipblty   - compute manipulability 
%   unit        - unitize a vector
%
% Creation of robot models.
%   Fanuc10L    - Fanuc 10L (DH, kine)
%   link        - construct a robot link object 
%   MotomanHP6  - Motoman HP6 (DH, kine)
%   puma560     - Puma 560 data (DH, kine, dyn)
%   puma560akb  - Puma 560 data (MDH, kine, dyn)
%   robot       - construct a robot object 
%   stanford    - Stanford arm data (DH, kine, dyn)
%   S4ABB2p8    - ABB S4 2.8 (DH, kine)
%   twolink     - simple 2-link example (DH, kine)
%
% Demonstrations.
%   rtdemo      - toolbox demonstration
%   
% Copyright (C) 2008, by Peter I. Corke
