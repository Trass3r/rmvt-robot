%CODEGENERATOR.GENMFUNFDYN Generates M-functions for the forward dynamics
%
% cGen.genmfunfdyn()
%
% Notes::
% - Is called by CodeGenerator.genfdyn if cGen has active flag genmfun
% - The generated M-function is composed of previously generated M-functions
% for the inertia matrix, coriolis matrix, vector of gravitational load and joint friction vector.
% This function recombines these components to compute the forward dynamics.
% - Access to generated functions is provided via 
% subclass of SerialLink stored in cGen.robjpath
%
% Authors::
%  J�rn Malzahn
%  2012 RST, Technische Universit�t Dortmund, Germany
%  http://www.rst.e-technik.tu-dortmund.de
%
% See also CodeGenerator, geninvdyn

% Copyright (C) 1993-2012, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
%
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
%
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function [] = genmfunfdyn(CGen)

%% Does robot class exist?
if ~exist(fullfile(CGen.robjpath,[CGen.getrobfname,'.m']),'file')
    CGen.logmsg([datestr(now),'\tCreating ',CGen.getrobfname,' m-constructor ']);
    CGen.createmconstructor;
    CGen.logmsg('\t%s\n',' done!');
end

%%

CGen.logmsg([datestr(now),'\tGenerating m-function for the joint inertial reaction forces/torques' ]);

symname = 'Iqdd';
fname = fullfile(CGen.sympath,[symname,'.mat']);

if exist(fname,'file')
    tmpStruct = load(fname);
else
    error ('genmfunfdyn:SymbolicsNotFound','Save symbolic expressions to disk first!')
end

funfilename = fullfile(CGen.robjpath,[symname,'.m']);
[q qd] = CGen.rob.gencoords;
tau = CGen.rob.genforces;

matlabFunction(tmpStruct.(symname),'file',funfilename,...              % generate function m-file
    'outputs', {'Iacc'},...
    'vars', {'rob',q,qd,tau});
hStruct = createHeaderStructIqdd(CGen.rob,symname);                 % replace autogenerated function header
replaceheader(CGen,hStruct,funfilename);
CGen.logmsg('\t%s\n',' done!');

%% Generate mfunction for the acceleration
CGen.logmsg([datestr(now),'\tGenerating joint acceleration m-function:']);

funfilename = fullfile(CGen.robjpath,'accel.m');
hStruct = createHeaderStructAccel(CGen.rob,funfilename);

fid = fopen(funfilename,'w+');

fprintf(fid, '%s\n', ['function qdd = accel(rob,q,qd,tau)']);                 % Function definition
fprintf(fid, '%s\n',constructheaderstring(CGen,hStruct));                   % Header

fprintf(fid, '%s \n', 'qdd = zeros(length(q),1);');                        % Code

funcCall = 'qdd = rob.inertia(q) \ rob.Iqdd(q,qd,tau).'';';
fprintf(fid, '%s \n', funcCall);

fclose(fid);
CGen.logmsg('\t%s\n',' done!');

end

function hStruct = createHeaderStructIqdd(rob,fName)
[~,hStruct.funName] = fileparts(fName);
hStruct.shortDescription = ['Vector of computed inertial forces/torques for ',rob.name];
hStruct.calls = {['Iacc = ',hStruct.funName,'(rob,q,qd,tau)'],...
    ['Iacc = rob.',hStruct.funName,'(q,qd,tau)']};
hStruct.detailedDescription = {'Given a full set of joint variables, their temporal derivatives and applied joint forces/torques',...
    'this function computes the reaction inertial forces/torques due to joint acceleration.'};
hStruct.inputs = { ['rob: robot object of ', rob.name, ' specific class'],...
                   ['q:  ',int2str(rob.n),'-element vector of generalized'],...
                   '     coordinates',...
                   ['qd:  ',int2str(rob.n),'-element vector of generalized'],...
                   '     velocities', ...
                   ['tau:  ',int2str(rob.n),'-element vector of joint'],...
                   '     input forces/torques',...
                   'Angles have to be given in radians!'};
hStruct.outputs = {['Iqdd:  [1x',int2str(rob.n),'] vector of inertial reaction forces/torques']};
hStruct.references = {'1) Robot Modeling and Control - Spong, Hutchinson, Vidyasagar',...
    '2) Modelling and Control of Robot Manipulators - Sciavicco, Siciliano',...
    '3) Introduction to Robotics, Mechanics and Control - Craig',...
    '4) Modeling, Identification & Control of Robots - Khalil & Dombre'};
hStruct.authors = {'This is an autogenerated function!',...
    'Code generator written by:',...
    'J�rn Malzahn',...
    '2012 RST, Technische Universit�t Dortmund, Germany',...
    'http://www.rst.e-technik.tu-dortmund.de'};
hStruct.seeAlso = {'accel'};
end


function hStruct = createHeaderStructAccel(rob,fname)
[~,hStruct.funName] = fileparts(fname);
hStruct.shortDescription = ['Computation of the joint acceleration for ',rob.name];
hStruct.calls = {['qdd = ',hStruct.funName,'(rob,q,qd,tau)'],...
    ['qdd = rob.',hStruct.funName,'(q,qd,tau)']};
hStruct.detailedDescription = {'Given a full set of joint variables, their temporal derivatives and applied joint forces/torques',...
    'this function computes the joint acceleration.'};
hStruct.inputs = { ['rob: robot object of ', rob.name, ' specific class'],...
                   ['q:  ',int2str(rob.n),'-element vector of generalized'],...
                   '     coordinates',...
                   ['qd:  ',int2str(rob.n),'-element vector of generalized'],...
                   '     velocities', ...
                   ['tau:  ',int2str(rob.n),'-element vector of joint'],...
                   '     input forces/torques',...
                   'Angles have to be given in radians!'};
hStruct.outputs = {['qdd:  [1x',int2str(rob.n),'] vector of joint accelerations']};
hStruct.references = {'1) Robot Modeling and Control - Spong, Hutchinson, Vidyasagar',...
    '2) Modelling and Control of Robot Manipulators - Sciavicco, Siciliano',...
    '3) Introduction to Robotics, Mechanics and Control - Craig',...
    '4) Modeling, Identification & Control of Robots - Khalil & Dombre'};
hStruct.authors = {'This is an autogenerated function!',...
    'Code generator written by:',...
    'J�rn Malzahn',...
    '2012 RST, Technische Universit�t Dortmund, Germany',...
    'http://www.rst.e-technik.tu-dortmund.de'};
hStruct.seeAlso = {'Iqdd'};
end
