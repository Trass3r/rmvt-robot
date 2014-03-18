%CODEGENERATOR.GENCCODEFDYN Generate C-code for forward dynamics
%
% cGen.genccodeinvdyn() generates a robot-specific C-code to compute the
% forward dynamics.
%
% Notes::
% - Is called by CodeGenerator.genfdyn if cGen has active flag genccode or
%   genmex.
% - The .c and .h files are generated in the directory specified
%   by the ccodepath property of the CodeGenerator object.
% - The resulting C-function is composed of previously generated C-functions
%   for the inertia matrix, coriolis matrix, vector of gravitational load and
%   joint friction vector. This function recombines these components to
%   compute the forward dynamics.
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also CodeGenerator.CodeGenerator, CodeGenerator.genfdyn,CodeGenerator.genccodeinvdyn.

% Copyright (C) 2012-2013, by Joern Malzahn
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

function [ ] = genccodefdyn( CGen )

checkexistanceofcfunctions(CGen);
[Q, QD] = CGen.rob.gencoords;
tau = CGen.rob.genforces;
nJoints = CGen.rob.n;

% Check for existance of C-code directories
srcDir = fullfile(CGen.ccodepath,'src');
hdrDir = fullfile(CGen.ccodepath,'include');
if ~exist(srcDir,'dir')
    mkdir(srcDir);
end
if ~exist(hdrDir,'dir')
    mkdir(hdrDir);
end

symname = 'fdyn';
outputname = 'QDD';

funname = [CGen.rob.name,'_',symname];
funfilename = [funname,'.c'];
hfilename = [funname,'.h'];

CGen.logmsg([datestr(now),'\tGenerating forward dynamics C-code']);

% Create the function description header
hStruct = createHeaderStruct(CGen.rob,symname); % create header
if ~isempty(hStruct)
    hFString = CGen.constructheaderstringc(hStruct);
end

dummy = sym(zeros(nJoints,1));
% Convert symbolic expression into C-code
[funstr hstring] = ccodefunctionstring(dummy,...
    'funname',funname,...
    'vars',{Q, QD, tau},'output',outputname,...
    'flag',1);



%% Generate C implementation file
fid = fopen(fullfile(srcDir,funfilename),'w+');

% Function description header
fprintf(fid,'%s\n\n',hFString);

% Includes
fprintf(fid,'%s\n\n',...
    ['#include "', hfilename,'"']);

% Function
fprintf(fid,'%s{\n\n',funstr);


% Allocate memory
fprintf(fid,'\t%s\n','/* declare variables */');
fprintf(fid,'\t%s\n','int iCol;');
fprintf(fid,'\t%s\n',['double inertia[',num2str(nJoints),'][',num2str(nJoints),'];']);
fprintf(fid,'\t%s\n',['double invinertia[',num2str(nJoints),'][',num2str(nJoints),'];']);
fprintf(fid,'\t%s\n',['double coriolis[',num2str(nJoints),'][',num2str(nJoints),'];']);
fprintf(fid,'\t%s\n',['double gravload[',num2str(nJoints),'][1];']);
fprintf(fid,'\t%s\n',['double friction[',num2str(nJoints),'][1];']);
fprintf(fid,'\t%s\n',['double tmpTau[',num2str(nJoints),'][1];']);

fprintf(fid,'%s\n',' '); % empty line

fprintf(fid,'\t%s\n','/* call the computational routines */');
fprintf(fid,'\t%s\n',[CGen.rob.name,'_','inertia(inertia, input1);']);
fprintf(fid,'\t%s\n',['gaussjordan(inertia, invinertia, ',num2str(nJoints),');']);
fprintf(fid,'\t%s\n',[CGen.rob.name,'_','coriolis(coriolis, input1, input2);']);
fprintf(fid,'\t%s\n',[CGen.rob.name,'_','gravload(gravload, input1);']);
fprintf(fid,'\t%s\n',[CGen.rob.name,'_','friction(friction, input2);']);

fprintf(fid,'%s\n',' '); % empty line

fprintf(fid,'\t%s\n','/* fill temporary vector */');
fprintf(fid,'\t%s\n',['matvecprod(tmpTau, coriolis, input2,',num2str(nJoints),',',num2str(nJoints),');']);
fprintf(fid,'\t%s\n',['for (iCol = 0; iCol < ',num2str(nJoints),'; iCol++){']);
fprintf(fid,'\t\t%s\n','tmpTau[iCol][0] = input3[iCol] -  tmpTau[iCol][0] - gravload[iCol][0] + friction[iCol][0];');
fprintf(fid,'\t%s\n','}');

fprintf(fid,'\t%s\n','/* compute acceleration */');
fprintf(fid,'\t%s\n',['matvecprod(QDD, invinertia, tmpTau,',num2str(nJoints),',',num2str(nJoints),');']);

fprintf(fid,'%s\n','}');

fclose(fid);


%% Generate C header file
fid = fopen(fullfile(hdrDir,hfilename),'w+');

% Function description header
fprintf(fid,'%s\n\n',hFString);

% Include guard
fprintf(fid,'%s\n%s\n\n',...
    ['#ifndef ', upper([funname,'_h'])],...
    ['#define ', upper([funname,'_h'])]);

% Includes
fprintf(fid,'%s\n%s\n%s\n%s\n%s\n%s\n\n',...
    '#include "matvecprod.h"',...
    '#include "gaussjordan.h"',...
    ['#include "',CGen.rob.name,'_inertia.h"'],...
    ['#include "',CGen.rob.name,'_coriolis.h"'],...
    ['#include "',CGen.rob.name,'_gravload.h"'],...
    ['#include "',CGen.rob.name,'_friction.h"']);


fprintf(fid,'%s\n\n',hstring);

% Include guard
fprintf(fid,'%s\n',...
    ['#endif /*', upper([funname,'_h */'])]);

fclose(fid);

CGen.logmsg('\t%s\n',' done!');

end

function [] = checkexistanceofcfunctions(CGen)

if ~(exist(fullfile(CGen.ccodepath,'src','inertia.c'),'file') == 2) || ~(exist(fullfile(CGen.ccodepath,'include','inertia.h'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Inertia C-code not found or not complete! Generating:');
    CGen.genccodeinertia;
end

if ~(exist(fullfile(CGen.ccodepath,'src','coriolis.c'),'file') == 2) || ~(exist(fullfile(CGen.ccodepath,'coriolis','inertia.h'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Coriolis C-code not found or not complete! Generating:');
    CGen.genccodecoriolis;
end

if ~(exist(fullfile(CGen.ccodepath,'src','gravload.c'),'file') == 2) || ~(exist(fullfile(CGen.ccodepath,'include','gravload.h'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Gravload C-code not found or not complete! Generating:');
    CGen.genccodegravload;
end

if ~(exist(fullfile(CGen.ccodepath,'src','friction.c'),'file') == 2) || ~(exist(fullfile(CGen.ccodepath,'include','friction.h'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Friction C-code not found or not complete! Generating:');
    CGen.genccodefriction;
end

if ~(exist(fullfile(CGen.ccodepath,'src','matvecprod.c'),'file') == 2) || ~(exist(fullfile(CGen.ccodepath,'include','matvecprod.h'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Matrix-Vector product C-code not found or not complete! Generating:');
    CGen.genmatvecprodc;
end

if ~(exist(fullfile(CGen.ccodepath,'src','gaussjordan.c'),'file') == 2) || ~(exist(fullfile(CGen.ccodepath,'include','gaussjordan.h'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Gauss-Jordan matrix inversion C-code not found or not complete! Generating:');
    CGen.gengaussjordanc;
end

end

%% Definition of the header contents for each generated file
function hStruct = createHeaderStruct(rob,fname)
[~,hStruct.funName] = fileparts(fname);
hStruct.shortDescription = ['C-implementation of the forward dynamics for the',rob.name,' arm.'];
hStruct.calls = {['qdd = ',hStruct.funName,'(rob,q,qd,tau)'],...
    ['qdd = rob.',hStruct.funName,'(q,qd,tau)']};
hStruct.detailedDescription = {'Given a full set of joint angles and velocities',...
    'this function computes the joint space accelerations effected by the generalized forces.'};
hStruct.inputs = { ['rob: robot object of ', rob.name, ' specific class'],...
    ['q:  ',int2str(rob.n),'-element vector of generalized'],...
    '     coordinates',...
    ['qd:  ',int2str(rob.n),'-element vector of generalized'],...
    '     velocities', ...
    ['tau:  [',int2str(rob.n),'x1] vector of generalized forces.'],...
    'Angles have to be given in radians!'};
hStruct.outputs = {['qdd:  ',int2str(rob.n),'-element vector of generalized accelerations.']};
hStruct.references = {'1) Robot Modeling and Control - Spong, Hutchinson, Vidyasagar',...
    '2) Modelling and Control of Robot Manipulators - Sciavicco, Siciliano',...
    '3) Introduction to Robotics, Mechanics and Control - Craig',...
    '4) Modeling, Identification & Control of Robots - Khalil & Dombre'};
hStruct.authors = {'This is an autogenerated function!',...
    'Code generator written by:',...
    'Joern Malzahn (joern.malzahn@tu-dortmund.de)'};
hStruct.seeAlso = {'invdyn'};
end