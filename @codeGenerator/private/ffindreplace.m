function [ ] = ffindreplace(CGen, fName, oText, nText, varargin)
%% FFINDREPLACE Find and replace all occurrences of string in a file.
%
%   [ ] = ffindreplace(CGen, fName, oText, nText, varargin)
%   [ ] = CGen.ffindreplace(fName, oText, nText, varargin)
%
%  Description::
%    The function opens and sweeps the text file fName. All occurrences of
%    oText are replaced by nText.
%
%  Input::
%    fName:             Name of the file to replace the text in.
%    oText:             Text passage to replace.
%    nText:             New text replacement.
%
%  Authors::
%        J�rn Malzahn   
%        2012 RST, Technische Universit�t Dortmund, Germany
%        http://www.rst.e-technik.tu-dortmund.de   
%
%  See also finsertfront.


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
 
fid = fopen(fName,'r'); % open the file

% determine number of lines in the file
nLines = getNLines(fid);

% read all lines to cell
oldLines = cell(1,nLines);
newLines = {1,nLines};
for iLines = 1:nLines
    oldLines{iLines} = fgets(fid);
end

% close the file again
fclose(fid);

% replace all occurrences of oText by nText in each line
for iLines = 1:nLines
    newLines{iLines}= strrep(oldLines{iLines}, oText, nText);
end

% rewrite the file
fid = fopen(fName,'w');
for iLines = 1:nLines
   fprintf(fid,'%s',newLines{iLines});
end
fclose(fid);


end

function [nLines] = getNLines(fid)
nLines = 0;
while (~feof(fid))      % go through each line until eof is reached
    fgets(fid);
    nLines = nLines +1; % increment line counter
end
frewind(fid);           % set file indicator back to the beginning of the file
end