cd robot
addpath ~/lib/matlab/m2html
!\rm -rf ../htmldoc
m2html('htmldir','../htmldoc', 'recursive', 'on') 
