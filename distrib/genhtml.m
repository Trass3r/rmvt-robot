% create the html format documentation using m2html
cd robot
addpath ../m2html
!\rm -rf ../htmldoc
m2html('htmldir','htmldoc', 'recursive', 'on') 
exit
