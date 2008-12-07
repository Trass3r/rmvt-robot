distro:	
	# clean up
	\rm -rf robot robot.zip robot.tgz

	# make a temp directory to hold the files
	mkdir robot

	# core toolbox components
	cp ../*.m robot
	cp ../*.xml robot
	cp -r ../@* robot
	cp -r ../demos robot
	cp -r ../simulink robot
	cp -r ../mex robot
	cp ~/lib/matlab/num*.m robot

	# doco
	cp ../doc/robot.pdf robot
	matlab -nodesktop -nosplash -r genhtml
	cp extras/* robot/htmldoc
	cp robot/htmldoc/{index.html,robot_product_page.html}

	# info files
	cp ../README robot
	cp ../RELEASE robot
	cp ../WHATSNEW robot
	cp ../LGPL-LICENCE.txt robot

	# remove all SVN files
	\rm -rf `find . -name .svn`

	# build the distrib archives
	zip -rq robot.zip ./robot
	tar zcf robot.tar.gz ./robot

clean:
	\rm -rf robot robot.zip robot.tgz
