distro:	
	# clean up
	\rm -rf robot robot.zip robot.tgz

	# make a temp directory to hold the files
	mkdir robot

	# core toolbox components
	cp ../*.m robot
	cp -r ../@* robot
	cp -r ../demos robot
	cp -r ../simulink robot
	cp -r ../mex robot

	# doco
	cp ../doc/robot.pdf robot
	cp -r ../htmldoc robot

	# info files
	cp ../README robot
	cp ../RELEASE robot
	cp ../WHATSNEW robot
	cp ../LGPL-LICENCE.txt robot

	# remove all SVN files
	\rm -rf `find . -name .svn`

	# build the distrib archives
	zip -rq robot.zip ./robot
	tar zcf robot.tgz ./robot

clean:
	\rm -rf robot robot.zip robot.tgz htmldoc
