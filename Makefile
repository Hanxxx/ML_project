#
# $RCSfile: Makefile,v $
# $Revision: 1.24 $
# $Date: 2000/04/15 02:29:03 $
# $Locker:  $
#  author: Tucker Balch
#

SUBDIRS = src lib Docs Domains

unexport THISDIR Localizer

include Makefile.src

.PHONEY: skelfiles teambots documentation updatesite publish

skelfiles:
	cd $(SKELDIR); $(RM) -rf *
	zip -ry $(SKELDIR)/skel.zip .
	cd $(SKELDIR); unzip skel.zip; rm skel.zip; make -i skelclean

publish:
	cp ../TeamBots.zip /afs/cs.cmu.edu/user/trb/www/TeamBots
	cp ../JCye.zip     /afs/cs.cmu.edu/user/trb/www/TeamBots
	cp ../CMVision.zip /afs/cs.cmu.edu/user/trb/www/TeamBots
	cd /afs/cs.cmu.edu/user/trb/www/TeamBots; unzip TeamBots

teambots:
	$(RM) ../TeamBots.zip
	zip -ry ../TeamBots.zip . -x \
                "*.Z" \
                ".nfs*" \
                "*.zip" \
                "*.tar" \
                "*profinfo" \
                "*~*" \
                "*experiment*" \
                "*RCS*" \
                "*CVS*" \
                "*.bak" \
                "*.swp" \
                "*internal*" \
                "*Util*" \
                "*robolink*" \
                "*control*" \
                "*vision*" \
                "*/old/*" \
                "*/Old/*" \
                "*ForageLearn*" \
                "*Bench*" \
                "*ForageLearn*"\
                "*Formation*"\
                "*Mobile*"\
                "*/misc/*"\
                "*SoccerLearn*"\
                "*Util*"\
                "bin*"\
		"Bench*"\
		"CVS*"\
		"Cacl*"\
		"Formation*"\
		"*Util*"\

cmvision:
	$(RM) ../CMVision.zip
	zip -ry ../CMVision.zip . -x \
                "Docs*" \
                "README.announce" \
                "README" \
                "README.developer" \
                "README.mac" \
                "README.todo" \
                "index.html" \
                "Domains*" \
                "Domains*" \
                "Domains*" \
                "*RCS*" \
                "lib*" \
                "src/EDU/*" \
                "src/JCye/*" \
                "src/RoboComm/*" \
                "src/TBHard/*" \
                "src/TBSim/*" \

jcye:
	$(RM) ../JCye.zip
	zip -ry ../JCye.zip . -x \
                "*Domains*" \
                "*EDU/gatech*" \
                "*EDU/cmu/cs/coral/abstractrobot/*" \
                "*EDU/cmu/cs/coral/localize/*" \
                "*EDU/cmu/cs/coral/simulation/*" \
                "*EDU/cmu/cs/coral/util/*" \
                "*EDU/gatech*" \
                "*Native*" \
                "*RoboComm*" \
                "*TBHard*" \
                "*TBSim*" \
                "profinfo" \
                "*.Z" \
                ".nfs*" \
                "*.zip" \
                "*.tar" \
                "*profinfo" \
                "*~*" \
                "*experiment*" \
                "*RCS*" \
                "*CVS*" \
                "*.bak" \
                "*.swp" \
                "*internal*" \
                "*Util*" \
                "*robolink*" \
                "*control*" \
                "*vision*" \
                "*/old/*" \
                "*/Old/*" \
                "*ForageLearn*" \
                "*Bench*" \
                "*ForageLearn*"\
                "*Formation*"\
                "*Mobile*"\
                "*/misc/*"\
                "*SoccerLearn*"\
                "*Util*"\
                "bin*"\
		"Bench*"\
		"CVS*"\
		"Cacl*"\
		"Formation*"\
		"*Util*"\

documentation::
	javadoc -d Docs \
		EDU.gatech.cc.is.abstractrobot \
		EDU.gatech.cc.is.clay \
		EDU.gatech.cc.is.communication \
		EDU.gatech.cc.is.learning \
		EDU.gatech.cc.is.newton \
		EDU.gatech.cc.is.nomad150 \
		EDU.gatech.cc.is.simulation \
		EDU.gatech.cc.is.util \
		EDU.cmu.cs.coral.cye \
		EDU.cmu.cs.coral.learning \
		EDU.cmu.cs.coral.simulation \
		EDU.cmu.cs.coral.util \
