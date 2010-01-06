all: installed
 
SVN_REVISION=-r 30401
SVN_DIR = kdl-svn-trunk
OLD_SVN_DIR = kdl-svn
SVN_URL = http://svn.mech.kuleuven.be/repos/orocos/trunk/kdl
SVN_PATCH=total.patch
INSTALL_DIR = kdl
CMAKE = cmake
BOOST_INCLUDE =$(shell rosboost-cfg --include_dirs)
EIGEN2_INCLUDE_DIR=`rospack cflags-only-I eigen| cut -d ' ' -f 1`
EIGEN2_DEFINE=`rospack cflags-only-other eigen`
TINYXML_INCLUDE_DIR=`rospack cflags-only-I tinyxml| cut -d ' ' -f 1`
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=`rospack find kdl`/$(INSTALL_DIR)/ \
		-DPYTHON_BINDINGS=ON \
		-DBUILD_MODELS=OFF \
		-DBOOST:STRING=$(BOOST_INCLUDE)\
		-DEIGEN2_INCLUDE_DIR=$(EIGEN2_INCLUDE_DIR)\
		-DTINYXML_INCLUDE_DIR=$(TINYXML_INCLUDE_DIR)\
		-DCMAKE_BUILD_TYPE="Release"

include $(shell rospack find mk)/svn_checkout.mk

installed: wiped $(SVN_DIR) Makefile
	-mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	if [ `uname` = Darwin ]; then \
		install_name_tool -id `rospack find kdl`/kdl/lib/liborocos-kdl.1.0.dylib kdl/lib/liborocos-kdl.1.0.dylib; \
	fi
	ln -s `rospack find kdl`/kdl/lib/`readlink \`which python\`` `rospack find kdl`/kdl/lib/python; \
        [ -d `rospack find kdl`/kdl/lib/python/dist-packages ] && ln -s `rospack find kdl`/kdl/lib/python/dist-packages `rospack find kdl`/kdl/lib/python/site-packages; \
	touch installed

wiped: Makefile $(SVN_PATCH)
	make wipe

clean:
	rm -rf $(INSTALL_DIR)
	rm -rf $(SVN_DIR)/build
	rm -rf installed

wipe: 	clean
	rm -rf $(SVN_DIR)
	rm -rf $(OLD_SVN_DIR)
	touch wiped
