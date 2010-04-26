all: installed
 
SVN_REVISION=-r 31715
SVN_DIR = kdl-svn-trunk
OLD_SVN_DIR = kdl-svn
SVN_URL = http://svn.mech.kuleuven.be/repos/orocos/trunk/kdl
SVN_PATCH=total.patch
INSTALL_DIR = `rospack find kdl`/
CMAKE = cmake
BOOST_INCLUDE =$(shell rosboost-cfg --include_dirs)
EIGEN2_INCLUDE_DIR=`rospack cflags-only-I eigen| cut -d ' ' -f 1`
EIGEN2_DEFINE=`rospack cflags-only-other eigen`
TINYXML_INCLUDE_DIR=`rospack cflags-only-I tinyxml| cut -d ' ' -f 1`
CMAKE_ARGS = -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR)/ \
		-DPYTHON_BINDINGS=ON \
	        -DCMAKE_CXX_FLAGS=$(EIGEN2_DEFINE) \
	        -DPYTHON_SITE_PACKAGES_DIR=$(INSTALL_DIR)/lib \
		-DBUILD_MODELS=OFF \
		-DBOOST:STRING=$(BOOST_INCLUDE)\
		-DEIGEN2_INCLUDE_DIR=$(EIGEN2_INCLUDE_DIR) \
		-DTINYXML_INCLUDE_DIR=$(TINYXML_INCLUDE_DIR)\
		-DCMAKE_BUILD_TYPE="Release"

include $(shell rospack find mk)/svn_checkout.mk

installed: wiped $(SVN_DIR) Makefile
	-mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && touch src/orocos-kdl.pc && make install
	if [ `uname` = Darwin ]; then \
		install_name_tool -id `rospack find kdl`/kdl/lib/liborocos-kdl.1.0.dylib kdl/lib/liborocos-kdl.1.0.dylib; \
	fi
	touch installed

wiped: Makefile $(SVN_PATCH)
	make wipe

clean:
	rm -rf $(INSTALL_DIR)/lib
	rm -rf $(INSTALL_DIR)/include
	rm -rf $(INSTALL_DIR)/share
	rm -rf $(SVN_DIR)
	rm -rf installed

wipe: 	clean
	rm -rf $(OLD_SVN_DIR)
	touch wiped
