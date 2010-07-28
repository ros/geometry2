all: installed
  
FILENAME = kdl-31715.tar.gz
# new tarbal that pulls in a number of kdl bugfixes
# needs extensive testing before we start using it
#FILENAME = kdl-32171.tar.gz
TARBALL = build/$(FILENAME)
TARBALL_URL = http://pr.willowgarage.com/downloads/$(FILENAME)
SOURCE_DIR = build/kdl-tar
UNPACK_CMD = tar xzf
MD5SUM_FILE = $(FILENAME).md5sum
include $(shell rospack find mk)/download_unpack_build.mk

PATCH = total.patch
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
		-DEIGEN2_INCLUDE_DIR=$(EIGEN2_INCLUDE_DIR)\
		-DTINYXML_INCLUDE_DIR=$(TINYXML_INCLUDE_DIR)\
		-DCMAKE_BUILD_TYPE="Release"


installed: wiped $(SOURCE_DIR)/unpacked 
	cd $(SOURCE_DIR) && patch -p0 < ../../$(PATCH)
	mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SOURCE_DIR)/build && make $(ROS_PARALLEL_JOBS) && touch src/orocos-kdl.pc && make install
	if [ `uname` = Darwin ]; then \
		install_name_tool -id `rospack find kdl`/kdl/lib/liborocos-kdl.1.0.dylib kdl/lib/liborocos-kdl.1.0.dylib; \
	fi
	touch installed


docs: 
	cd $(SOURCE_DIR)/build && make docs
	ln -s $(SOURCE_DIR)/build/doc doc

wiped: Makefile $(PATCH) kdl-31715.tar.gz.md5sum
	make wipe
	touch wiped

clean:
	rm -rf lib
	rm -rf include
	rm -rf share
	rm -rf build
	rm -rf installed

wipe: 	clean
	touch wiped
