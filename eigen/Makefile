all: eigen

TARBALL     = build/eigen2-2.0.3.tar.bz2
TARBALL_URL = http://pr.willowgarage.com/downloads/eigen2-2.0.3.tar.bz2
UNPACK_CMD  = tar xfj
SOURCE_DIR  = build/eigen2

include $(shell rospack find mk)/download_unpack_build.mk

eigen: $(SOURCE_DIR)/unpacked

clean:
	rm -rf $(SOURCE_DIR)

wipe: clean
	rm -rf build
