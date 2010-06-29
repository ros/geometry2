all: eigen

TARBALL     = build/eigen2-2.0.14.tar.bz2
TARBALL_URL = http://pr.willowgarage.com/downloads/eigen2-2.0.14.tar.bz2
UNPACK_CMD  = tar xfj
SOURCE_DIR  = build/eigen
MD5SUM_FILE = eigen2-2.0.14.tar.bz2.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = include


eigen: $(INSTALL_DIR)/installed

$(INSTALL_DIR)/installed: $(SOURCE_DIR)/unpacked
	mkdir -p $(INSTALL_DIR)
	cp -r $(SOURCE_DIR)/Eigen $(INSTALL_DIR)
	touch $(INSTALL_DIR)/installed

clean:
	rm -rf $(SOURCE_DIR) $(INSTALL_DIR)

wipe: clean
	rm -rf build
