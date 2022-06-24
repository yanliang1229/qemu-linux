################################################################################
#
# sys_mmap test
#
################################################################################

MMAP_VERSION:=1.0.0
MMAP_SITE=$(TOPDIR)/app/mmap
MMAP_SITE_METHOD=local

define MMAP_BUILD_CMDS
    $(TARGET_MAKE_ENV) $(MAKE) CC=$(TARGET_CC) CXX=$(TARGET_CXX) -C $(@D)
endef

define MMAP_CLEAN_CMDS
    $(TARGET_MAKE_ENV) $(MAKE) -C $(@D) clean
endef

define MMAP_INSTALL_TARGET_CMDS
    	mkdir -p $(TARGET_DIR)/test
	$(INSTALL) -D -m 0755 $(@D)/mmap $(TARGET_DIR)/test
endef

$(eval $(generic-package))
