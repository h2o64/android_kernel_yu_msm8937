#
# Makefile for the linux kernel.
#
# Shuai.Chen, Date20160702, Modify For Wiko Unify Version, HCABMA-2, Start
#WIKO_UNIFY_DEF := $(strip $(subst ro.wiko.type =,,$(shell cat $(PWD)/build.ini |grep ^ro.wiko.type)))
##$(warning -wlj--WIKO_UNIFY_DEF= $(WIKO_UNIFY_DEF))
#$(warning -wlj--PWD= $(PWD)-------)
#ifeq ($(strip $(WIKO_UNIFY_DEF)),unify)
#$(warning -wlj--PWD= $(PWD)-------)
#ccflags-y += -DCONFIG_WIKO_UNIFY
#$(warning -wlj---ccflags-y += -DCONFIG_WIKO_UNIFY---)
#ccflags-y += -DCONFIG_TINNO_DEV_INFO
#$(warning -wlj---ccflags-y += -DCONFIG_TINNO_DEV_INFO---)
#else
#$(warning -wlj-- error)
#endif


WIKO_UNIFY_DEF := $(strip $(subst ro.wiko.type =,,$(shell cat $(PWD)/../../../../../../build.ini |grep ^ro.wiko.type)))
$(warning -wlj--1--PWD= $(PWD)-------)
ifeq ($(strip $(WIKO_UNIFY_DEF)),unify)
$(warning -wlj--1--WIKO_UNIFY_DEF= $(WIKO_UNIFY_DEF))
ccflags-y += -DCONFIG_WIKO_UNIFY
$(warning -wlj--1---ccflags-y += -DCONFIG_WIKO_UNIFY---)
ccflags-y += -DCONFIG_TINNO_DEV_INFO
$(warning -wlj--1---ccflags-y += -DCONFIG_TINNO_DEV_INFO---)
endif

WIKO_UNIFY_DEF := $(strip $(subst ro.wiko.type =,,$(shell cat $(PWD)/build.ini |grep ^ro.wiko.type)))
$(warning -wlj--2--PWD= $(PWD)-------)
ifeq ($(strip $(WIKO_UNIFY_DEF)),unify)
$(warning -wlj--2--WIKO_UNIFY_DEF= $(WIKO_UNIFY_DEF))
ccflags-y += -DCONFIG_WIKO_UNIFY
$(warning -wlj--2---ccflags-y += -DCONFIG_WIKO_UNIFY---)
ccflags-y += -DCONFIG_TINNO_DEV_INFO
$(warning -wlj--2---ccflags-y += -DCONFIG_TINNO_DEV_INFO---)
endif


# Shuai.Chen, Date20160702, Modify For Wiko Unify Version, HCABMA-2, End

obj-y                          := main.o version.o mounts.o
obj-y                          += noinitramfs.o
obj-$(CONFIG_BLK_DEV_INITRD)   += initramfs.o
obj-$(CONFIG_GENERIC_CALIBRATE_DELAY) += calibrate.o

ifneq ($(CONFIG_ARCH_INIT_TASK),y)
obj-y                          += init_task.o
endif

mounts-y			:= do_mounts.o
mounts-$(CONFIG_BLK_DEV_RAM)	+= do_mounts_rd.o
mounts-$(CONFIG_BLK_DEV_INITRD)	+= do_mounts_initrd.o
mounts-$(CONFIG_BLK_DEV_MD)	+= do_mounts_md.o

# dependencies on generated files need to be listed explicitly
$(obj)/version.o: include/generated/compile.h

# compile.h changes depending on hostname, generation number, etc,
# so we regenerate it always.
# mkcompile_h will make sure to only update the
# actual file if its content has changed.

       chk_compile.h = :
 quiet_chk_compile.h = echo '  CHK     $@'
silent_chk_compile.h = :
include/generated/compile.h: FORCE
	@$($(quiet)chk_compile.h)
	$(Q)$(CONFIG_SHELL) $(srctree)/scripts/mkcompile_h $@ \
	"$(UTS_MACHINE)" "$(CONFIG_SMP)" "$(CONFIG_PREEMPT)" "$(CC) $(KBUILD_CFLAGS)"
