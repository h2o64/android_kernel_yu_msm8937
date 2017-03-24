#
# Makefile for the fingerprint drivers.
#

ifneq ($(strip $(MTK_TARGET_PROJECT)),)
  TARGET_PRJ := $(MTK_TARGET_PROJECT)
else
  TARGET_PRJ := $(TARGET_PRODUCT)
endif 
TARGET_PRJ := $(strip $(TARGET_PRJ))

obj-y	+=  fp_drv/

obj-$(CONFIG_FINGERPRINT_GOODIX_GFX1XM)		+= gfx1xm/

obj-$(CONFIG_FINGERPRINT_GOODIX_GFX1XM_QSEE)	+= gfx1xm_qsee/

ifeq ($(strip $(TARGET_PRJ)),p7201)
    $(warning ---p7201 elan fingerprint kernel---)
    obj-$(CONFIG_FINGERPRINT_ELAN)		+= elan_new/
else
    obj-$(CONFIG_FINGERPRINT_ELAN)		+= elan/
endif


