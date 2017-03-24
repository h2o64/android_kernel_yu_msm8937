#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.

# Shuai.Chen, Date20160702, Modify For Wiko Unify Version, HCABMA-2, Start
WIKO_UNIFY_DEF := $(strip $(subst ro.wiko.type =,,$(shell cat $(PWD)/../build.ini |grep ^ro.wiko.type)))
ifeq ($(strip $(WIKO_UNIFY_DEF)),unify)
ccflags-y += -DCONFIG_WIKO_UNIFY
$(warning ---add marco CONFIG_WIKO_UNIFY---)
endif
# Shuai.Chen, Date20160702, Modify For Wiko Unify Version, HCABMA-2, End

obj-$(CONFIG_INPUT)		+= input-core.o
input-core-y := input.o input-compat.o input-mt.o ff-core.o

obj-$(CONFIG_INPUT_FF_MEMLESS)	+= ff-memless.o
obj-$(CONFIG_INPUT_POLLDEV)	+= input-polldev.o
obj-$(CONFIG_INPUT_SPARSEKMAP)	+= sparse-keymap.o
obj-$(CONFIG_INPUT_MATRIXKMAP)	+= matrix-keymap.o

obj-$(CONFIG_INPUT_MOUSEDEV)	+= mousedev.o
obj-$(CONFIG_INPUT_JOYDEV)	+= joydev.o
obj-$(CONFIG_INPUT_EVDEV)	+= evdev.o
obj-$(CONFIG_INPUT_EVBUG)	+= evbug.o

obj-$(CONFIG_INPUT_KEYBOARD)	+= keyboard/
obj-$(CONFIG_INPUT_MOUSE)	+= mouse/
obj-$(CONFIG_INPUT_JOYSTICK)	+= joystick/
obj-$(CONFIG_INPUT_TABLET)	+= tablet/
obj-$(CONFIG_INPUT_TOUCHSCREEN)	+= touchscreen/
obj-$(CONFIG_INPUT_MISC)	+= misc/

obj-$(CONFIG_INPUT_APMPOWER)	+= apm-power.o
obj-$(CONFIG_INPUT_KEYRESET)	+= keyreset.o
obj-$(CONFIG_INPUT_KEYCOMBO)	+= keycombo.o

#zenghaihui add for fingerprint
obj-$(CONFIG_INPUT_FINGERPRINT)	+= fingerprint/

