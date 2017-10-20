#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.
obj-$(CONFIG_TOUCHSCREEN_FT5XX) += focaltech_core.o focaltech_ctl.o focaltech_ex_fun.o focaltech_gesture.o focaltech_flash.o 
#obj-y += fts_ts.o 
#fts_ts-y += focaltech_core.o focaltech_ctl.o focaltech_ex_fun.o focaltech_gesture.o focaltech_flash.o 
extlibs :=$(PWD)/drivers/input/touchscreen/ft5xx/ft_gesture_64bit_lib_v1.0_20140820.a 
#ldflagobj-y += fts_ts.o 
#obj-y s-y +=--whole-archive $(extlibs) --no-whole-archive
