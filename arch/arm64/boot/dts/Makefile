dts-dirs += apm
dts-dirs += arm
dts-dirs += cavium

DTS_DIR := qcom
ifeq ($(CONFIG_PROJECT_P7701),y)
DTS_DIR := p7701
endif

ifeq ($(CONFIG_PROJECT_P7705),y)
DTS_DIR := p7705
endif

ifeq ($(CONFIG_PROJECT_P7203),y)
DTS_DIR := p7203
endif

ifeq ($(CONFIG_PROJECT_P7201),y)
DTS_DIR := p7201
endif

ifeq ($(CONFIG_PROJECT_I9051),y)
DTS_DIR := i9051
endif
dts-dirs += $(DTS_DIR)

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
