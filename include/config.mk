ARCH   = arm
CPU    = arm926ejs
BOARD  = config_kw
VENDOR = mv_feroceon
MV_OUTPUT = buffalo_mvlsxh_6281
MV_FLAGS += -DCONFIG_BUFFALO_PLATFORM
MV_FLAGS += -DMV88F6281
MV_FLAGS += -DBF_MVLSXH
MV_IMAGE_FLAGS = -DMV_SEC_64K
MV_IMAGE_FLAGS += -DMV_BOOTSIZE_512K
MV_IMAGE_FLAGS += -DMV_LARGE_PAGE
MV_FLAGS += -DMV_BOOTROM
MV_FLAGS += -DTCLK_AUTO_DETECT
MV_DDR_FREQ = 400mvlsxh
CROSS_COMPILE = arm-none-linux-gnueabi-
MV_FLAGS += -DMV_USB -DCONFIG_CMD_USB
MV_USB=y
MV_FLAGS += -DMV_SPI
MV_FLAGS += -DMV_SPI_BOOT
SPI_BOOT =y
CPPFLAGS += $(MV_IMAGE_FLAGS) $(MV_FLAGS)
