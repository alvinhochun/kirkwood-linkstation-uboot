#!/bin/sh

. /opt/cross.conf

CROSS_ENV_PATH=${MARVELL_ARM_ENV2}
export PATH=${CROSS_ENV_PATH}/bin:${PATH}
IMAGE_DIR=../images

build_lsvl()
{
	echo "*** LS-VL ***"
	OUTPUT=buffalo_mvlsvl_6282
	RULE=${OUTPUT}_config
	
	make mrproper
	make OTHERCFLAGS="" ${RULE} SPIBOOT=1 DDR3=1
	make -s
	if [ -f u-boot-${OUTPUT}.bin ]; then
		./tools/doimage -T flash -D 0x600000 -E 0x660000 -R dramregs_533ddr3db_A_buf.txt \
		u-boot-${OUTPUT}.bin u-boot_lsvl.bin
		cp u-boot_lsvl.bin $IMAGE_DIR
	fi
}

build_lswvl()
{
	echo "*** LS-WVL ***"
	OUTPUT=buffalo_mvlswvl_6282
	RULE=${OUTPUT}_config
	
	make mrproper
	make OTHERCFLAGS="" ${RULE} SPIBOOT=1 DDR3=1 NAND=1
	make -s
	if [ -f u-boot-${OUTPUT}.bin ]; then
		./tools/doimage -T flash -D 0x600000 -E 0x660000 -R dramregs_533ddr3db_A_buf.txt \
		u-boot-${OUTPUT}.bin u-boot_lswvl.bin
		cp u-boot_lswvl.bin $IMAGE_DIR
	fi
}

build_lsxhl()
{
	echo "*** LS-XHL/LS-XL ***"
	OUTPUT=buffalo_mvlsxh_6281
	RULE=${OUTPUT}_config
	
	make mrproper
	make OTHERCFLAGS="" ${RULE} SPIBOOT=1  LE=1
	make -s
	if [ -f u-boot-${OUTPUT}.bin ]; then
		./tools/doimage -T flash -D 0x600000 -E 0x660000 -R dramregs_400mvlsxh_A.txt \
		u-boot-${OUTPUT}.bin u-boot_lsxh.bin
		cp u-boot_lsxh.bin $IMAGE_DIR

		./tools/doimage -T flash -D 0x600000 -E 0x660000 -R dramregs_300xl_A.txt \
		u-boot-${OUTPUT}.bin u-boot_lsxl.bin
		cp u-boot_lsxl.bin $IMAGE_DIR
	fi
}

build_lswxl()
{
	echo "*** LS-WXL/LS-WSX ***"
	OUTPUT=buffalo_mvwxl_6281
	RULE=${OUTPUT}_config

	make mrproper
	make OTHERCFLAGS="" ${RULE} SPIBOOT=1 LE=1 NAND=1
	make -s
	if [ -f u-boot-${OUTPUT}.bin ]; then
		./tools/doimage -T flash -D 0x600000 -E 0x660000 -R dramregs_300wxl_A.txt \
		u-boot-${OUTPUT}.bin u-boot_lswxl.bin
		cp u-boot_lswxl.bin $IMAGE_DIR
	fi
}

build_lsxlv2()
{
	echo "*** LS-XL-V2 ***"
	OUTPUT=buffalo_mvlsxlv2_6192
	RULE=${OUTPUT}_config
	
	make mrproper
	make OTHERCFLAGS="" ${RULE} SPIBOOT=1 LE=1 NAND=1
	make -s
	if [ -f u-boot-${OUTPUT}.bin ]; then
		./tools/doimage -T flash -D 0x600000 -E 0x660000 -R dramregs_lsxlv2_64mb.txt \
		u-boot-${OUTPUT}.bin u-boot_lsxlv2_64mb.bin
		cp u-boot_lsxlv2_64mb.bin $IMAGE_DIR

		./tools/doimage -T uart -D 0x600000 -E 0x660000 -R dramregs_lsxlv2_64mb.txt \
		u-boot-${OUTPUT}.bin u-boot_lsxlv2_64mb_uart.bin
		cp u-boot_lsxlv2_64mb_uart.bin $IMAGE_DIR

		./tools/doimage -T flash -D 0x600000 -E 0x660000 -R dramregs_lsxlv2_128mb.txt \
		u-boot-${OUTPUT}.bin u-boot_lsxlv2_128mb.bin
		cp u-boot_lsxlv2_128mb.bin $IMAGE_DIR

		./tools/doimage -T uart -D 0x600000 -E 0x660000 -R dramregs_lsxlv2_128mb.txt \
		u-boot-${OUTPUT}.bin u-boot_lsxlv2_128mb_uart.bin
		cp u-boot_lsxlv2_128mb_uart.bin $IMAGE_DIR
	fi
}

[ -d $IMAGE_DIR ] || mkdir $IMAGE_DIR

case $1 in
    lsxhl|lsxl)
	build_lsxhl
	;;
    lswxl|lswsxl)
	build_lswxl
	;;
    lsvl)
	build_lsvl
	;;
    lswvl)
	build_lswvl
	;;
    lsxlv2)
	build_lsxlv2
	;;
    all)
	build_lsxhl
	build_lswxl
	build_lsvl
	build_lswvl
	build_lsxlv2
	;;
    *)
	echo "Usage: `basename $0` <target>"
esac
