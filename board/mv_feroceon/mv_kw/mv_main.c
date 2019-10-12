/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File in accordance with the terms and conditions of the General 
Public License Version 2, June 1991 (the "GPL License"), a copy of which is 
available along with the File in the license.txt file or by writing to the Free 
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or 
on the worldwide web at http://www.gnu.org/licenses/gpl.txt. 

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED 
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY 
DISCLAIMED.  The GPL License provides additional details about this warranty 
disclaimer.

*******************************************************************************/

#include <common.h>
#include "mvTypes.h"
#include "mvBoardEnvLib.h"
#include "mvCpuIf.h"
#include "mvCtrlEnvLib.h"
#include "mv_mon_init.h"
#include "mvDebug.h"
#include "device/mvDevice.h"
#include "twsi/mvTwsi.h"
#include "eth/mvEth.h"
#include "pex/mvPex.h"
#include "gpp/mvGpp.h"
#include "sys/mvSysUsb.h"
#include "mv_service.h"

#ifdef MV_INCLUDE_RTC
#include "rtc/integ_rtc/mvRtc.h"
#include "rtc.h"
#elif CONFIG_RTC_DS1338_DS1339
#include "rtc/ext_rtc/mvDS133x.h"
#endif

#if defined(MV_INCLUDE_XOR)
#include "xor/mvXor.h"
#endif
#include "lcd/mvLcd.h"
#if defined(MV_INCLUDE_IDMA)
#include "sys/mvSysIdma.h"
#include "idma/mvIdma.h"
#endif
#if defined(MV_INCLUDE_USB)
#include "usb/mvUsb.h"
#endif

#include "cpu/mvCpu.h"
#include "nand.h"
#ifdef CONFIG_PCI
#   include <pci.h>
#endif
#include "pci/mvPciRegs.h"

#include <asm/arch/vfpinstr.h>
#include <asm/arch/vfp.h>

#include "net.h"
#include <command.h>
#include <version.h>

/* #define MV_DEBUG */
#ifdef MV_DEBUG
#define DB(x) x
#else
#define DB(x)
#endif

#ifdef CONFIG_BUFFALO_PLATFORM
#include "eth-phy/mvEthPhy.h"
#include "buffalo/BuffaloWol.h"
#include <linux/ctype.h>
#include <ext2fs.h>
  #define SWAP_LONG(x) \
		((__u32)( \
		(((__u32)(x) & (__u32)0x000000ffUL) << 24) | \
		(((__u32)(x) & (__u32)0x0000ff00UL) <<  8) | \
		(((__u32)(x) & (__u32)0x00ff0000UL) >>  8) | \
		(((__u32)(x) & (__u32)0xff000000UL) >> 24) ))
#endif

extern int ext2fs_devread (int sector, int log2blksize, int byte_offset, int byte_len,
			   char *buf);

/* CPU address decode table. */
MV_CPU_DEC_WIN mvCpuAddrWinMap[] = MV_CPU_IF_ADDR_WIN_MAP_TBL;

static void mvHddPowerCtrl(void);

#if (CONFIG_COMMANDS & CFG_CMD_RCVR)
static void recoveryDetection(void);
void recoveryHandle(void);
static u32 rcvrflag = 0;
#endif
void mv_cpu_init(void);
#if defined(MV_INCLUDE_CLK_PWR_CNTRL)
int mv_set_power_scheme(void);
#endif

#ifdef	CFG_FLASH_CFI_DRIVER
MV_VOID mvUpdateNorFlashBaseAddrBank(MV_VOID);
int mv_board_num_flash_banks;
extern flash_info_t	flash_info[]; /* info for FLASH chips */
extern unsigned long flash_add_base_addr (uint flash_index, ulong flash_base_addr);
#endif	/* CFG_FLASH_CFI_DRIVER */

#if defined(MV_INCLUDE_UNM_ETH) || defined(MV_INCLUDE_GIG_ETH)
extern MV_VOID mvBoardEgigaPhySwitchInit(void);
#endif 

#if (CONFIG_COMMANDS & CFG_CMD_NAND)
/* Define for SDK 2.0 */
int __aeabi_unwind_cpp_pr0(int a,int b,int c) {return 0;}
int __aeabi_unwind_cpp_pr1(int a,int b,int c) {return 0;}
#endif

extern nand_info_t nand_info[];       /* info for NAND chips */
MV_VOID mvMppModuleTypePrint(MV_VOID);

#ifdef MV_NAND_BOOT
extern MV_U32 nandEnvBase;
#endif

/* Define for SDK 2.0 */
int raise(void) {return 0;}

#if defined(CONFIG_BUFFALO_PLATFORM)
MV_VOID bfEthPhyPolaritySet(void);
MV_BOOL bfIsStartBootProcess(void);
void bfErrorCodeDisp(MV_32 gppbit, int code);
/*cmd_boot.c*/
extern int do_reset (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);

static block_dev_desc_t *get_dev (char* ifname, int dev)
{
	if (strncmp(ifname,"ide",3)==0) {
		extern block_dev_desc_t * ide_get_dev(int dev);
		return((dev >= CFG_IDE_MAXDEVICE) ? NULL : ide_get_dev(dev));
	}
}

MV_BOOL isValidMacAddr(const char *macAddr)
{
	int i;

	for (i = 0; i < 17; i++) {
		if ((i+1) % 3) {
			if (!isxdigit(*macAddr)) return MV_FALSE;
		}
		else {
			if (*macAddr != ':') return MV_FALSE;
		}
		macAddr++;
	}

	return MV_TRUE;
}

MV_BOOL isMyDisk(int dev, const char *macAddr)
{
	block_dev_desc_t *dev_desc;
	ulong part_length;
	char macAddrOnDisk[] = "00:00:00:00:00:00";
	MV_BOOL ret = MV_FALSE;

	if (!macAddr)
		goto _isMyDisk_end;

	dev_desc = get_dev("ide", dev);
	if (!dev_desc) {
		printf ("\n** Block device %d not supported\n", dev);
		goto _isMyDisk_end;
	}

	if ((part_length = ext2fs_set_blk_dev(dev_desc, 0)) == 0) {
		printf("** Bad partition - %d:%d **\n", dev, 0);
		goto _isMyDisk_end;
	}

	if (!ext2fs_devread(0, 0, 6, 17, macAddrOnDisk)) {
		printf("** Bad ext2 partition or disk - %d:%d **\n", dev, 0);
		goto _isMyDisk_end;
	}

	if (!isValidMacAddr(macAddrOnDisk)) {
		if (!ext2fs_devread(0, 0, 0x28, 17, macAddrOnDisk)) {
			printf ("** Bad ext2 partition or disk - %d:%d **\n", dev, 0);
			goto _isMyDisk_end;
		}
	}

	printf("MAC Address read from block device %d : %s\n", dev, macAddrOnDisk);

	if (!isValidMacAddr(macAddrOnDisk))
		goto _isMyDisk_end;
	
	if (strnicmp(macAddrOnDisk, macAddr, 17) == 0)
		ret = MV_TRUE;

 _isMyDisk_end:
	ext2fs_close();
	return ret;
}


#define KERNEL_LOAD_ADDR	0x00100000
#define INITRD_LOAD_ADDR	0x00800000

MV_STATUS serch_boot_drv(char max_disk)
{
	image_header_t header;
	ulong data, len, checksum, newest_disk_ihtime=0;
	int i ;
	char init_argv[5][120] = {"ext2load","ide","0:1","",""};

	char *init_p[5];
	int newest_fw_dev = -1 ; /*NG*/
	char bootcmd[256] = "";
	char tmpbuf[256];
	char *env;
	char *pAddr;
	char macAddr[] = "00:00:00:00:00:00";
	MV_BOOL my_disk = MV_FALSE;
	MV_BOOL my_disk_found = MV_FALSE;

	env = getenv("initrd");
	if (!env) {
		printf("Getting environment variable 'initrd' failed.\n");
		return MV_FAIL;
	}

	sprintf(init_argv[3], "0x%08x", INITRD_LOAD_ADDR);
	sprintf(init_argv[4], "/%s", env);

	for (i=0; i < 5; i++) {
		init_p[i] = init_argv[i];
	}

	pAddr = getenv("ethaddr");
	if (!pAddr) {
		pAddr = getenv("eth1addr");
		if (!pAddr) {
			pAddr = macAddr;
		}
	}

	printf("MAC Address : %s\n", pAddr);

	for (i = 0 ;i < max_disk ; i++) {
		if (isMyDisk(i, pAddr)) {
			my_disk = MV_TRUE;
		}
		else if (!my_disk_found) {
			my_disk = MV_FALSE;
		}
		else {
			printf("SKIP Loading kernel and initrd\n"
			       "This HDD [block device %d] does not seem to contain my system.\n",
			       i);
			continue;
		}

		sprintf(init_argv[2], "%d:1", i);
		if(do_ext2load(0, 0, 5 , init_p) == 1) {
			DB(printf ("%s (%d)>No init_file %x \n",
					  __FUNCTION__, __LINE__, i));
			continue;
		}

		memmove (&header, (char *)INITRD_LOAD_ADDR, sizeof(image_header_t));
		DB(print_image_hdr((image_header_t *)INITRD_LOAD_ADDR));

		if (header.ih_magic != SWAP_LONG(IH_MAGIC)) {
			DB(printf ("%s (%d)>Bad majic No. %x \n",
					  __FUNCTION__, __LINE__, i));
			continue;
		}

		data = (ulong)&header;
		len  = sizeof(image_header_t);
		
		checksum = SWAP_LONG(header.ih_hcrc);
		header.ih_hcrc = 0;
		if (crc32 (0, (char *)data, len) != checksum) {
			DB(printf("%s (%d)>Bad Header Checksum\n",
					 __FUNCTION__, __LINE__));
			continue;
		}

		data = INITRD_LOAD_ADDR + sizeof(image_header_t);
		len  = SWAP_LONG(header.ih_size);

		if (crc32 (0, (char *)data, len) != SWAP_LONG(header.ih_dcrc)) {
			DB(printf("%s (%d)>Bad Data CRC\n",
					 __FUNCTION__, __LINE__));
			continue;
		}

		if (newest_disk_ihtime < SWAP_LONG(header.ih_time)) {
			if (my_disk) {
				newest_fw_dev =  i;
				newest_disk_ihtime = SWAP_LONG(header.ih_time);
				my_disk_found = MV_TRUE;
			}
			else if (!my_disk && !my_disk_found) {
				newest_fw_dev =  i;
				newest_disk_ihtime = SWAP_LONG(header.ih_time);
			}
		}
		else {
			if (my_disk && !my_disk_found) {
				newest_fw_dev =  i;
				newest_disk_ihtime = SWAP_LONG(header.ih_time);
				my_disk_found = MV_TRUE;
			}
		}
	}

	if (newest_fw_dev == -1)
		return MV_FAIL;

	printf("Booting from Device %d\n", newest_fw_dev);

	env = getenv("initrd");
	if (!env)
		return MV_FAIL;

	sprintf(init_argv[2], "%d:1", newest_fw_dev);
	sprintf(init_argv[3], "0x%08x", INITRD_LOAD_ADDR);
	sprintf(init_argv[4], "/%s", env);

	sprintf(bootcmd, "%s %s %s %s %s; ",
		init_argv[0],
		init_argv[1],
		init_argv[2],
		init_argv[3],
		init_argv[4]);

	env = getenv("kernel");
	if (!env)
		return MV_FAIL;

	sprintf(init_argv[2], "%d:1", newest_fw_dev);
	sprintf(init_argv[3], "0x%08x", KERNEL_LOAD_ADDR);
	sprintf(init_argv[4], "/%s", env);

	sprintf(tmpbuf, "%s %s %s %s %s; ",
		init_argv[0],
		init_argv[1],
		init_argv[2],
		init_argv[3],
		init_argv[4]);

	strcat(bootcmd, tmpbuf);

	strcat(bootcmd, getenv("bootcommon"));
	setenv("idebootcmd", bootcmd);
	DB(printf("** idebootcmd = %s\n", getenv("idebootcmd")));

	return MV_OK ;
}
#endif

void print_mvBanner(void)
{
#ifdef CONFIG_SILENT_CONSOLE
	DECLARE_GLOBAL_DATA_PTR;
	gd->flags |= GD_FLG_SILENT;
#endif
#if defined(CONFIG_BUFFALO_PLATFORM)
#else
	printf("\n");
	printf("         __  __                      _ _\n");
	printf("        |  \\/  | __ _ _ ____   _____| | |\n");
	printf("        | |\\/| |/ _` | '__\\ \\ / / _ \\ | |\n");
	printf("        | |  | | (_| | |   \\ V /  __/ | |\n");
	printf("        |_|  |_|\\__,_|_|    \\_/ \\___|_|_|\n");
	printf(" _   _     ____              _\n");
	printf("| | | |   | __ )  ___   ___ | |_ \n");
	printf("| | | |___|  _ \\ / _ \\ / _ \\| __| \n");
	printf("| |_| |___| |_) | (_) | (_) | |_ \n");
	printf(" \\___/    |____/ \\___/ \\___/ \\__| ");
#endif
#if !defined(MV_NAND_BOOT)
#if defined(MV_INCLUDE_MONT_EXT)
    mvMPPConfigToSPI();
	if(!enaMonExt())
		printf(" ** LOADER **"); 
	else
		printf(" ** MONITOR **");
    mvMPPConfigToDefault();
#else
	printf(" ** Forcing LOADER mode only **"); 
#endif /* MV_INCLUDE_MONT_EXT */
#endif
	return;
}

void print_dev_id(void){
	static char boardName[30];

	mvBoardNameGet(boardName);
	
#if defined(CONFIG_BUFFALO_PLATFORM)
 #if defined(MV_CPU_BE)
	printf("\n ** BUFFALO BOARD: %s BE ",boardName);
 #else
	printf("\n ** BUFFALO BOARD: %s LE ",boardName);
 #endif
	printf("\n ** Board ID: 0x%X", mvBoardIdGet());
#else
 #if defined(MV_CPU_BE)
	printf("\n ** MARVELL BOARD: %s BE ",boardName);
 #else
	printf("\n ** MARVELL BOARD: %s LE ",boardName);
 #endif
#endif
    return;
}


void maskAllInt(void)
{
        /* mask all external interrupt sources */
        MV_REG_WRITE(CPU_MAIN_IRQ_MASK_REG, 0);
        MV_REG_WRITE(CPU_MAIN_FIQ_MASK_REG, 0);
        MV_REG_WRITE(CPU_ENPOINT_MASK_REG, 0);
        MV_REG_WRITE(CPU_MAIN_IRQ_MASK_HIGH_REG, 0);
        MV_REG_WRITE(CPU_MAIN_FIQ_MASK_HIGH_REG, 0);
        MV_REG_WRITE(CPU_ENPOINT_MASK_HIGH_REG, 0);
}

/* init for the Master*/
void misc_init_r_dec_win(void)
{
#if defined(MV_INCLUDE_USB)
	{
		char *env;

		env = getenv("usb0Mode");
		if((!env) || (strcmp(env,"device") == 0) || (strcmp(env,"Device") == 0) )
		{
			printf("USB 0: device mode\n");	
			mvUsbInit(0, MV_FALSE);
		}
		else
		{
			printf("USB 0: host mode\n");	
			mvUsbInit(0, MV_TRUE);
		}
	}
#endif/* #if defined(MV_INCLUDE_USB) */

#if defined(MV_INCLUDE_XOR)
	mvXorInit();
#endif
	mvLcdInit();
#if defined(MV_INCLUDE_CLK_PWR_CNTRL)
	mv_set_power_scheme();
#endif

    return;
}


/*
 * Miscellaneous platform dependent initialisations
 */

extern	MV_STATUS mvEthPhyRegRead(MV_U32 phyAddr, MV_U32 regOffs, MV_U16 *data);
extern	MV_STATUS mvEthPhyRegWrite(MV_U32 phyAddr, MV_U32 regOffs, MV_U16 data);

/* golabal mac address for yukon EC */
unsigned char yuk_enetaddr[6];
extern int interrupt_init (void);
extern void i2c_init(int speed, int slaveaddr);


int board_init (void)
{
	DECLARE_GLOBAL_DATA_PTR;
#if defined(MV_INCLUDE_TWSI)
	MV_TWSI_ADDR slave;
#endif
	unsigned int i;

	maskAllInt();

	/* must initialize the int in order for udelay to work */
	interrupt_init();

#if defined(MV_INCLUDE_TWSI)
	slave.type = ADDR7_BIT;
	slave.address = 0;
	mvTwsiInit(0, CFG_I2C_SPEED, CFG_TCLK, &slave, 0);
#endif
 
	/* Init the Board environment module (device bank params init) */
	mvBoardEnvInit();
   	
	/* Init the Controlloer environment module (MPP init) */
	mvCtrlEnvInit();

	mvBoardDebugLed(3);

        /* Init the Controller CPU interface */
	mvCpuIfInit(mvCpuAddrWinMap);

        /* arch number of Integrator Board */
        gd->bd->bi_arch_number = 527;
 
        /* adress of boot parameters */
        gd->bd->bi_boot_params = 0x00000100;

	/* relocate the exception vectors */
	/* U-Boot is running from DRAM at this stage */
	for(i = 0; i < 0x100; i+=4)
	{
		*(unsigned int *)(0x0 + i) = *(unsigned int*)(TEXT_BASE + i);
	}
	
	/* Update NOR flash base address bank for CFI driver */
#ifdef	CFG_FLASH_CFI_DRIVER
	mvUpdateNorFlashBaseAddrBank();
#endif	/* CFG_FLASH_CFI_DRIVER */

#if defined(MV_INCLUDE_UNM_ETH) || defined(MV_INCLUDE_GIG_ETH)
	/* Init the PHY or Switch of the board */
	mvBoardEgigaPhySwitchInit();
#endif /* #if defined(MV_INCLUDE_UNM_ETH) || defined(MV_INCLUDE_GIG_ETH) */

	mvBoardDebugLed(4);
	
	return 0;
}

void misc_init_r_env(void){
        char *env;
        char tmp_buf[10];
        unsigned int malloc_len;
        DECLARE_GLOBAL_DATA_PTR;
	
        unsigned int flashSize =0 , secSize =0, ubootSize =0;
	char buff[256];
	int need_saveenv=0;

#if defined(MV_BOOTSIZE_4M)
	flashSize = _4M;
#elif defined(MV_BOOTSIZE_8M)
	flashSize = _8M;
#elif defined(MV_BOOTSIZE_16M)
	flashSize = _16M;
#elif defined(MV_BOOTSIZE_32M)
	flashSize = _32M;
#elif defined(MV_BOOTSIZE_64M)
	flashSize = _64M;
#endif

#if defined(MV_SEC_64K)
	secSize = _64K;
#if defined(MV_TINY_IMAGE)
	ubootSize = _256K;
#else
	ubootSize = _512K;
#endif
#elif defined(MV_SEC_128K)
	secSize = _128K;
#if defined(MV_TINY_IMAGE)
	ubootSize = _128K * 3;
#else
	ubootSize = _128K * 5;
#endif
#elif defined(MV_SEC_256K)
	secSize = _256K;
#if defined(MV_TINY_IMAGE)
	ubootSize = _256K * 3;
#else
	ubootSize = _256K * 3;
#endif
#endif
	if ((0 == flashSize) || (0 == secSize) || (0 == ubootSize))
	{
		env = getenv("console");
		if(!env)
			setenv("console","console=ttyS0,115200");
	}
	else
#if defined(MV_SPI_BOOT)
	{
		sprintf(buff,"console=ttyS0,115200 mtdparts=spi_flash:0x%x@0(uboot)ro,0x%x@0x%x(root)",
			 ubootSize, flashSize - 0x100000, 0x100000);
		env = getenv("console");
		if(!env)
			setenv("console",buff);
	}
#elif defined(MV_NAND_BOOT)
	{
		sprintf(buff,"console=ttyS0,115200 mtdparts=nand_mtd:0x%x@0(uboot)ro,0x%x@0x%x(root)",
			 ubootSize, nand_info[0].size - 0x100000, 0x100000);
		env = getenv("console");
		if(!env)
			setenv("console",buff);

		env = getenv("nandEnvBase");
        strcpy(env, "");
        sprintf(env, "%x", nandEnvBase);
        setenv("nandEnvBase", env);
	}
#endif
		
        /* Linux open port support */
	env = getenv("mainlineLinux");
	if(env && ((strcmp(env,"yes") == 0) ||  (strcmp(env,"Yes") == 0)))
		setenv("mainlineLinux","yes");
	else
		setenv("mainlineLinux","no");

	env = getenv("mainlineLinux");
	if(env && ((strcmp(env,"yes") == 0) ||  (strcmp(env,"Yes") == 0)))
	{
	    /* arch number for open port Linux */
	    env = getenv("arcNumber");
	    if(!env )
	    {
		/* arch number according to board ID */
		int board_id = mvBoardIdGet();	
		switch(board_id){
		    case(DB_88F6281A_BP_ID):
			sprintf(tmp_buf,"%d", DB_88F6281_BP_MLL_ID);
			board_id = DB_88F6281_BP_MLL_ID; 
			break;
		    case(RD_88F6192A_ID):
			sprintf(tmp_buf,"%d", RD_88F6192_MLL_ID);
			board_id = RD_88F6192_MLL_ID; 
			break;
		    case(RD_88F6281A_ID):
			sprintf(tmp_buf,"%d", RD_88F6281_MLL_ID);
			board_id = RD_88F6281_MLL_ID; 
			break;
            case(DB_CUSTOMER_ID):
            break;
		    default:
			sprintf(tmp_buf,"%d", board_id);
			board_id = board_id; 
			break;
		}
		gd->bd->bi_arch_number = board_id;
		setenv("arcNumber", tmp_buf);
	    }
	    else
	    {
		gd->bd->bi_arch_number = simple_strtoul(env, NULL, 10);
	    }
	}

        /* update the CASset env parameter */
        env = getenv("CASset");
        if(!env )
        {
#ifdef MV_MIN_CAL
                setenv("CASset","min");
#else
                setenv("CASset","max");
#endif
        }
        /* Monitor extension */
#ifdef MV_INCLUDE_MONT_EXT
        env = getenv("enaMonExt");
        if(/* !env || */ ( (strcmp(env,"yes") == 0) || (strcmp(env,"Yes") == 0) ) )
                setenv("enaMonExt","yes");
        else
#endif
                setenv("enaMonExt","no");
#if defined (MV_INC_BOARD_NOR_FLASH)
        env = getenv("enaFlashBuf");
        if( ( (strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) ) )
                setenv("enaFlashBuf","no");
        else
                setenv("enaFlashBuf","yes");
#endif

	/* CPU streaming */
        env = getenv("enaCpuStream");
        if(!env || ( (strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) ) )
                setenv("enaCpuStream","no");
        else
                setenv("enaCpuStream","yes");

	/* Write allocation */
	env = getenv("enaWrAllo");
	if( !env || ( ((strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) )))
		setenv("enaWrAllo","no");
	else
		setenv("enaWrAllo","yes");

	/* Pex mode */
	env = getenv("pexMode");
	if( env && ( ((strcmp(env,"EP") == 0) || (strcmp(env,"ep") == 0) )))
		setenv("pexMode","EP");
	else
		setenv("pexMode","RC");

    	env = getenv("disL2Cache");
    	if(!env || ( (strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) ) )
        	setenv("disL2Cache","no"); 
    	else
        	setenv("disL2Cache","yes");

    	env = getenv("setL2CacheWT");
    	if(!env || ( (strcmp(env,"yes") == 0) || (strcmp(env,"Yes") == 0) ) )
        	setenv("setL2CacheWT","yes"); 
    	else
        	setenv("setL2CacheWT","no");

    	env = getenv("disL2Prefetch");
    	if(!env || ( (strcmp(env,"yes") == 0) || (strcmp(env,"Yes") == 0) ) )
	{
        	setenv("disL2Prefetch","yes"); 
	
		/* ICache Prefetch */
		env = getenv("enaICPref");
		if( env && ( ((strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) )))
			setenv("enaICPref","no");
		else
			setenv("enaICPref","yes");
		
		/* DCache Prefetch */
		env = getenv("enaDCPref");
		if( env && ( ((strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) )))
			setenv("enaDCPref","no");
		else
			setenv("enaDCPref","yes");
	}
    	else
	{
        	setenv("disL2Prefetch","no");
		setenv("enaICPref","no");
		setenv("enaDCPref","no");
	}


        env = getenv("sata_dma_mode");
        if( env && ((strcmp(env,"No") == 0) || (strcmp(env,"no") == 0) ) )
                setenv("sata_dma_mode","no");
        else
                setenv("sata_dma_mode","yes");


        /* Malloc length */
        env = getenv("MALLOC_len");
        malloc_len =  simple_strtoul(env, NULL, 10) << 20;
        if(malloc_len == 0){
                sprintf(tmp_buf,"%d",CFG_MALLOC_LEN>>20);
                setenv("MALLOC_len",tmp_buf);
	}
         
        /* primary network interface */
#if !defined(CONFIG_BUFFALO_PLATFORM)
        env = getenv("ethprime");
	if(!env)
    {
        if(mvBoardIdGet() == RD_88F6281A_ID)
            setenv("ethprime","egiga1");
        else
            setenv("ethprime",ENV_ETH_PRIME);
    }
 
	/* netbsd boot arguments */
        env = getenv("netbsd_en");
	if( !env || ( ((strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) )))
		setenv("netbsd_en","no");
	else
	{
	    setenv("netbsd_en","yes");
	    env = getenv("netbsd_gw");
	    if(!env)
                setenv("netbsd_gw","192.168.0.254");

	    env = getenv("netbsd_mask");
	    if(!env)
                setenv("netbsd_mask","255.255.255.0");

	    env = getenv("netbsd_fs");
	    if(!env)
                setenv("netbsd_fs","nfs");

	    env = getenv("netbsd_server");
	    if(!env)
                setenv("netbsd_server","192.168.0.1");

	    env = getenv("netbsd_ip");
	    if(!env)
	    {
		env = getenv("ipaddr");
               	setenv("netbsd_ip",env);
	    }

	    env = getenv("netbsd_rootdev");
	    if(!env)
                setenv("netbsd_rootdev","mgi0");

	    env = getenv("netbsd_add");
	    if(!env)
                setenv("netbsd_add","0x800000");

	    env = getenv("netbsd_get");
	    if(!env)
                setenv("netbsd_get","tftpboot $(netbsd_add) $(image_name)");

#if defined(MV_INC_BOARD_QD_SWITCH)
	    env = getenv("netbsd_netconfig");
	    if(!env)
	    setenv("netbsd_netconfig","mv_net_config=<((mgi0,00:00:11:22:33:44,0)(mgi1,00:00:11:22:33:55,1:2:3:4)),mtu=1500>");
#endif
	    env = getenv("netbsd_set_args");
	    if(!env)
	    	setenv("netbsd_set_args","setenv bootargs nfsroot=$(netbsd_server):$(rootpath) fs=$(netbsd_fs) \
ip=$(netbsd_ip) serverip=$(netbsd_server) mask=$(netbsd_mask) gw=$(netbsd_gw) rootdev=$(netbsd_rootdev) \
ethaddr=$(ethaddr) eth1addr=$(eth1addr) ethmtu=$(ethmtu) eth1mtu=$(eth1mtu) $(netbsd_netconfig)");

	    env = getenv("netbsd_boot");
	    if(!env)
	    	setenv("netbsd_boot","bootm $(netbsd_add) $(bootargs)");

	    env = getenv("netbsd_bootcmd");
	    if(!env)
	    	setenv("netbsd_bootcmd","run netbsd_get ; run netbsd_set_args ; run netbsd_boot");
	}

	/* vxWorks boot arguments */
        env = getenv("vxworks_en");
	if( !env || ( ((strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) )))
		setenv("vxworks_en","no");
	else
	{
	    char* buff = 0x1100;
	    setenv("vxworks_en","yes");
		
	    sprintf(buff,"mgi(0,0) host:vxWorks.st");
	    env = getenv("serverip");
	    strcat(buff, " h=");
	    strcat(buff,env);
	    env = getenv("ipaddr");
	    strcat(buff, " e=");
	    strcat(buff,env);
	    strcat(buff, ":ffff0000 u=anonymous pw=target ");

	    setenv("vxWorks_bootargs",buff);
	}
#endif // !defined(CONFIG_BUFFALO_PLATFORM)
        /* linux boot arguments */
        env = getenv("bootargs_root");
        if(!env)
#if defined(CONFIG_BUFFALO_PLATFORM)
		setenv("bootargs_root", "root=/dev/sda2 rw initrd=0x00800040,12M panic=5");
#else
                setenv("bootargs_root","root=/dev/nfs rw");
#endif	// (CONFIG_BUFFALO_PLATFORM)

	/* For open Linux we set boot args differently */
	env = getenv("mainlineLinux");
	if(env && ((strcmp(env,"yes") == 0) ||  (strcmp(env,"Yes") == 0)))
	{
        	env = getenv("bootargs_end");
        	if(!env)
                setenv("bootargs_end",":::orion:eth0:none");
	}
	else
	{
        	env = getenv("bootargs_end");
        	if(!env)
#if defined(MV_INC_BOARD_QD_SWITCH)
                setenv("bootargs_end",CFG_BOOTARGS_END_SWITCH);
#else
                setenv("bootargs_end",CFG_BOOTARGS_END);
#endif
	}
	
        env = getenv("image_name");
        if(!env)
                setenv("image_name","uImage");
 
#if defined(CONFIG_BUFFALO_PLATFORM)
	extern char buffalo_version_string[];
	setenv("buffalo_ver", buffalo_version_string);
	setenv("kernel", "uImage.buffalo");
	setenv("initrd", "initrd.buffalo");
	setenv("bootcommon",
	       "setenv bootargs $console $bootargs_root $bootargs_func $bootargs_wol $buffalo_ver $mtdparts $tftpboot $nandboot; "
	       "bootm 0x00100000 0x00800000");

	setenv("tftpbootcmd",
	       "tftp 0x00100000 $kernel; tftp 0x00800000 $initrd; "
	       "setenv tftpboot tftpboot=yes; "
	       "run bootcommon");

	setenv("idebootcmd",
	       "ext2load ide 0:1 0x00100000 /$kernel; "
	       "ext2load ide 0:1 0x00800000 /$initrd; "
	       "run bootcommon");

# if defined(MV_NAND)
	setenv("mtdids", "nand0=nand_mtd");
	sprintf(buff,"mtdparts=nand_mtd:0x%x(boot),0x%x(rootfs),0x%x(reserve)",
		0x01000000,
		nand_info[0].size - 0x01000000 - 0x00800000,
		0x00800000);
	setenv("mtdparts", buff);
	setenv("nandbootcmd",
	       "fsload 0x00100000 /$kernel; "
	       "fsload 0x00800000 /$initrd; "
	       "setenv nandboot nandboot=yes; "
	       "run bootcommon");

	extern nand_info_t nand_info[];
	sprintf(buff, "ide");
	if (nand_info[0].name)
		strcat(buff, " nand");
# else
	sprintf(buff, "ide");
# endif // defined(MV_NAND)

	setenv("bootorder", buff);
	setenv("bootcmd",
	       "for i in $bootorder; do run ${i}bootcmd; done");

#else // !defined(CONFIG_BUFFALO_PLATFORM)

#if (CONFIG_BOOTDELAY >= 0)
        env = getenv("bootcmd");
        if(!env)
#if defined(MV_INCLUDE_TDM) && defined(MV_INC_BOARD_QD_SWITCH)
	    setenv("bootcmd","tftpboot 0x2000000 $(image_name); \
setenv bootargs $(console) $(bootargs_root) nfsroot=$(serverip):$(rootpath) \
ip=$(ipaddr):$(serverip)$(bootargs_end) $(mvNetConfig) $(mvPhoneConfig) $(lcd0_bootargs);  bootm 0x2000000; ");
#elif defined(MV_INC_BOARD_QD_SWITCH)
            setenv("bootcmd","tftpboot 0x2000000 $(image_name); \
setenv bootargs $(console) $(bootargs_root) nfsroot=$(serverip):$(rootpath) \
ip=$(ipaddr):$(serverip)$(bootargs_end) $(mvNetConfig) $(lcd0_bootargs);  bootm 0x2000000; ");
#elif defined(MV_INCLUDE_TDM)
            setenv("bootcmd","tftpboot 0x2000000 $(image_name); \
setenv bootargs $(console) $(bootargs_root) nfsroot=$(serverip):$(rootpath) \
ip=$(ipaddr):$(serverip)$(bootargs_end) $(mvNetConfig) $(mvPhoneConfig) $(lcd0_bootargs);  bootm 0x2000000; ");
#else

            setenv("bootcmd","tftpboot 0x2000000 $(image_name); \
setenv bootargs $(console) $(bootargs_root) nfsroot=$(serverip):$(rootpath) \
ip=$(ipaddr):$(serverip)$(bootargs_end) $(lcd0_bootargs);  bootm 0x2000000; ");
#endif
#endif /* (CONFIG_BOOTDELAY >= 0) */

        env = getenv("standalone");
        if(!env)
#if defined(MV_INCLUDE_TDM) && defined(MV_INC_BOARD_QD_SWITCH)
	    setenv("standalone","fsload 0x2000000 $(image_name);setenv bootargs $(console) root=/dev/mtdblock0 rw \
ip=$(ipaddr):$(serverip)$(bootargs_end) $(mvNetConfig) $(mvPhoneConfig); bootm 0x2000000;");
#elif defined(MV_INC_BOARD_QD_SWITCH)
            setenv("standalone","fsload 0x2000000 $(image_name);setenv bootargs $(console) root=/dev/mtdblock0 rw \
ip=$(ipaddr):$(serverip)$(bootargs_end) $(mvNetConfig); bootm 0x2000000;");
#elif defined(MV_INCLUDE_TDM)
            setenv("standalone","fsload 0x2000000 $(image_name);setenv bootargs $(console) root=/dev/mtdblock0 rw \
ip=$(ipaddr):$(serverip)$(bootargs_end) $(mvPhoneConfig); bootm 0x2000000;");
#else
            setenv("standalone","fsload 0x2000000 $(image_name);setenv bootargs $(console) root=/dev/mtdblock0 rw \
ip=$(ipaddr):$(serverip)$(bootargs_end); bootm 0x2000000;");
#endif
#endif // (CONFIG_BUFFALO_PLATFORM)


	/* Add LCD param to Linux */
	if (mvCtrlModelGet() == MV_6282_DEV_ID)
	{
        	env = getenv("lcd0_enable");
        	if(!env)
	    		setenv("lcd0_enable","0");

        	env = getenv("lcd0_params");
        	if(!env)
	    		setenv("lcd0_params",ENV_LCD0_PARAMS);

        	env = getenv("lcd0_bootargs");
        	if(!env)
	    		setenv("lcd0_bootargs",ENV_LCD_BOOTARGS);
	}

                 
       /* Set boodelay to 3 sec, if Monitor extension are disabled */
        if(!enaMonExt()){
                setenv("bootdelay","3");
		setenv("disaMvPnp","no");
	}

	/* Disable PNP config of Marvel memory controller devices. */
        env = getenv("disaMvPnp");
        if(!env)
                setenv("disaMvPnp","no");

#if (defined(MV_INCLUDE_GIG_ETH) || defined(MV_INCLUDE_UNM_ETH))
	/* Generate random ip and mac address */
	/* Read RTC to create pseudo-random data for enc */
    struct rtc_time tm;
	unsigned int xi, xj, xk, xl, i;
	char ethaddr_0[30];
	char ethaddr_1[30];

    rtc_get(&tm);
	xi = ((tm.tm_yday + tm.tm_sec)% 254);
	/* No valid ip with one of the fileds has the value 0 */
	if (xi == 0)
		xi+=2;

	xj = ((tm.tm_yday + tm.tm_min)%254);
	/* No valid ip with one of the fileds has the value 0 */
	if (xj == 0)
		xj+=2;

	/* Check if the ip address is the same as the server ip */
	if ((xj == 1) && (xi == 11))
		xi+=2;

	xk = (tm.tm_min * tm.tm_sec)%254;
	xl = (tm.tm_hour * tm.tm_sec)%254;

	sprintf(ethaddr_0,"00:50:43:%02x:%02x:%02x",xk,xi,xj);
	sprintf(ethaddr_1,"00:50:43:%02x:%02x:%02x",xl,xi,xj);

	/* MAC addresses */
        env = getenv("ethaddr");
        if(!env) {
                setenv("ethaddr",ethaddr_0);
		need_saveenv = 1;
	}

        env = getenv("ethmtu");
        if(!env)
                setenv("ethmtu","1500");
        
#if !defined(MV_INC_BOARD_QD_SWITCH)
/* ETH1ADDR not define in GWAP boards */
	if ((mvBoardMppGroupTypeGet(MV_BOARD_MPP_GROUP_1) == MV_BOARD_RGMII) || 
			(mvBoardMppGroupTypeGet(MV_BOARD_MPP_GROUP_2) == MV_BOARD_RGMII) || 
			(mvBoardMppGroupTypeGet(MV_BOARD_MPP_GROUP_1) == MV_BOARD_MII))
	{
        	env = getenv("eth1addr");
        	if(!env)
               		setenv("eth1addr",ethaddr_1);

            env = getenv("eth1mtu");
            if(!env)
                    setenv("eth1mtu","1500");
	}
#elif defined(MV_INC_BOARD_QD_SWITCH) && (defined(RD_88F6190A) || defined(RD_88F6192A))
    env = getenv("eth1addr");
    if(!env)
            setenv("eth1addr",ethaddr_1);

    env = getenv("eth1mtu");
    if(!env)
            setenv("eth1mtu","1500");
#endif
#if defined(MV_INCLUDE_TDM)
        /* Set mvPhoneConfig env parameter */
        env = getenv("mvPhoneConfig");
        if(!env )
            setenv("mvPhoneConfig","mv_phone_config=dev[0]:fxs,dev[1]:fxo");
#endif
        /* Set mvNetConfig env parameter */
        env = getenv("mvNetConfig");
        if(!env )
		    setenv("mvNetConfig","mv_net_config=(00:11:88:0f:62:81,0:1:2:3),mtu=1500");
#endif /*  (MV_INCLUDE_GIG_ETH) || defined(MV_INCLUDE_UNM_ETH) */

#if defined(MV_INCLUDE_USB)
	/* USB Host */
	env = getenv("usb0Mode");
	if(!env)
		setenv("usb0Mode",ENV_USB0_MODE);
#endif  /* (MV_INCLUDE_USB) */

#if defined(YUK_ETHADDR)
	env = getenv("yuk_ethaddr");
	if(!env)
		setenv("yuk_ethaddr",YUK_ETHADDR);

	{
		int i;
		char *tmp = getenv ("yuk_ethaddr");
		char *end;

		for (i=0; i<6; i++) {
			yuk_enetaddr[i] = tmp ? simple_strtoul(tmp, &end, 16) : 0;
			if (tmp)
				tmp = (*end) ? end+1 : end;
		}
	}
#endif /* defined(YUK_ETHADDR) */

#if defined(MV_NAND)
	env = getenv("nandEcc");
	if(!env)
    {
        setenv("nandEcc", "1bit");
    }
#endif

#if defined(RD_88F6281A) || defined(RD_88F6192A) || defined(RD_88F6190A)
	mvHddPowerCtrl();
#endif
#if (CONFIG_COMMANDS & CFG_CMD_RCVR)
	env = getenv("netretry");
	if (!env)
		setenv("netretry","no");

	env = getenv("rcvrip");
	if (!env)
		setenv("rcvrip",RCVR_IP_ADDR);

	env = getenv("loadaddr");
	if (!env)
		setenv("loadaddr",RCVR_LOAD_ADDR);

	env = getenv("autoload");
	if (!env)
		setenv("autoload","no");

	/* Check the recovery trigger */
	recoveryDetection();
#endif
	if (need_saveenv)
		saveenv();

        return;
}

#ifdef BOARD_LATE_INIT
int board_late_init (void)
{
#if defined(CONFIG_BUFFALO_PLATFORM)
	bfEthPhyPolaritySet();
	while (!bfIsStartBootProcess())
		;

	buffalo_all_link_led_on();
	bfGppBlinkRegBitSet(BIT_PWR_LED);
	bfGppOutRegBitAssert(BIT_PWR_LED);
	bfGppOutRegBitAssert(BIT_FAN_LOW);
	bfGppOutRegBitAssert(BIT_FAN_HIGH);

	int i;
	MV_32 pin;

	for (i=0; (pin = mvBoardGpioPinNumGet(BOARD_GPP_HDD_POWER, i)) != MV_ERROR; i++) {
		printf("HDD%d Power ON\n", i);
		bfGppOutRegBitAssert(pin);
		udelay(5 * 1000 * 1000);
	}

#if 0
	for (i=0; (pin = mvBoardGpioPinNumGet(BOARD_GPP_USB_VBUS_EN, i)) != MV_ERROR; i++) {
		printf("USB%d Power ON\n", i);
		bfGppOutRegBitAssert(pin);
		udelay(5 * 1000 * 1000);
	}
	
	/* Initialize USB */
	run_command("usb reset",0);
	char *argv[] = {"fatinfo", "usb", "0"};
	extern int do_fat_fsinfo(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
	if (do_fat_fsinfo(NULL, 0, 3, argv) == 0)
		setenv("usbloadcmd", "fatload");
	else
		setenv("usbloadcmd", "ext2load");

	if (!getenv("usbbootcmd"))
		setenv("usbbootcmd",
		       "$(usbloadcmd) usb 0:1 0x00100000 /$(kernel); "
		       "$(usbloadcmd) usb 0:1 0x00800000 /$(initrd); "
		       "setenv bootargs $(console) $(bootargs_root) $(buffalo_ver) ;"
		       "bootm 0x00100000 0x00800000");
#endif
	extern MV_BOOL hdd_found;
	ide_init();
	if (hdd_found == MV_FALSE) {
		printf("HDD is not found \n");
#if defined(MV_NAND)
		if (nand_info[0].size == 0) {
			bfErrorCodeDisp(BIT_ALARM_LED, 7);
			setenv("force_tftp", "1");
		}
#else
		bfErrorCodeDisp(BIT_ALARM_LED, 7);
		setenv("force_tftp", "1");
#endif
	}
	else {
		int max_drv;

		switch (mvBoardIdGet()) {
		case BF_MVLSQV_ID:
			max_drv = 4;
			break;
		case BF_MVWXL_ID:
		case BF_MVWSSX_ID:
		case BF_MVLSWV_ID:
			max_drv = 2;
			break;
		case BF_MVLSXH_ID:
		case BF_MVLSXL_ID:
		case BF_MVLSVL_ID:
		default:
			max_drv = 1;
			break;
		}

		if (serch_boot_drv(max_drv) == MV_FAIL) {
		}
	}

	if(bfIsSupportWol(0) == MV_TRUE)
	{
		if (bfGppInRegBitTest(BIT_PWRAUTO_SW))
		{
			MV_U16 wol_status = bfGetWolInterruptStatus(0, 0);
			if(wol_status)
			{
				char sz_bootargs_wol[128];
				memset(sz_bootargs_wol, 0, sizeof(sz_bootargs_wol));

				sprintf(sz_bootargs_wol, "wol_status=%s", bfGetWolPatternString(wol_status));
				setenv("bootargs_wol", sz_bootargs_wol);
			}
		}
		bfControlWolInterrupt(0, FLAG_BUFFALO_WOL_INTERRUPT_DISABLE);
		bfResetEthPhy(0);
	}
#else
	/* Check if to use the LED's for debug or to use single led for init and Linux heartbeat */
	mvBoardDebugLed(0);
#endif // CONFIG_BUFFALO_PLATFORM
	return 0;
}
#endif // BOARD_LATE_INIT

void pcie_tune(void)
{

  MV_REG_WRITE(0xF1041AB0, 0x100);
  MV_REG_WRITE(0xF1041A20, 0x78000801);
  MV_REG_WRITE(0xF1041A00, 0x4014022F);
  MV_REG_WRITE(0xF1040070, 0x18110008);

  return;

}

int misc_init_r (void)
{
	char name[128], *env;
	
	mvBoardDebugLed(5);

	mvCpuNameGet(name);
	printf("\nCPU : %s\n",  name);

	/* init special env variables */
	misc_init_r_env();

	mv_cpu_init();

#if defined(MV_INCLUDE_MONT_EXT)
	if(enaMonExt()){
			printf("\n      Marvell monitor extension:\n");
			mon_extension_after_relloc();
	}
    	printf("\n");		
#endif /* MV_INCLUDE_MONT_EXT */

	/* print detected modules */
	mvMppModuleTypePrint();

    	printf("\n");		
	/* init the units decode windows */
	misc_init_r_dec_win();

#ifdef CONFIG_PCI
#if !defined(MV_MEM_OVER_PCI_WA) && !defined(MV_MEM_OVER_PEX_WA)
       pci_init();
#endif
#endif

	mvBoardDebugLed(6);

	mvBoardDebugLed(7);

	env = getenv("pcieTune");
	if(env && ((strcmp(env,"yes") == 0) || (strcmp(env,"yes") == 0)))
        pcie_tune();
    else
        setenv("pcieTune","no");
	
	return 0;
}

MV_U32 mvTclkGet(void)
{
        DECLARE_GLOBAL_DATA_PTR;
        /* get it only on first time */
        if(gd->tclk == 0)
                gd->tclk = mvBoardTclkGet();

        return gd->tclk;
}

MV_U32 mvSysClkGet(void)
{
        DECLARE_GLOBAL_DATA_PTR;
        /* get it only on first time */
        if(gd->bus_clk == 0)
                gd->bus_clk = mvBoardSysClkGet();

        return gd->bus_clk;
}
 
#ifndef MV_TINY_IMAGE
/* exported for EEMBC */
MV_U32 mvGetRtcSec(void)
{
        MV_RTC_TIME time;
#ifdef MV_INCLUDE_RTC
        mvRtcTimeGet(&time);
#elif CONFIG_RTC_DS1338_DS1339
        mvRtcDS133xTimeGet(&time);
#endif
	return (time.minutes * 60) + time.seconds;
}
#endif

void reset_cpu(void)
{
	mvBoardReset();
}

void mv_cpu_init(void)
{
	char *env;
	volatile unsigned int temp;

	/*CPU streaming & write allocate */
	env = getenv("enaWrAllo");
	if(env && ((strcmp(env,"yes") == 0) ||  (strcmp(env,"Yes") == 0)))  
	{
		__asm__ __volatile__("mrc    p15, 1, %0, c15, c1, 0" : "=r" (temp));
		temp |= BIT28;
		__asm__ __volatile__("mcr    p15, 1, %0, c15, c1, 0" :: "r" (temp));
		
	}
	else
	{
		__asm__ __volatile__("mrc    p15, 1, %0, c15, c1, 0" : "=r" (temp));
		temp &= ~BIT28;
		__asm__ __volatile__("mcr    p15, 1, %0, c15, c1, 0" :: "r" (temp));
	}

	env = getenv("enaCpuStream");
	if(!env || (strcmp(env,"no") == 0) ||  (strcmp(env,"No") == 0) )
	{
		__asm__ __volatile__("mrc    p15, 1, %0, c15, c1, 0" : "=r" (temp));
		temp &= ~BIT29;
		__asm__ __volatile__("mcr    p15, 1, %0, c15, c1, 0" :: "r" (temp));
	}
	else
	{
		__asm__ __volatile__("mrc    p15, 1, %0, c15, c1, 0" : "=r" (temp));
		temp |= BIT29;
		__asm__ __volatile__("mcr    p15, 1, %0, c15, c1, 0" :: "r" (temp));
	}
		
	/* Verifay write allocate and streaming */
	printf("\n");
	__asm__ __volatile__("mrc    p15, 1, %0, c15, c1, 0" : "=r" (temp));
	if (temp & BIT29)
		printf("Streaming enabled \n");
	else
		printf("Streaming disabled \n");
	if (temp & BIT28)
		printf("Write allocate enabled\n");
	else
		printf("Write allocate disabled\n");

	/* DCache Pref  */
	env = getenv("enaDCPref");
	if(env && ((strcmp(env,"yes") == 0) ||  (strcmp(env,"Yes") == 0)))
	{
		MV_REG_BIT_SET( CPU_CONFIG_REG , CCR_DCACH_PREF_BUF_ENABLE);
	}

	if(env && ((strcmp(env,"no") == 0) ||  (strcmp(env,"No") == 0)))
        {
		MV_REG_BIT_RESET( CPU_CONFIG_REG , CCR_DCACH_PREF_BUF_ENABLE);
	}

	/* ICache Pref  */
	env = getenv("enaICPref");
        if(env && ((strcmp(env,"yes") == 0) ||  (strcmp(env,"Yes") == 0)))
	{
		MV_REG_BIT_SET( CPU_CONFIG_REG , CCR_ICACH_PREF_BUF_ENABLE);
	}
	
	if(env && ((strcmp(env,"no") == 0) ||  (strcmp(env,"No") == 0)))
        {
		MV_REG_BIT_RESET( CPU_CONFIG_REG , CCR_ICACH_PREF_BUF_ENABLE);
	}

	/* Set L2C WT mode - Set bit 4 */
	temp = MV_REG_READ(CPU_L2_CONFIG_REG);
    	env = getenv("setL2CacheWT");
    	if(!env || ( (strcmp(env,"yes") == 0) || (strcmp(env,"Yes") == 0) ) )
	{
		temp |= BIT4;
	}
	else
		temp &= ~BIT4;
	MV_REG_WRITE(CPU_L2_CONFIG_REG, temp);


	/* L2Cache settings */
	asm ("mrc p15, 1, %0, c15, c1, 0":"=r" (temp));

	/* Disable L2C pre fetch - Set bit 24 */
    	env = getenv("disL2Prefetch");
    	if(env && ( (strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) ) )
		temp &= ~BIT24;
	else
		temp |= BIT24;

	/* enable L2C - Set bit 22 */
    	env = getenv("disL2Cache");
    	if(!env || ( (strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) ) )
		temp |= BIT22;
	else
		temp &= ~BIT22;
	
	asm ("mcr p15, 1, %0, c15, c1, 0": :"r" (temp));


	/* Enable i cache */
	asm ("mrc p15, 0, %0, c1, c0, 0":"=r" (temp));
	temp |= BIT12;
	asm ("mcr p15, 0, %0, c1, c0, 0": :"r" (temp));

	/* Change reset vector to address 0x0 */
	asm ("mrc p15, 0, %0, c1, c0, 0":"=r" (temp));
	temp &= ~BIT13;
	asm ("mcr p15, 0, %0, c1, c0, 0": :"r" (temp));
}
/*******************************************************************************
* mvBoardMppModuleTypePrint - print module detect
*
* DESCRIPTION:
*
* INPUT:
*
* OUTPUT:
*       None.
*
* RETURN:
*
*******************************************************************************/
MV_VOID mvMppModuleTypePrint(MV_VOID)
{

	MV_BOARD_MPP_GROUP_CLASS devClass;
	MV_BOARD_MPP_TYPE_CLASS mppGroupType;
	MV_U32 devId;
	MV_U32 maxMppGrp = 1;
	
	devId = mvCtrlModelGet();

	switch(devId){
		case MV_6281_DEV_ID:
			maxMppGrp = MV_6281_MPP_MAX_MODULE;
			break;
        case MV_6282_DEV_ID:
            maxMppGrp = MV_6282_MPP_MAX_MODULE;
            break;
        case MV_6280_DEV_ID:
            maxMppGrp = MV_6280_MPP_MAX_MODULE;
            break;
		case MV_6192_DEV_ID:
			maxMppGrp = MV_6192_MPP_MAX_MODULE;
			break;
        case MV_6190_DEV_ID:
            maxMppGrp = MV_6190_MPP_MAX_MODULE;
            break;
		case MV_6180_DEV_ID:
			maxMppGrp = MV_6180_MPP_MAX_MODULE;
			break;		
	}

	for (devClass = 0; devClass < maxMppGrp; devClass++)
	{
		mppGroupType = mvBoardMppGroupTypeGet(devClass);
		
		/* 6282 doesn't support LCD module when booting from NAND */
		if ((mppGroupType == MV_BOARD_LCD) && (mvCtrlIsBootFromNAND()))
		{
		    printf("Error!, MV88F6282 doesn't support LCD module when booting from NAND!\n");
		    break;
		}

		switch(mppGroupType)
		{
			case MV_BOARD_TDM:
                if(devId != MV_6190_DEV_ID)
                    printf("Module %d is TDM\n", devClass);
				break;
			case MV_BOARD_AUDIO:
                if(devId != MV_6190_DEV_ID)
                    printf("Module %d is AUDIO\n", devClass);
				break;
			case MV_BOARD_RGMII:
                if(devId != MV_6190_DEV_ID)
                    printf("Module %d is RGMII\n", devClass);
				break;
			case MV_BOARD_GMII:
                if(devId != MV_6190_DEV_ID)
                    printf("Module %d is GMII\n", devClass);
				break;
			case MV_BOARD_TS:
                if(devId != MV_6190_DEV_ID)
                    printf("Module %d is TS\n", devClass);
				break;
			case MV_BOARD_MII:
                if(devId != MV_6190_DEV_ID)
                    printf("Module %d is MII\n", devClass);
				break;
			case MV_BOARD_LCD:
                		if(devId != MV_6190_DEV_ID)
                    		printf("Module %d is LCD\n", devClass);
				break;

			default:
				break;
		}
	}
}

/* Set unit in power off mode acording to the detection of MPP */
#if defined(MV_INCLUDE_CLK_PWR_CNTRL)
int mv_set_power_scheme(void)
{
	int mppGroupType1 = mvBoardMppGroupTypeGet(MV_BOARD_MPP_GROUP_1);
	int mppGroupType2 = mvBoardMppGroupTypeGet(MV_BOARD_MPP_GROUP_2);
	MV_U32 devId = mvCtrlModelGet();
	MV_U32 boardId = mvBoardIdGet();

    if (boardId == RD_88F6192A_ID)
    {
		/* Sdio power down */
		mvCtrlPwrMemSet(SDIO_UNIT_ID, 0, MV_FALSE);
		mvCtrlPwrClckSet(SDIO_UNIT_ID, 0, MV_FALSE);
    }

    if (devId == MV_6180_DEV_ID || boardId == RD_88F6281A_PCAC_ID || devId == MV_6280_DEV_ID || boardId == SHEEVA_PLUG_ID)
	{
		/* Sata power down */
		mvCtrlPwrMemSet(SATA_UNIT_ID, 1, MV_FALSE);
		mvCtrlPwrMemSet(SATA_UNIT_ID, 0, MV_FALSE);
		mvCtrlPwrClckSet(SATA_UNIT_ID, 1, MV_FALSE);
		mvCtrlPwrClckSet(SATA_UNIT_ID, 0, MV_FALSE);
		/* Sdio power down */
		mvCtrlPwrMemSet(SDIO_UNIT_ID, 0, MV_FALSE);
		mvCtrlPwrClckSet(SDIO_UNIT_ID, 0, MV_FALSE);
	}

    if (boardId == RD_88F6281A_ID || boardId == SHEEVA_PLUG_ID || devId == MV_6280_DEV_ID)
    {
		DB(printf("Warning: TS is Powered Off\n"));
		mvCtrlPwrClckSet(TS_UNIT_ID, 0, MV_FALSE);
    }

    if (devId == MV_6280_DEV_ID)
    {
		DB(printf("Warning: PCI-E is Powered Off\n"));
		mvCtrlPwrClckSet(PEX_UNIT_ID, 0, MV_FALSE);
    }
	
	/* Close egiga 1 */
	if ((mppGroupType1 != MV_BOARD_GMII) && (mppGroupType1 != MV_BOARD_RGMII) && (mppGroupType2 != MV_BOARD_RGMII)
		 && (mppGroupType1 != MV_BOARD_MII))
	{
		DB(printf("Warning: Giga1 is Powered Off\n"));
		mvCtrlPwrMemSet(ETH_GIG_UNIT_ID, 1, MV_FALSE);
		mvCtrlPwrClckSet(ETH_GIG_UNIT_ID, 1, MV_FALSE);
	}

	/* Close TDM */
	if ((mppGroupType1 != MV_BOARD_TDM) && (mppGroupType2 != MV_BOARD_TDM))
	{
		DB(printf("Warning: TDM is Powered Off\n"));
		mvCtrlPwrClckSet(TDM_UNIT_ID, 0, MV_FALSE);
	}

	/* Close AUDIO */
	if ((mppGroupType1 != MV_BOARD_AUDIO) && (mppGroupType2 != MV_BOARD_AUDIO) && boardId != RD_88F6281A_ID)
	{
		DB(printf("Warning: AUDIO is Powered Off\n"));
		mvCtrlPwrMemSet(AUDIO_UNIT_ID, 0, MV_FALSE);
		mvCtrlPwrClckSet(AUDIO_UNIT_ID, 0, MV_FALSE);
	}

	/* Close TS */
	if ((mppGroupType1 != MV_BOARD_TS) && (mppGroupType2 != MV_BOARD_TS))
	{
		DB(printf("Warning: TS is Powered Off\n"));
		mvCtrlPwrClckSet(TS_UNIT_ID, 0, MV_FALSE);
	}

	return MV_OK;
}

#endif /* defined(MV_INCLUDE_CLK_PWR_CNTRL) */

/*******************************************************************************
* mvUpdateNorFlashBaseAddrBank - 
*
* DESCRIPTION:
*       This function update the CFI driver base address bank with on board NOR
*       devices base address.
*
* INPUT:
*
* OUTPUT:
*
* RETURN:
*       None
*
*******************************************************************************/
#ifdef	CFG_FLASH_CFI_DRIVER
MV_VOID mvUpdateNorFlashBaseAddrBank(MV_VOID)
{
    
    MV_U32 devBaseAddr;
    MV_U32 devNum = 0;
    int i;

    /* Update NOR flash base address bank for CFI flash init driver */
    for (i = 0 ; i < CFG_MAX_FLASH_BANKS_DETECT; i++)
    {
	devBaseAddr = mvBoardGetDeviceBaseAddr(i,BOARD_DEV_NOR_FLASH);
	if (devBaseAddr != 0xFFFFFFFF)
	{
	    flash_add_base_addr (devNum, devBaseAddr);
	    devNum++;
	}
    }
    mv_board_num_flash_banks = devNum;

    /* Update SPI flash count for CFI flash init driver */
    /* Assumption only 1 SPI flash on board */
    for (i = 0 ; i < CFG_MAX_FLASH_BANKS_DETECT; i++)
    {
    	devBaseAddr = mvBoardGetDeviceBaseAddr(i,BOARD_DEV_SPI_FLASH);
    	if (devBaseAddr != 0xFFFFFFFF)
		mv_board_num_flash_banks += 1;
    }
}
#endif	/* CFG_FLASH_CFI_DRIVER */


/*******************************************************************************
* mvHddPowerCtrl - 
*
* DESCRIPTION:
*       This function set HDD power on/off acording to env or wait for button push
* INPUT:
*	None
* OUTPUT:
*	None
* RETURN:
*       None
*
*******************************************************************************/
static void mvHddPowerCtrl(void)
{

    MV_32 hddPowerBit;
    MV_32 fanPowerBit;
	MV_32 hddHigh = 0;
	MV_32 fanHigh = 0;
	char* env;
	
    if(RD_88F6281A_ID == mvBoardIdGet())
    {
        hddPowerBit = mvBoarGpioPinNumGet(BOARD_GPP_HDD_POWER, 0);
        fanPowerBit = mvBoarGpioPinNumGet(BOARD_GPP_FAN_POWER, 0);
        if (hddPowerBit > 31)
    	{
    		hddPowerBit = hddPowerBit % 32;
    		hddHigh = 1;
    	}
    
    	if (fanPowerBit > 31)
    	{
    		fanPowerBit = fanPowerBit % 32;
    		fanHigh = 1;
    	}
    }

	if ((RD_88F6281A_ID == mvBoardIdGet()) || (RD_88F6192A_ID == mvBoardIdGet()) || 
        (RD_88F6190A_ID == mvBoardIdGet()))
	{
		env = getenv("hddPowerCtrl");
 		if(!env || ( (strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) ) )
                	setenv("hddPowerCtrl","no");
        	else
                	setenv("hddPowerCtrl","yes");

        if(RD_88F6281A_ID == mvBoardIdGet())
        {
            mvBoardFanPowerControl(MV_TRUE);
            mvBoardHDDPowerControl(MV_TRUE);
        }
        else
        {
            /* FAN power on */
    		MV_REG_BIT_SET(GPP_DATA_OUT_REG(fanHigh),(1<<fanPowerBit));
    		MV_REG_BIT_RESET(GPP_DATA_OUT_EN_REG(fanHigh),(1<<fanPowerBit));
            /* HDD power on */
            MV_REG_BIT_SET(GPP_DATA_OUT_REG(hddHigh),(1<<hddPowerBit));
            MV_REG_BIT_RESET(GPP_DATA_OUT_EN_REG(hddHigh),(1<<hddPowerBit));
        }
        
	}
}


#if (CONFIG_COMMANDS & CFG_CMD_RCVR)
static void recoveryDetection(void)
{
    	MV_32 stateButtonBit = mvBoardGpioPinNumGet(BOARD_GPP_WPS_BUTTON,0);
	MV_32 buttonHigh = 0;
	char* env;

	/* Check if auto recovery is en */	
	env = getenv("enaAutoRecovery");
 	if(!env || ( (strcmp(env,"yes") == 0) || (strcmp(env,"Yes") == 0) ) )
               	setenv("enaAutoRecovery","yes");
        else
	{
               	setenv("enaAutoRecovery","no");
		rcvrflag = 0;
		return;
	}

	if (stateButtonBit == MV_ERROR)
	{	
		rcvrflag = 0;
		return;
	}

	if (stateButtonBit > 31)
	{
		stateButtonBit = stateButtonBit % 32;
		buttonHigh = 1;
	}

	/* Set state input indication pin as input */
	MV_REG_BIT_SET(GPP_DATA_OUT_EN_REG(buttonHigh),(1<<stateButtonBit));

	/* check if recovery triggered - button is pressed */
	if (!(MV_REG_READ(GPP_DATA_IN_REG(buttonHigh)) & (1 << stateButtonBit)))
	{	
		rcvrflag = 1;
	}
}

extern int do_bootm (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);

void recoveryHandle(void)
{
	char cmd[256];
	char img[10];
	char * argv[3];
	char *env, *env1, *multi;
	MV_32 imagAddr = 0x400000;	
	MV_32 imagSize = 0, netflag = 1;	
	char ip[16]= {"dhcp"};
    char* usbload[5];
	unsigned int netwait = 3000;
	int upgrade = -1;

	/* Recovery with Multipart/InitEmbedded uImage support */
	multi = getenv("image_multi");
	if(multi && ((strcmp(multi,"yes") == 0) ||  (strcmp(multi,"Yes") == 0)))
	{
		setenv("image_multi","yes");
		printf("Recovery imgage will be treated in multi mode\n");
	}
	else
	{
		setenv("image_multi","no");
		printf("Recovery imgage will be treated in normal mode\n");
	}
	multi = getenv("image_multi");

    /* First try to perform recovery from USB DOK*/
#ifdef CONFIG_USB_STORAGE
    /* try to recognize storage devices immediately */
    if (usb_init() >= 0)
    {
        if(usb_stor_scan(1) >= 0)
        {
            netflag = 0;
			usbload[0] = "usbload";
			usbload[1] = "usb";
			usbload[2] = "0:1";
			usbload[3] = getenv("loadaddr");
			if ((strcmp(multi,"yes") == 0))
				usbload[4] = "/multiware.img";
			else
				usbload[4] = "/flashware.img";

            printf("Trying to load image from USB flash drive using FAT FS\n");
            if(do_fat_fsload(0, 0, 5, usbload) == 1)
            {
                printf("Trying to load image from USB flash drive using ext2 FS partition 0\n");
                usbload[2] = "0:0";
                if(do_ext2load(0, 0, 5, usbload) == 1)
                {
                    printf("Trying to load image from USB flash drive using ext2 FS partition 1\n");
                    usbload[2] = "0:1";
                    if(do_ext2load(0, 0, 5, usbload) == 1)
                    {
                        printf("Couldn't load recovery image from USB flash drive, Trying network interface\n");
                        netflag = 1;
                    }
                    else
                    {
                        env = getenv("filesize");
						usbload[3] = getenv("loadaddr");
						imagSize = simple_strtoul(env, NULL, 16); /* get the filesize env var */
						
						/* Trying to check if we forced to upgrade by placing upgrade.me */
						usbload[4] = "/upgrade.me";
						sprintf(usbload[3], "0x%x", simple_strtoul(getenv("loadaddr"), NULL, 16) + imagSize + 1);
						upgrade=do_ext2load(0, 0, 5, usbload);
                    }
                }
                else
                {
                    env = getenv("filesize");
                    imagSize = simple_strtoul(env, NULL, 16); /* get the filesize env var */

					/* Trying to check if we forced to upgrade by placing upgrade.me */
					usbload[4] = "/upgrade.me";
					sprintf(usbload[3], "0x%x", simple_strtoul(getenv("loadaddr"), NULL, 16) + imagSize + 1);
					upgrade=do_ext2load(0, 0, 5, usbload);
                }
            }
            else
            {
                env = getenv("filesize");
                imagSize = simple_strtoul(env, NULL, 16); /* get the filesize env var */
				
				/* Trying to check if we forced to upgrade by placing upgrade.me */
				usbload[4] = "/upgrade.me";
				usbload[3] = "0x000000000";
				env=getenv("loadaddr");
				imagAddr=simple_strtoul(env, NULL, 16);
				sprintf(usbload[3], "0x%x", imagAddr + imagSize + 1);
				printf("dummyaddr:%s\n", usbload[3]);
				upgrade=do_fat_fsload(0, 0, 5, usbload);
			}
		// Decide on upgrade/init mode
		multi = getenv("image_multi");
		printf("Multi:%s, Upgrade:%d\n", multi, upgrade);
		if ( (upgrade == 0) && (strcmp(multi,"yes") == 0) )
			sprintf(ip, "usb_upgrade");
		else
			sprintf(ip, "usb");
        }
    }
#endif

    if(netflag == 1)
    {
        /* Perform the DHCP */
		printf("Aquiring an IP address using DHCP after settling network...\n");
		mvOsDelay(netwait);

		while (netwait)
		{
			if (NetLoop(DHCP) == -1)
			{
				if (!netwait)
				{
					ulong tmpip;
					printf("Failed to retreive an IP address, switching to static IP address (%s)!\n", getenv("rcvrip"));
					tmpip = getenv_IPaddr ("rcvrip");
					NetCopyIP(&NetOurIP, &tmpip);
					sprintf(ip, "static");
				}
				else
				{
					mvOsDelay(1000);
					netwait-=1000;
				}
			}
			else
			{
				printf("DHCP should be ready for Recovery.\n");
				break;
    		}
    	}
    
    	/* Perform the recovery */
		printf("Starting the Recovery process to retreive the image file...\n");
        if ((imagSize = NetLoop(RCVR)) == -1)
    	{
    		printf("Failed\n");
    		return;
    	}
    }

	/* Boot the downloaded image */	
	env = getenv("loadaddr");
	if (!env) {
		printf("Missing loadaddr environment variable assuming default (0x200000)!\n");
		setenv("loadaddr","0x200000");
		saveenv();
	}
	else
		imagAddr = simple_strtoul(env, NULL, 16); /* get the loadaddr env var */

    /* This assignment to cmd should execute prior to the RD setenv and saveenv below*/
    printf("Update bootcmd\n");
    env = getenv("ethaddr");
    env1 = getenv("eth1addr");

	multi = getenv("image_multi");
	if ((strcmp(multi,"yes") == 0))
	{
		sprintf(cmd,"setenv bootargs $(console) root=/dev/ram0 $(mvNetConfig) recovery=%s serverip=%d.%d.%d.%d  ethact=$(ethact) ethaddr=%s eth1addr=%s; bootm $(loadaddr);", ip, (NetServerIP & 0xFF), ((NetServerIP >> 8) & 0xFF), ((NetServerIP >> 16) & 0xFF), ((NetServerIP >> 24) & 0xFF),env, env1);
		printf("\nbootcmd(recovery-image-multi): %s\n", cmd);
	}
	else
	{
		sprintf(cmd,"setenv bootargs $(console) root=/dev/ram0 $(mvNetConfig) rootfstype=squashfs initrd=0x%x,0x%x ramdisk_size=%d recovery=%s serverip=%d.%d.%d.%d  ethact=$(ethact) ethaddr=%s eth1addr=%s; bootm $(loadaddr);", imagAddr + 0x200000, (imagSize - 0x300000), (imagSize - 0x300000)/1024, ip, (NetServerIP & 0xFF), ((NetServerIP >> 8) & 0xFF), ((NetServerIP >> 16) & 0xFF), ((NetServerIP >> 24) & 0xFF),env, env1);
		printf("\nbootcmd(recovery-image-normal): %s\n", cmd);
	}

	if(RD_88F6192A_ID == mvBoardIdGet())
    	{
		setenv("console","console=ttyS0,115200 mtdparts=spi_flash:0x100000@0x0(uboot)ro,0x200000@0x100000(uimage),0xb80000@0x300000(rootfs),0x180000@0xe80000(varfs),0xf00000@0x100000(flash) varfs=/dev/mtdblock3");
		setenv("bootcmd","setenv bootargs $(console) rootfstype=squashfs root=/dev/mtdblock2; bootm 0xf8100000;");
		saveenv();
    	}
	else
	{
		if ((strcmp(multi,"yes") == 0))
		{
			setenv("bootcmd","setenv bootargs $(console) ubi.mtd=2 root=ubi0:rootfsU rootfstype=ubifs $(mvNetConfig) $(mvPhoneConfig) $(lcd0_bootargs); nand read.e $(loadaddr) 0x100000 0x300000; bootm $(loadaddr);");
			setenv("console","console=ttyS0,115200");
			saveenv();
		}
		else
		{
			setenv("bootcmd","setenv bootargs $(console) rootfstype=squashfs root=/dev/mtdblock2 $(mvNetConfig) $(mvPhoneConfig) $(lcd0_bootargs); nand read.e $(loadaddr) 0x100000 0x200000; bootm $(loadaddr);");
			setenv("console","console=ttyS0,115200");
			saveenv();
		}
	}

	printf("\nbootcmd(saved)): %s\n", getenv("bootcmd"));
	printf("\nconsole(saved)): %s\n", getenv("console"));
	setenv("bootcmd", cmd);

	printf("Booting the image (@ 0x%x)...\n", imagAddr);

	sprintf(cmd, "boot");
	sprintf(img, "0x%x", imagAddr);
	argv[0] = cmd;
	argv[1] = img;

	do_bootd(NULL, 0, 2, argv);
}

void recoveryCheck(void)
{
	/* Start the recovery process if indicated by user */
	if (rcvrflag)
		recoveryHandle();
}
#endif

#ifdef MV_INC_BOARD_SPI_FLASH
#include <environment.h>
#include "sflash/mvSFlash.h"

void memcpyFlash(env_t *env_ptr, void* buffer, MV_U32 size)
{
    MV_SFLASH_INFO *pFlash;
    pFlash = getMvFlashInfo(BOOT_FLASH_INDEX);

    mvSFlashBlockRd(pFlash,(MV_U32 *)env_ptr - mvFlashBaseAddrGet(pFlash),
                    (MV_U8 *)buffer, size);
}
#endif

#if defined(CONFIG_BUFFALO_PLATFORM)
MV_VOID bfEthPhyPolaritySet(void)
{
	switch (mvBoardIdGet()) {
	case BF_MVLSXH_ID:
	case BF_MVLSXL_ID:
		break;
	case BF_MVWXL_ID:
	case BF_MVWSSX_ID:
	case BF_MVLSVL_ID:
	case BF_MVLSWV_ID:
	case BF_MVLSQV_ID:
		mvEthE111xPhyBasicInitLedPolarityChange(0);
		mvEthE111xPhyBasicInitLedPolarityChange(1);
		break;
	default:
		break;
	}
}

MV_BOOL bfIsStartBootProcess(void)
{
	if (mvBoardIdGet() == BF_MVLSXL_GE_V2_ID) {
		return MV_TRUE;
	}

	MV_U32 magic = bfGetMagicKey();
	printf("MacicKey = 0x%02x\n", magic);

	if (buffalo_IsStopAtUbootStatus()) {
		if (magic == MagicKeyUpsShutdownReachedHalt) {
			bfSetMagicKey(MagicKeyUpsShutdown);
			printf("Magic key condition is ups shutdown.\n"
			       "Indicate ups error and wait re power active.\n");
			bfErrorCodeDisp(BIT_ALARM_LED, 10);
		}
		else {
			bfSetMagicKey(MagicKeySwPoffUbootPassed);
		}
	}

	buffalo_all_link_led_on();

	if (bfGppInRegBitTest(BIT_FUNC_SW)) {
		printf("Switched to func-boot mode.\n");
		printf("Now waiting for func switch release.\n");

		bfGppBlinkRegBitSet(BIT_FUNC_LED);
		while (bfGppInRegBitTest(BIT_FUNC_SW))
			udelay(1000);

		printf("The Func switch was released.\n");
		printf("Now waiting for the Func switch pushed or timeout.\n");

		int i;
		int sw_off_cnt = 0;
		for (i = 0; i < 60 * 1000; i++) {
			if (mvBoardIdGet() == BF_MVLSQV_ID) {
				if (bfGppInRegBitTest(BIT_FUNC_SW)) {
					setenv("bootargs_func", "func=1");
					return MV_TRUE;
				}
				if (bfGppInRegBitTest(BIT_PWR_SW)) {
					sw_off_cnt++;
				}
				else {
					sw_off_cnt = 0;
				}
			}
			else {
				if (bfIsPWR_or_PWRAUTO_SignalAsserted() &&
				    bfGppInRegBitTest(BIT_FUNC_SW)) {
					setenv("bootargs_func", "func=1");
					return MV_TRUE;
				}

				if (!bfIsPWR_or_PWRAUTO_SignalAsserted()) {
					sw_off_cnt++;
				}
				else {
					sw_off_cnt = 0;
				}
			}
			
			if (sw_off_cnt > 50) { // 50ms
				printf("The power switch was moved to OFF position.\n");
				do_reset(NULL, 0, 0, NULL);
				return MV_FALSE;
			}

			udelay(1000);
		}
		// time out.
	}

	return MV_TRUE;
}

void bfAllLedOff(void)
{
	bfGppOutRegBitNagate(BIT_PWR_LED);
	bfGppBlinkRegBitClr(BIT_PWR_LED);
	bfGppOutRegBitNagate(BIT_INFO_LED);
	bfGppBlinkRegBitClr(BIT_INFO_LED);
	bfGppOutRegBitNagate(BIT_ALARM_LED);
	bfGppBlinkRegBitClr(BIT_ALARM_LED);
	bfGppOutRegBitNagate(BIT_FUNC_LED);
	bfGppBlinkRegBitClr(BIT_FUNC_LED);
	bfGppOutRegBitNagate(BIT_FUNC_RED_LED);
	bfGppBlinkRegBitClr(BIT_FUNC_RED_LED);
	bfGppOutRegBitNagate(BIT_HDD_ERROR0);
	bfGppBlinkRegBitClr(BIT_HDD_ERROR0);
	bfGppOutRegBitNagate(BIT_HDD_ERROR1);
	bfGppBlinkRegBitClr(BIT_HDD_ERROR1);
}

void bfErrorCodeDisp(MV_32 gppbit, int code)
{
	int i;
	int ones_place;
	int tens_place;

	if (mvBoardIdGet() == BF_MVLSXL_GE_V2_ID)
		return;

	bfAllLedOff();

	if (code < 0 || gppbit < 0)
		return;

	ones_place = code % 10;
	tens_place = code / 10;

	for (;;) {
		bfGppOutRegBitNagate(gppbit);
		udelay(2 * 1000 * 1000);
		for (i = tens_place; i != 0; i--) {
			bfGppOutRegBitAssert(gppbit);
			udelay(1000 * 1000);
			bfGppOutRegBitNagate(gppbit);
			udelay(300 * 1000);
			// switch handle
			if (!bfIsPWR_or_PWRAUTO_SignalAsserted())
				do_reset(NULL, 0, 0, NULL);

			if (bfGppInRegBitTest(BIT_FUNC_SW))
				goto disp_end;
		}

		for (i = ones_place; i != 0; i--) {
			bfGppOutRegBitAssert(gppbit);
			udelay(200 * 1000);
			bfGppOutRegBitNagate(gppbit);
			udelay(300 * 1000);
			// switch handle
			if (!bfIsPWR_or_PWRAUTO_SignalAsserted())
				do_reset(NULL, 0, 0, NULL);

			if (bfGppInRegBitTest(BIT_FUNC_SW))
				goto disp_end;
		}
	}

 disp_end:
	bfGppOutRegBitNagate(gppbit);
	bfGppOutRegBitAssert(BIT_PWR_LED);
	bfGppBlinkRegBitSet(BIT_PWR_LED);
}

void bfDispAllInitrdError(void)
{
	bfErrorCodeDisp(BIT_ALARM_LED, 6);
}

#endif
