/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell 
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

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
********************************************************************************
Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File under the following licensing terms. 
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    *   Redistributions of source code must retain the above copyright notice,
	    this list of conditions and the following disclaimer. 

    *   Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution. 

    *   Neither the name of Marvell nor the names of its contributors may be 
        used to endorse or promote products derived from this software without 
        specific prior written permission. 
    
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/
#include "mvCommon.h"
#include "mvBoardEnvLib.h"
#include "mvBoardEnvSpec.h"
#include "twsi/mvTwsi.h"

#if !defined(CONFIG_BUFFALO_PLATFORM)
#define DB_88F6281A_BOARD_PCI_IF_NUM            0x0
#define DB_88F6281A_BOARD_TWSI_DEF_NUM		    0x7
#define DB_88F6281A_BOARD_MAC_INFO_NUM		    0x2
#define DB_88F6281A_BOARD_GPP_INFO_NUM		    0x1
#define DB_88F6281A_BOARD_MPP_CONFIG_NUM		0x1
#define DB_88F6281A_BOARD_MPP_GROUP_TYPE_NUM	0x1
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
    #define DB_88F6281A_BOARD_DEVICE_CONFIG_NUM	    0x1
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
    #define DB_88F6281A_BOARD_DEVICE_CONFIG_NUM	    0x2
#else
    #define DB_88F6281A_BOARD_DEVICE_CONFIG_NUM	    0x1
#endif
#define DB_88F6281A_BOARD_DEBUG_LED_NUM		    0x0
#define DB_88F6281A_BOARD_NAND_READ_PARAMS		    0x000C0282
#define DB_88F6281A_BOARD_NAND_WRITE_PARAMS		    0x00010305
#define DB_88F6281A_BOARD_NAND_CONTROL		        0x01c00541


MV_BOARD_TWSI_INFO	db88f6281AInfoBoardTwsiDev[] =
	/* {{MV_BOARD_DEV_CLASS	devClass, MV_U8	twsiDevAddr, MV_U8 twsiDevAddrType}} */
	{
	{BOARD_DEV_TWSI_EXP, 0x20, ADDR7_BIT},
	{BOARD_DEV_TWSI_EXP, 0x21, ADDR7_BIT},
	{BOARD_DEV_TWSI_EXP, 0x27, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4C, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4D, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4E, ADDR7_BIT},
	{BOARD_TWSI_AUDIO_DEC, 0x4A, ADDR7_BIT}
	};

MV_BOARD_MAC_INFO db88f6281AInfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{
	{BOARD_MAC_SPEED_AUTO, 0x8},
	{BOARD_MAC_SPEED_AUTO, 0x9}
	}; 

MV_BOARD_MPP_TYPE_INFO db88f6281AInfoBoardMppTypeInfo[] = 
	/* {{MV_BOARD_MPP_TYPE_CLASS	boardMppGroup1,
		MV_BOARD_MPP_VOLTAGE_LEVEL	boardMppGroup1Voltage,
 		MV_BOARD_MPP_TYPE_CLASS	boardMppGroup2,
		MV_BOARD_MPP_VOLTAGE_LEVEL	boardMppGroup2Voltage}} */
	{{MV_BOARD_AUTO, MV_BOARD_1_8V, MV_BOARD_AUTO, MV_BOARD_1_8V}
	}; 

MV_BOARD_GPP_INFO db88f6281AInfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{
	{BOARD_GPP_TSU_DIRCTION, 33}
	/*muxed with TDM/Audio module via IOexpender
	{BOARD_GPP_SDIO_DETECT, 38},
	{BOARD_GPP_USB_VBUS, 49}*/
	};

MV_DEV_CS_INFO db88f6281AInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
		 {{0, N_A, BOARD_DEV_NAND_FLASH, 8}};	   /* NAND DEV */         
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
		 {
         {0, N_A, BOARD_DEV_NAND_FLASH, 8},	   /* NAND DEV */
         {1, N_A, BOARD_DEV_SPI_FLASH, 8},	   /* SPI DEV */
         };
#else
	 {{1, N_A, BOARD_DEV_SPI_FLASH, 8}};	   /* SPI DEV */         
#endif

MV_BOARD_MPP_INFO	db88f6281AInfoBoardMppConfigValue[] = 
	{{{
	DB_88F6281A_MPP0_7,		
	DB_88F6281A_MPP8_15,		
	DB_88F6281A_MPP16_23,		
	DB_88F6281A_MPP24_31,		
	DB_88F6281A_MPP32_39,		
	DB_88F6281A_MPP40_47,		
	DB_88F6281A_MPP48_55		
	}}};


MV_BOARD_INFO db88f6281AInfo = {
	"DB-88F6281A-BP",				/* boardName[MAX_BOARD_NAME_LEN] */
	DB_88F6281A_BOARD_MPP_GROUP_TYPE_NUM,		/* numBoardMppGroupType */
	db88f6281AInfoBoardMppTypeInfo,
	DB_88F6281A_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfig */
	db88f6281AInfoBoardMppConfigValue,
	0,						/* intsGppMaskLow */
	0,						/* intsGppMaskHigh */
	DB_88F6281A_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDevIf */
	db88f6281AInfoBoardDeCsInfo,
	DB_88F6281A_BOARD_TWSI_DEF_NUM,			/* numBoardTwsiDev */
	db88f6281AInfoBoardTwsiDev,					
	DB_88F6281A_BOARD_MAC_INFO_NUM,			/* numBoardMacInfo */
	db88f6281AInfoBoardMacInfo,
	DB_88F6281A_BOARD_GPP_INFO_NUM,			/* numBoardGppInfo */
	db88f6281AInfoBoardGppInfo,
	DB_88F6281A_BOARD_DEBUG_LED_NUM,			/* activeLedsNumber */              
	NULL,
	0,						/* ledsPolarity */		
	DB_88F6281A_OE_LOW,				/* gppOutEnLow */
	DB_88F6281A_OE_HIGH,				/* gppOutEnHigh */
	DB_88F6281A_OE_VAL_LOW,				/* gppOutValLow */
	DB_88F6281A_OE_VAL_HIGH,				/* gppOutValHigh */
	0,						/* gppPolarityValLow */
	BIT6, 						/* gppPolarityValHigh */
	NULL,						/* pSwitchInfo */
    DB_88F6281A_BOARD_NAND_READ_PARAMS,
    DB_88F6281A_BOARD_NAND_WRITE_PARAMS,
    DB_88F6281A_BOARD_NAND_CONTROL
};


#define RD_88F6281A_BOARD_PCI_IF_NUM		0x0
#define RD_88F6281A_BOARD_TWSI_DEF_NUM		0x2
#define RD_88F6281A_BOARD_MAC_INFO_NUM		0x2
#define RD_88F6281A_BOARD_GPP_INFO_NUM		0x5
#define RD_88F6281A_BOARD_MPP_GROUP_TYPE_NUM	0x1
#define RD_88F6281A_BOARD_MPP_CONFIG_NUM		0x1
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
    #define RD_88F6281A_BOARD_DEVICE_CONFIG_NUM	    0x1
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
    #define RD_88F6281A_BOARD_DEVICE_CONFIG_NUM	    0x2
#else
    #define RD_88F6281A_BOARD_DEVICE_CONFIG_NUM	    0x1
#endif
#define RD_88F6281A_BOARD_DEBUG_LED_NUM		0x0
#define RD_88F6281A_BOARD_NAND_READ_PARAMS		    0x000C0282
#define RD_88F6281A_BOARD_NAND_WRITE_PARAMS		    0x00010305
#define RD_88F6281A_BOARD_NAND_CONTROL		        0x01c00541

MV_BOARD_MAC_INFO rd88f6281AInfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{{BOARD_MAC_SPEED_1000M, 0xa},
    {BOARD_MAC_SPEED_AUTO, 0xb}
	}; 

MV_BOARD_SWITCH_INFO rd88f6281AInfoBoardSwitchInfo[] = 
	/* MV_32 linkStatusIrq, {MV_32 qdPort0, MV_32 qdPort1, MV_32 qdPort2, MV_32 qdPort3, MV_32 qdPort4}, 
		MV_32 qdCpuPort, MV_32 smiScanMode, MV_32 switchOnPort} */
	{{38, {0, 1, 2, 3, -1}, 5, 2, 0},
	 {-1, {-1}, -1, -1, -1}};

MV_BOARD_TWSI_INFO	rd88f6281AInfoBoardTwsiDev[] =
	/* {{MV_BOARD_DEV_CLASS	devClass, MV_U8	twsiDevAddr, MV_U8 twsiDevAddrType}} */
	{
	{BOARD_DEV_TWSI_EXP, 0xFF, ADDR7_BIT}, /* dummy entry to align with modules indexes */
	{BOARD_DEV_TWSI_EXP, 0x27, ADDR7_BIT}
	};

MV_BOARD_MPP_TYPE_INFO rd88f6281AInfoBoardMppTypeInfo[] = 
	{{MV_BOARD_RGMII, MV_BOARD_1_8V, MV_BOARD_TDM, MV_BOARD_3_3V}
	}; 

MV_DEV_CS_INFO rd88f6281AInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
		 {{0, N_A, BOARD_DEV_NAND_FLASH, 8}};	   /* NAND DEV */
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
		 {
         {0, N_A, BOARD_DEV_NAND_FLASH, 8},	   /* NAND DEV */
         {1, N_A, BOARD_DEV_SPI_FLASH, 8},	   /* SPI DEV */
         };
#else
		 {{1, N_A, BOARD_DEV_SPI_FLASH, 8}};	   /* SPI DEV */         
#endif

MV_BOARD_GPP_INFO rd88f6281AInfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{{BOARD_GPP_SDIO_DETECT, 28},
    {BOARD_GPP_USB_OC, 29},
    {BOARD_GPP_WPS_BUTTON, 35},
    {BOARD_GPP_MV_SWITCH, 38},
    {BOARD_GPP_USB_VBUS, 49}
	};

MV_BOARD_MPP_INFO	rd88f6281AInfoBoardMppConfigValue[] = 
	{{{
	RD_88F6281A_MPP0_7,		
	RD_88F6281A_MPP8_15,		
	RD_88F6281A_MPP16_23,		
	RD_88F6281A_MPP24_31,		
	RD_88F6281A_MPP32_39,		
	RD_88F6281A_MPP40_47,		
	RD_88F6281A_MPP48_55		
	}}};

MV_BOARD_INFO rd88f6281AInfo = {
	"RD-88F6281A",				/* boardName[MAX_BOARD_NAME_LEN] */
	RD_88F6281A_BOARD_MPP_GROUP_TYPE_NUM,		/* numBoardMppGroupType */
	rd88f6281AInfoBoardMppTypeInfo,
	RD_88F6281A_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfig */
	rd88f6281AInfoBoardMppConfigValue,
	0,						/* intsGppMaskLow */
	(1 << 3),					/* intsGppMaskHigh */
	RD_88F6281A_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDevIf */
	rd88f6281AInfoBoardDeCsInfo,
	RD_88F6281A_BOARD_TWSI_DEF_NUM,			/* numBoardTwsiDev */
	rd88f6281AInfoBoardTwsiDev,					
	RD_88F6281A_BOARD_MAC_INFO_NUM,			/* numBoardMacInfo */
	rd88f6281AInfoBoardMacInfo,
	RD_88F6281A_BOARD_GPP_INFO_NUM,			/* numBoardGppInfo */
	rd88f6281AInfoBoardGppInfo,
	RD_88F6281A_BOARD_DEBUG_LED_NUM,			/* activeLedsNumber */              
	NULL,
	0,										/* ledsPolarity */		
	RD_88F6281A_OE_LOW,				/* gppOutEnLow */
	RD_88F6281A_OE_HIGH,				/* gppOutEnHigh */
	RD_88F6281A_OE_VAL_LOW,				/* gppOutValLow */
	RD_88F6281A_OE_VAL_HIGH,				/* gppOutValHigh */
	0,						/* gppPolarityValLow */
	BIT6, 						/* gppPolarityValHigh */
	rd88f6281AInfoBoardSwitchInfo,			/* pSwitchInfo */
    RD_88F6281A_BOARD_NAND_READ_PARAMS,
    RD_88F6281A_BOARD_NAND_WRITE_PARAMS,
    RD_88F6281A_BOARD_NAND_CONTROL
};


#define DB_88F6192A_BOARD_PCI_IF_NUM            0x0
#define DB_88F6192A_BOARD_TWSI_DEF_NUM		    0x7
#define DB_88F6192A_BOARD_MAC_INFO_NUM		    0x2
#define DB_88F6192A_BOARD_GPP_INFO_NUM		    0x3
#define DB_88F6192A_BOARD_MPP_GROUP_TYPE_NUM	0x1
#define DB_88F6192A_BOARD_MPP_CONFIG_NUM		0x1
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
    #define DB_88F6192A_BOARD_DEVICE_CONFIG_NUM	    0x1
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
    #define DB_88F6192A_BOARD_DEVICE_CONFIG_NUM	    0x2
#else
    #define DB_88F6192A_BOARD_DEVICE_CONFIG_NUM	    0x1
#endif
#define DB_88F6192A_BOARD_DEBUG_LED_NUM		    0x0
#define DB_88F6192A_BOARD_NAND_READ_PARAMS		    0x000C0282
#define DB_88F6192A_BOARD_NAND_WRITE_PARAMS		    0x00010305
#define DB_88F6192A_BOARD_NAND_CONTROL		        0x01c00541

MV_BOARD_TWSI_INFO	db88f6192AInfoBoardTwsiDev[] =
	/* {{MV_BOARD_DEV_CLASS	devClass, MV_U8	twsiDevAddr, MV_U8 twsiDevAddrType}} */
	{
	{BOARD_DEV_TWSI_EXP, 0x20, ADDR7_BIT},
	{BOARD_DEV_TWSI_EXP, 0x21, ADDR7_BIT},
	{BOARD_DEV_TWSI_EXP, 0x27, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4C, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4D, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4E, ADDR7_BIT},
	{BOARD_TWSI_AUDIO_DEC, 0x4A, ADDR7_BIT}
	};

MV_BOARD_MAC_INFO db88f6192AInfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{
	{BOARD_MAC_SPEED_AUTO, 0x8},
	{BOARD_MAC_SPEED_AUTO, 0x9}
	}; 

MV_BOARD_MPP_TYPE_INFO db88f6192AInfoBoardMppTypeInfo[] = 
	{{MV_BOARD_AUTO, MV_BOARD_1_8V, MV_BOARD_OTHER, MV_BOARD_3_3V}
	}; 

MV_DEV_CS_INFO db88f6192AInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
		 {{0, N_A, BOARD_DEV_NAND_FLASH, 8}};	   /* NAND DEV */
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
		 {
         {0, N_A, BOARD_DEV_NAND_FLASH, 8},	   /* NAND DEV */
         {1, N_A, BOARD_DEV_SPI_FLASH, 8},	   /* SPI DEV */
         };
#else
		 {{1, N_A, BOARD_DEV_SPI_FLASH, 8}};	   /* SPI DEV */         
#endif

MV_BOARD_GPP_INFO db88f6192AInfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{
    {BOARD_GPP_SDIO_WP, 20},
	{BOARD_GPP_USB_VBUS, 22},
	{BOARD_GPP_SDIO_DETECT, 23},
	};

MV_BOARD_MPP_INFO	db88f6192AInfoBoardMppConfigValue[] = 
	{{{
	DB_88F6192A_MPP0_7,		
	DB_88F6192A_MPP8_15,		
	DB_88F6192A_MPP16_23,		
	DB_88F6192A_MPP24_31,		
	DB_88F6192A_MPP32_35
	}}};

MV_BOARD_INFO db88f6192AInfo = {
	"DB-88F6192A-BP",				/* boardName[MAX_BOARD_NAME_LEN] */
	DB_88F6192A_BOARD_MPP_GROUP_TYPE_NUM,		/* numBoardMppGroupType */
	db88f6192AInfoBoardMppTypeInfo,
	DB_88F6192A_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfig */
	db88f6192AInfoBoardMppConfigValue,
	0,						/* intsGppMaskLow */
	(1 << 3),					/* intsGppMaskHigh */
	DB_88F6192A_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDevIf */
	db88f6192AInfoBoardDeCsInfo,
	DB_88F6192A_BOARD_TWSI_DEF_NUM,			/* numBoardTwsiDev */
	db88f6192AInfoBoardTwsiDev,					
	DB_88F6192A_BOARD_MAC_INFO_NUM,			/* numBoardMacInfo */
	db88f6192AInfoBoardMacInfo,
	DB_88F6192A_BOARD_GPP_INFO_NUM,			/* numBoardGppInfo */
	db88f6192AInfoBoardGppInfo,
	DB_88F6192A_BOARD_DEBUG_LED_NUM,			/* activeLedsNumber */              
	NULL,
	0,										/* ledsPolarity */		
	DB_88F6192A_OE_LOW,				/* gppOutEnLow */
	DB_88F6192A_OE_HIGH,				/* gppOutEnHigh */
	DB_88F6192A_OE_VAL_LOW,				/* gppOutValLow */
	DB_88F6192A_OE_VAL_HIGH,				/* gppOutValHigh */
	0,						/* gppPolarityValLow */
	0, 						/* gppPolarityValHigh */
	NULL,						/* pSwitchInfo */
    DB_88F6192A_BOARD_NAND_READ_PARAMS,
    DB_88F6192A_BOARD_NAND_WRITE_PARAMS,
    DB_88F6192A_BOARD_NAND_CONTROL
};

#define DB_88F6190A_BOARD_MAC_INFO_NUM		0x1

MV_BOARD_INFO db88f6190AInfo = {
	"DB-88F6190A-BP",				/* boardName[MAX_BOARD_NAME_LEN] */
	DB_88F6192A_BOARD_MPP_GROUP_TYPE_NUM,		/* numBoardMppGroupType */
	db88f6192AInfoBoardMppTypeInfo,
	DB_88F6192A_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfig */
	db88f6192AInfoBoardMppConfigValue,
	0,						/* intsGppMaskLow */
	(1 << 3),					/* intsGppMaskHigh */
	DB_88F6192A_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDevIf */
	db88f6192AInfoBoardDeCsInfo,
	DB_88F6192A_BOARD_TWSI_DEF_NUM,			/* numBoardTwsiDev */
	db88f6192AInfoBoardTwsiDev,					
	DB_88F6190A_BOARD_MAC_INFO_NUM,			/* numBoardMacInfo */
	db88f6192AInfoBoardMacInfo,
	DB_88F6192A_BOARD_GPP_INFO_NUM,			/* numBoardGppInfo */
	db88f6192AInfoBoardGppInfo,
	DB_88F6192A_BOARD_DEBUG_LED_NUM,			/* activeLedsNumber */              
	NULL,
	0,										/* ledsPolarity */		
	DB_88F6192A_OE_LOW,				/* gppOutEnLow */
	DB_88F6192A_OE_HIGH,				/* gppOutEnHigh */
	DB_88F6192A_OE_VAL_LOW,				/* gppOutValLow */
	DB_88F6192A_OE_VAL_HIGH,				/* gppOutValHigh */
	0,						/* gppPolarityValLow */
	0, 						/* gppPolarityValHigh */
	NULL,						/* pSwitchInfo */
    DB_88F6192A_BOARD_NAND_READ_PARAMS,
    DB_88F6192A_BOARD_NAND_WRITE_PARAMS,
    DB_88F6192A_BOARD_NAND_CONTROL
};

#define RD_88F6192A_BOARD_PCI_IF_NUM		0x0
#define RD_88F6192A_BOARD_TWSI_DEF_NUM		0x0
#define RD_88F6192A_BOARD_MAC_INFO_NUM		0x1
#define RD_88F6192A_BOARD_GPP_INFO_NUM		0xE
#define RD_88F6192A_BOARD_MPP_GROUP_TYPE_NUM	0x1
#define RD_88F6192A_BOARD_MPP_CONFIG_NUM		0x1
#define RD_88F6192A_BOARD_DEVICE_CONFIG_NUM	0x1
#define RD_88F6192A_BOARD_DEBUG_LED_NUM		0x3
#define RD_88F6192A_BOARD_NAND_READ_PARAMS		    0x000C0282
#define RD_88F6192A_BOARD_NAND_WRITE_PARAMS		    0x00010305
#define RD_88F6192A_BOARD_NAND_CONTROL		        0x01c00541

MV_U8	rd88f6192AInfoBoardDebugLedIf[] =
	{17, 28, 29};

MV_BOARD_MAC_INFO rd88f6192AInfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{{BOARD_MAC_SPEED_AUTO, 0x8}
	}; 

MV_BOARD_MPP_TYPE_INFO rd88f6192AInfoBoardMppTypeInfo[] = 
	{{MV_BOARD_OTHER, MV_BOARD_1_8V, MV_BOARD_OTHER, MV_BOARD_3_3V}
	}; 

MV_DEV_CS_INFO rd88f6192AInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
		 {{1, N_A, BOARD_DEV_SPI_FLASH, 8}};	   /* SPI DEV */

MV_BOARD_GPP_INFO rd88f6192AInfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{
	{BOARD_GPP_USB_VBUS_EN, 10},
	{BOARD_GPP_USB_HOST_DEVICE, 11},
	{BOARD_GPP_RESET, 14},
	{BOARD_GPP_POWER_ON_LED, 15},
	{BOARD_GPP_HDD_POWER, 16},
	{BOARD_GPP_WPS_BUTTON, 24},
	{BOARD_GPP_TS_BUTTON_C, 25},
	{BOARD_GPP_USB_VBUS, 26},
	{BOARD_GPP_USB_OC, 27},
	{BOARD_GPP_TS_BUTTON_U, 30},
	{BOARD_GPP_TS_BUTTON_R, 31},
	{BOARD_GPP_TS_BUTTON_L, 32},
	{BOARD_GPP_TS_BUTTON_D, 34},
	{BOARD_GPP_FAN_POWER, 35}
	};

MV_BOARD_MPP_INFO	rd88f6192AInfoBoardMppConfigValue[] = 
	{{{
	RD_88F6192A_MPP0_7,		
	RD_88F6192A_MPP8_15,		
	RD_88F6192A_MPP16_23,		
	RD_88F6192A_MPP24_31,		
	RD_88F6192A_MPP32_35
	}}};

MV_BOARD_INFO rd88f6192AInfo = {
	"RD-88F6192A-NAS",				/* boardName[MAX_BOARD_NAME_LEN] */
	RD_88F6192A_BOARD_MPP_GROUP_TYPE_NUM,		/* numBoardMppGroupType */
	rd88f6192AInfoBoardMppTypeInfo,
	RD_88F6192A_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfig */
	rd88f6192AInfoBoardMppConfigValue,
	0,						/* intsGppMaskLow */
	(1 << 3),					/* intsGppMaskHigh */
	RD_88F6192A_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDevIf */
	rd88f6192AInfoBoardDeCsInfo,
	RD_88F6192A_BOARD_TWSI_DEF_NUM,			/* numBoardTwsiDev */
	NULL,					
	RD_88F6192A_BOARD_MAC_INFO_NUM,			/* numBoardMacInfo */
	rd88f6192AInfoBoardMacInfo,
	RD_88F6192A_BOARD_GPP_INFO_NUM,			/* numBoardGppInfo */
	rd88f6192AInfoBoardGppInfo,
	RD_88F6192A_BOARD_DEBUG_LED_NUM,			/* activeLedsNumber */              
	rd88f6192AInfoBoardDebugLedIf,
	0,										/* ledsPolarity */		
	RD_88F6192A_OE_LOW,				/* gppOutEnLow */
	RD_88F6192A_OE_HIGH,				/* gppOutEnHigh */
	RD_88F6192A_OE_VAL_LOW,				/* gppOutValLow */
	RD_88F6192A_OE_VAL_HIGH,				/* gppOutValHigh */
	0,						/* gppPolarityValLow */
	0, 						/* gppPolarityValHigh */
	NULL,						/* pSwitchInfo */
    RD_88F6192A_BOARD_NAND_READ_PARAMS,
    RD_88F6192A_BOARD_NAND_WRITE_PARAMS,
    RD_88F6192A_BOARD_NAND_CONTROL
};

MV_BOARD_INFO rd88f6190AInfo = {
	"RD-88F6190A-NAS",				/* boardName[MAX_BOARD_NAME_LEN] */
	RD_88F6192A_BOARD_MPP_GROUP_TYPE_NUM,		/* numBoardMppGroupType */
	rd88f6192AInfoBoardMppTypeInfo,
	RD_88F6192A_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfig */
	rd88f6192AInfoBoardMppConfigValue,
	0,						/* intsGppMaskLow */
	(1 << 3),					/* intsGppMaskHigh */
	RD_88F6192A_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDevIf */
	rd88f6192AInfoBoardDeCsInfo,
	RD_88F6192A_BOARD_TWSI_DEF_NUM,			/* numBoardTwsiDev */
	NULL,					
	RD_88F6192A_BOARD_MAC_INFO_NUM,			/* numBoardMacInfo */
	rd88f6192AInfoBoardMacInfo,
	RD_88F6192A_BOARD_GPP_INFO_NUM,			/* numBoardGppInfo */
	rd88f6192AInfoBoardGppInfo,
	RD_88F6192A_BOARD_DEBUG_LED_NUM,			/* activeLedsNumber */              
	rd88f6192AInfoBoardDebugLedIf,
	0,										/* ledsPolarity */		
	RD_88F6192A_OE_LOW,				/* gppOutEnLow */
	RD_88F6192A_OE_HIGH,				/* gppOutEnHigh */
	RD_88F6192A_OE_VAL_LOW,				/* gppOutValLow */
	RD_88F6192A_OE_VAL_HIGH,				/* gppOutValHigh */
	0,						/* gppPolarityValLow */
	0, 						/* gppPolarityValHigh */
	NULL,						/* pSwitchInfo */
    RD_88F6192A_BOARD_NAND_READ_PARAMS,
    RD_88F6192A_BOARD_NAND_WRITE_PARAMS,
    RD_88F6192A_BOARD_NAND_CONTROL
};

#define DB_88F6180A_BOARD_PCI_IF_NUM		0x0
#define DB_88F6180A_BOARD_TWSI_DEF_NUM		0x5
#define DB_88F6180A_BOARD_MAC_INFO_NUM		0x1
#define DB_88F6180A_BOARD_GPP_INFO_NUM		0x0
#define DB_88F6180A_BOARD_MPP_GROUP_TYPE_NUM	0x2
#define DB_88F6180A_BOARD_MPP_CONFIG_NUM		0x1
#define DB_88F6180A_BOARD_DEVICE_CONFIG_NUM	    0x1
#define DB_88F6180A_BOARD_DEBUG_LED_NUM		0x0
#define DB_88F6180A_BOARD_NAND_READ_PARAMS		    0x000C0282
#define DB_88F6180A_BOARD_NAND_WRITE_PARAMS		    0x00010305
#define DB_88F6180A_BOARD_NAND_CONTROL		        0x01c00541

MV_BOARD_TWSI_INFO	db88f6180AInfoBoardTwsiDev[] =
	/* {{MV_BOARD_DEV_CLASS	devClass, MV_U8	twsiDevAddr, MV_U8 twsiDevAddrType}} */
	{
    {BOARD_DEV_TWSI_EXP, 0x20, ADDR7_BIT},
    {BOARD_DEV_TWSI_EXP, 0x21, ADDR7_BIT},
    {BOARD_DEV_TWSI_EXP, 0x27, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4C, ADDR7_BIT},
	{BOARD_TWSI_AUDIO_DEC, 0x4A, ADDR7_BIT}
	};

MV_BOARD_MAC_INFO db88f6180AInfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{{BOARD_MAC_SPEED_AUTO, 0x8}
	}; 

MV_BOARD_GPP_INFO db88f6180AInfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{
	/* Muxed with TDM/Audio module via IOexpender
	{BOARD_GPP_USB_VBUS, 6} */
	};

MV_BOARD_MPP_TYPE_INFO db88f6180AInfoBoardMppTypeInfo[] = 
	{{MV_BOARD_OTHER, MV_BOARD_3_3V, MV_BOARD_AUTO, MV_BOARD_3_3V}
	}; 

MV_DEV_CS_INFO db88f6180AInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
#if defined(MV_NAND_BOOT)
		 {{0, N_A, BOARD_DEV_NAND_FLASH, 8}};	   /* NAND DEV */         
#else
		 {{1, N_A, BOARD_DEV_SPI_FLASH, 8}};	   /* SPI DEV */         
#endif

MV_BOARD_MPP_INFO	db88f6180AInfoBoardMppConfigValue[] = 
	{{{
	DB_88F6180A_MPP0_7,		
	DB_88F6180A_MPP8_15,
    DB_88F6180A_MPP16_23,
    DB_88F6180A_MPP24_31,		
    DB_88F6180A_MPP32_39,
    DB_88F6180A_MPP40_44
	}}};

MV_BOARD_INFO db88f6180AInfo = {
	"DB-88F6180A-BP",				/* boardName[MAX_BOARD_NAME_LEN] */
	DB_88F6180A_BOARD_MPP_GROUP_TYPE_NUM,		/* numBoardMppGroupType */
	db88f6180AInfoBoardMppTypeInfo,
	DB_88F6180A_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfig */
	db88f6180AInfoBoardMppConfigValue,
	0,						/* intsGppMaskLow */
	0,					/* intsGppMaskHigh */
	DB_88F6180A_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDevIf */
	db88f6180AInfoBoardDeCsInfo,
	DB_88F6180A_BOARD_TWSI_DEF_NUM,			/* numBoardTwsiDev */
	db88f6180AInfoBoardTwsiDev,					
	DB_88F6180A_BOARD_MAC_INFO_NUM,			/* numBoardMacInfo */
	db88f6180AInfoBoardMacInfo,
	DB_88F6180A_BOARD_GPP_INFO_NUM,			/* numBoardGppInfo */
	NULL,
	DB_88F6180A_BOARD_DEBUG_LED_NUM,			/* activeLedsNumber */              
	NULL,
	0,										/* ledsPolarity */		
	DB_88F6180A_OE_LOW,				/* gppOutEnLow */
	DB_88F6180A_OE_HIGH,				/* gppOutEnHigh */
	DB_88F6180A_OE_VAL_LOW,				/* gppOutValLow */
	DB_88F6180A_OE_VAL_HIGH,				/* gppOutValHigh */
	0,						/* gppPolarityValLow */
	0, 						/* gppPolarityValHigh */
	NULL,						/* pSwitchInfo */
    DB_88F6180A_BOARD_NAND_READ_PARAMS,
    DB_88F6180A_BOARD_NAND_WRITE_PARAMS,
    DB_88F6180A_BOARD_NAND_CONTROL
};


#define RD_88F6281A_PCAC_BOARD_PCI_IF_NUM		0x0
#define RD_88F6281A_PCAC_BOARD_TWSI_DEF_NUM		0x1
#define RD_88F6281A_PCAC_BOARD_MAC_INFO_NUM		0x1
#define RD_88F6281A_PCAC_BOARD_GPP_INFO_NUM		0x0
#define RD_88F6281A_PCAC_BOARD_MPP_GROUP_TYPE_NUM	0x1
#define RD_88F6281A_PCAC_BOARD_MPP_CONFIG_NUM		0x1
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
    #define RD_88F6281A_PCAC_BOARD_DEVICE_CONFIG_NUM	    0x1
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
    #define RD_88F6281A_PCAC_BOARD_DEVICE_CONFIG_NUM	    0x2
#else
    #define RD_88F6281A_PCAC_BOARD_DEVICE_CONFIG_NUM	    0x1
#endif
#define RD_88F6281A_PCAC_BOARD_DEBUG_LED_NUM		0x4
#define RD_88F6281A_PCAC_BOARD_NAND_READ_PARAMS		    0x000C0282
#define RD_88F6281A_PCAC_BOARD_NAND_WRITE_PARAMS		    0x00010305
#define RD_88F6281A_PCAC_BOARD_NAND_CONTROL		        0x01c00541

MV_U8	rd88f6281APcacInfoBoardDebugLedIf[] =
	{38, 39, 40, 41};

MV_BOARD_MAC_INFO rd88f6281APcacInfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{{BOARD_MAC_SPEED_AUTO, 0x8}
	}; 

MV_BOARD_TWSI_INFO	rd88f6281APcacInfoBoardTwsiDev[] =
	/* {{MV_BOARD_DEV_CLASS	devClass, MV_U8	twsiDevAddr, MV_U8 twsiDevAddrType}} */
	{
	{BOARD_TWSI_OTHER, 0xa7, ADDR7_BIT}
	};

MV_BOARD_MPP_TYPE_INFO rd88f6281APcacInfoBoardMppTypeInfo[] = 
	{{MV_BOARD_OTHER, MV_BOARD_3_3V, MV_BOARD_OTHER, MV_BOARD_3_3V}
	}; 

MV_DEV_CS_INFO rd88f6281APcacInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
		 {{0, N_A, BOARD_DEV_NAND_FLASH, 8}};	   /* NAND DEV */
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
		 {
         {0, N_A, BOARD_DEV_NAND_FLASH, 8},	   /* NAND DEV */
         {1, N_A, BOARD_DEV_SPI_FLASH, 8},	   /* SPI DEV */
         };
#else
	 {{1, N_A, BOARD_DEV_SPI_FLASH, 8}};	   /* SPI DEV */         
#endif

MV_BOARD_MPP_INFO	rd88f6281APcacInfoBoardMppConfigValue[] = 
	{{{
	RD_88F6281A_PCAC_MPP0_7,		
	RD_88F6281A_PCAC_MPP8_15,		
	RD_88F6281A_PCAC_MPP16_23,		
	RD_88F6281A_PCAC_MPP24_31,		
	RD_88F6281A_PCAC_MPP32_39,		
	RD_88F6281A_PCAC_MPP40_47,		
	RD_88F6281A_PCAC_MPP48_55		
	}}};

MV_BOARD_INFO rd88f6281APcacInfo = {
	"RD-88F6281A-PCAC",				/* boardName[MAX_BOARD_NAME_LEN] */
	RD_88F6281A_PCAC_BOARD_MPP_GROUP_TYPE_NUM,	/* numBoardMppGroupType */
	rd88f6281APcacInfoBoardMppTypeInfo,
	RD_88F6281A_PCAC_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfig */
	rd88f6281APcacInfoBoardMppConfigValue,
	0,						/* intsGppMaskLow */
	(1 << 3),					/* intsGppMaskHigh */
	RD_88F6281A_PCAC_BOARD_DEVICE_CONFIG_NUM,	/* numBoardDevIf */
	rd88f6281APcacInfoBoardDeCsInfo,
	RD_88F6281A_PCAC_BOARD_TWSI_DEF_NUM,		/* numBoardTwsiDev */
	rd88f6281APcacInfoBoardTwsiDev,					
	RD_88F6281A_PCAC_BOARD_MAC_INFO_NUM,		/* numBoardMacInfo */
	rd88f6281APcacInfoBoardMacInfo,
	RD_88F6281A_PCAC_BOARD_GPP_INFO_NUM,		/* numBoardGppInfo */
	0,
	RD_88F6281A_PCAC_BOARD_DEBUG_LED_NUM,		/* activeLedsNumber */              
	NULL,
	0,										/* ledsPolarity */		
	RD_88F6281A_PCAC_OE_LOW,			/* gppOutEnLow */
	RD_88F6281A_PCAC_OE_HIGH,			/* gppOutEnHigh */
	RD_88F6281A_PCAC_OE_VAL_LOW,			/* gppOutValLow */
	RD_88F6281A_PCAC_OE_VAL_HIGH,			/* gppOutValHigh */
	0,						/* gppPolarityValLow */
	0, 	 					/* gppPolarityValHigh */
	NULL,						/* pSwitchInfo */
    RD_88F6281A_PCAC_BOARD_NAND_READ_PARAMS,
    RD_88F6281A_PCAC_BOARD_NAND_WRITE_PARAMS,
    RD_88F6281A_PCAC_BOARD_NAND_CONTROL
};


#define DB_88F6280A_BOARD_PCI_IF_NUM            0x0
#define DB_88F6280A_BOARD_TWSI_DEF_NUM		    0x7
#define DB_88F6280A_BOARD_MAC_INFO_NUM		    0x1
#define DB_88F6280A_BOARD_GPP_INFO_NUM		    0x0
#define DB_88F6280A_BOARD_MPP_CONFIG_NUM		0x1
#define DB_88F6280A_BOARD_MPP_GROUP_TYPE_NUM	0x1
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
    #define DB_88F6280A_BOARD_DEVICE_CONFIG_NUM	    0x1
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
    #define DB_88F6280A_BOARD_DEVICE_CONFIG_NUM	    0x2
#else
    #define DB_88F6280A_BOARD_DEVICE_CONFIG_NUM	    0x1
#endif
#define DB_88F6280A_BOARD_DEBUG_LED_NUM		    0x0
#define DB_88F6280A_BOARD_NAND_READ_PARAMS		    0x000C0282
#define DB_88F6280A_BOARD_NAND_WRITE_PARAMS		    0x00010305
#define DB_88F6280A_BOARD_NAND_CONTROL		        0x01c00541


MV_BOARD_TWSI_INFO	db88f6280AInfoBoardTwsiDev[] =
	/* {{MV_BOARD_DEV_CLASS	devClass, MV_U8	twsiDevAddr, MV_U8 twsiDevAddrType}} */
	{
	{BOARD_DEV_TWSI_EXP, 0x20, ADDR7_BIT},
	{BOARD_DEV_TWSI_EXP, 0x21, ADDR7_BIT},
	{BOARD_DEV_TWSI_EXP, 0x27, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4C, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4D, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4E, ADDR7_BIT},
	{BOARD_TWSI_AUDIO_DEC, 0x4A, ADDR7_BIT}
	};

MV_BOARD_MAC_INFO db88f6280AInfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{
	{BOARD_MAC_SPEED_AUTO, 0x8}
	}; 

MV_BOARD_MPP_TYPE_INFO db88f6280AInfoBoardMppTypeInfo[] = 
	{{MV_BOARD_AUTO, MV_BOARD_1_8V, MV_BOARD_OTHER, MV_BOARD_3_3V}
	}; 

MV_DEV_CS_INFO db88f6280AInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
		 {{0, N_A, BOARD_DEV_NAND_FLASH, 8}};	   /* NAND DEV */         
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
		 {
         {0, N_A, BOARD_DEV_NAND_FLASH, 8},	   /* NAND DEV */
         {1, N_A, BOARD_DEV_SPI_FLASH, 8},	   /* SPI DEV */
         };
#else
	 {{0, N_A, BOARD_DEV_SPI_FLASH, 8}};	   /* SPI DEV */         
#endif

MV_BOARD_MPP_INFO	db88f6280AInfoBoardMppConfigValue[] = 
	{{{
	DB_88F6280A_MPP0_7,		
	DB_88F6280A_MPP8_15,		
	DB_88F6280A_MPP16_23,		
	DB_88F6280A_MPP24_31,		
	DB_88F6280A_MPP32_39,		
	DB_88F6280A_MPP40_47,		
	DB_88F6280A_MPP48_55		
	}}};


MV_BOARD_INFO db88f6280AInfo = {
	"DB-88F6280A-BP",				/* boardName[MAX_BOARD_NAME_LEN] */
	DB_88F6280A_BOARD_MPP_GROUP_TYPE_NUM,		/* numBoardMppGroupType */
	db88f6280AInfoBoardMppTypeInfo,
	DB_88F6280A_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfig */
	db88f6280AInfoBoardMppConfigValue,
	0,						/* intsGppMaskLow */
	0,						/* intsGppMaskHigh */
	DB_88F6280A_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDevIf */
	db88f6280AInfoBoardDeCsInfo,
	DB_88F6280A_BOARD_TWSI_DEF_NUM,			/* numBoardTwsiDev */
	db88f6280AInfoBoardTwsiDev,					
	DB_88F6280A_BOARD_MAC_INFO_NUM,			/* numBoardMacInfo */
	db88f6280AInfoBoardMacInfo,
	DB_88F6280A_BOARD_GPP_INFO_NUM,			/* numBoardGppInfo */
	NULL,
	DB_88F6280A_BOARD_DEBUG_LED_NUM,			/* activeLedsNumber */              
	NULL,
	0,						/* ledsPolarity */		
	DB_88F6280A_OE_LOW,				/* gppOutEnLow */
	DB_88F6280A_OE_HIGH,				/* gppOutEnHigh */
	DB_88F6280A_OE_VAL_LOW,				/* gppOutValLow */
	DB_88F6280A_OE_VAL_HIGH,				/* gppOutValHigh */
	0,						/* gppPolarityValLow */
	BIT6, 						/* gppPolarityValHigh */
	NULL,						/* pSwitchInfo */
    DB_88F6280A_BOARD_NAND_READ_PARAMS,
    DB_88F6280A_BOARD_NAND_WRITE_PARAMS,
    DB_88F6280A_BOARD_NAND_CONTROL
};


#define DB_88F6282A_BOARD_PCI_IF_NUM            0x0
#define DB_88F6282A_BOARD_TWSI_DEF_NUM		    0x7
#define DB_88F6282A_BOARD_MAC_INFO_NUM		    0x2
#define DB_88F6282A_BOARD_GPP_INFO_NUM		    0x1
#define DB_88F6282A_BOARD_MPP_CONFIG_NUM		0x1
#define DB_88F6282A_BOARD_MPP_GROUP_TYPE_NUM	0x1
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
    #define DB_88F6282A_BOARD_DEVICE_CONFIG_NUM	    0x1
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
    #define DB_88F6282A_BOARD_DEVICE_CONFIG_NUM	    0x2
#else
    #define DB_88F6282A_BOARD_DEVICE_CONFIG_NUM	    0x1
#endif
#define DB_88F6282A_BOARD_DEBUG_LED_NUM		    0x0
#define DB_88F6282A_BOARD_NAND_READ_PARAMS		    0x000C0282
#define DB_88F6282A_BOARD_NAND_WRITE_PARAMS		    0x00010305
#define DB_88F6282A_BOARD_NAND_CONTROL		        0x01c00541


MV_BOARD_TWSI_INFO	db88f6282AInfoBoardTwsiDev[] =
	/* {{MV_BOARD_DEV_CLASS	devClass, MV_U8	twsiDevAddr, MV_U8 twsiDevAddrType}} */
	{
	{BOARD_DEV_TWSI_EXP, 0x20, ADDR7_BIT},
	{BOARD_DEV_TWSI_EXP, 0x21, ADDR7_BIT},
	{BOARD_DEV_TWSI_EXP, 0x27, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4C, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4D, ADDR7_BIT},
	{BOARD_DEV_TWSI_SATR, 0x4E, ADDR7_BIT},
	{BOARD_TWSI_AUDIO_DEC, 0x4A, ADDR7_BIT}
	};

MV_BOARD_MAC_INFO db88f6282AInfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{
	{BOARD_MAC_SPEED_AUTO, 0x8},
	{BOARD_MAC_SPEED_AUTO, 0x9}
	}; 

MV_BOARD_MPP_TYPE_INFO db88f6282AInfoBoardMppTypeInfo[] = 
	/* {{MV_BOARD_MPP_TYPE_CLASS	boardMppGroup1,
 		MV_BOARD_MPP_TYPE_CLASS	boardMppGroup2}} */
	{{MV_BOARD_AUTO, MV_BOARD_AUTO}
	}; 

MV_BOARD_GPP_INFO db88f6282AInfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{
	{BOARD_GPP_TSU_DIRCTION, 33}
	/*muxed with TDM/Audio module via IOexpender
	{BOARD_GPP_SDIO_DETECT, 38},
	{BOARD_GPP_USB_VBUS, 49}*/
	};

MV_DEV_CS_INFO db88f6282AInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
		 {{0, N_A, BOARD_DEV_NAND_FLASH, 8}};	   /* NAND DEV */         
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
		 {
         {0, N_A, BOARD_DEV_NAND_FLASH, 8},	   /* NAND DEV */
         {1, N_A, BOARD_DEV_SPI_FLASH, 8},	   /* SPI DEV */
         };
#else
	 {{1, N_A, BOARD_DEV_SPI_FLASH, 8}};	   /* SPI DEV */         
#endif

MV_BOARD_MPP_INFO	db88f6282AInfoBoardMppConfigValue[] = 
	{{{
	DB_88F6282A_MPP0_7,		
	DB_88F6282A_MPP8_15,		
	DB_88F6282A_MPP16_23,		
	DB_88F6282A_MPP24_31,		
	DB_88F6282A_MPP32_39,		
	DB_88F6282A_MPP40_47,		
	DB_88F6282A_MPP48_55		
	}}};


MV_BOARD_INFO db88f6282AInfo = {
	"DB-88F6282A-BP",				/* boardName[MAX_BOARD_NAME_LEN] */
	DB_88F6282A_BOARD_MPP_GROUP_TYPE_NUM,		/* numBoardMppGroupType */
	db88f6282AInfoBoardMppTypeInfo,
	DB_88F6282A_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfig */
	db88f6282AInfoBoardMppConfigValue,
	0,						/* intsGppMaskLow */
	0,						/* intsGppMaskHigh */
	DB_88F6282A_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDevIf */
	db88f6282AInfoBoardDeCsInfo,
	DB_88F6282A_BOARD_TWSI_DEF_NUM,			/* numBoardTwsiDev */
	db88f6282AInfoBoardTwsiDev,					
	DB_88F6282A_BOARD_MAC_INFO_NUM,			/* numBoardMacInfo */
	db88f6282AInfoBoardMacInfo,
	DB_88F6282A_BOARD_GPP_INFO_NUM,			/* numBoardGppInfo */
	db88f6282AInfoBoardGppInfo,
	DB_88F6282A_BOARD_DEBUG_LED_NUM,			/* activeLedsNumber */              
	NULL,
	0,						/* ledsPolarity */		
	DB_88F6282A_OE_LOW,				/* gppOutEnLow */
	DB_88F6282A_OE_HIGH,				/* gppOutEnHigh */
	DB_88F6282A_OE_VAL_LOW,				/* gppOutValLow */
	DB_88F6282A_OE_VAL_HIGH,				/* gppOutValHigh */
	0,						/* gppPolarityValLow */
	BIT6, 						/* gppPolarityValHigh */
	NULL,						/* pSwitchInfo */
    DB_88F6282A_BOARD_NAND_READ_PARAMS,
    DB_88F6282A_BOARD_NAND_WRITE_PARAMS,
    DB_88F6282A_BOARD_NAND_CONTROL
};


/* 6281 Sheeva Plug*/

#define SHEEVA_PLUG_BOARD_PCI_IF_NUM		        0x0
#define SHEEVA_PLUG_BOARD_TWSI_DEF_NUM		        0x0
#define SHEEVA_PLUG_BOARD_MAC_INFO_NUM		        0x1
#define SHEEVA_PLUG_BOARD_GPP_INFO_NUM		        0x0
#define SHEEVA_PLUG_BOARD_MPP_GROUP_TYPE_NUN        0x1
#define SHEEVA_PLUG_BOARD_MPP_CONFIG_NUM		    0x1
#define SHEEVA_PLUG_BOARD_DEVICE_CONFIG_NUM	        0x1
#define SHEEVA_PLUG_BOARD_DEBUG_LED_NUM		        0x1
#define SHEEVA_PLUG_BOARD_NAND_READ_PARAMS		    0x000E02C2
#define SHEEVA_PLUG_BOARD_NAND_WRITE_PARAMS		    0x00010305
#define SHEEVA_PLUG_BOARD_NAND_CONTROL		        0x01c00541

MV_U8	sheevaPlugInfoBoardDebugLedIf[] =
	{49};

MV_BOARD_MAC_INFO sheevaPlugInfoBoardMacInfo[] = 
    /* {{MV_BOARD_MAC_SPEED	boardMacSpeed,	MV_U8	boardEthSmiAddr}} */
	{{BOARD_MAC_SPEED_AUTO, 0x0}}; 

MV_BOARD_TWSI_INFO	sheevaPlugInfoBoardTwsiDev[] =
	/* {{MV_BOARD_DEV_CLASS	devClass, MV_U8	twsiDevAddr, MV_U8 twsiDevAddrType}} */
	{{BOARD_TWSI_OTHER, 0x0, ADDR7_BIT}};

MV_BOARD_MPP_TYPE_INFO sheevaPlugInfoBoardMppTypeInfo[] = 
	{{MV_BOARD_OTHER, MV_BOARD_OTHER}
	}; 

MV_DEV_CS_INFO sheevaPlugInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
		 {{0, N_A, BOARD_DEV_NAND_FLASH, 8}};	   /* NAND DEV */

MV_BOARD_MPP_INFO	sheevaPlugInfoBoardMppConfigValue[] = 
	{{{
	RD_SHEEVA_PLUG_MPP0_7,		
	RD_SHEEVA_PLUG_MPP8_15,		
	RD_SHEEVA_PLUG_MPP16_23,		
	RD_SHEEVA_PLUG_MPP24_31,		
	RD_SHEEVA_PLUG_MPP32_39,		
	RD_SHEEVA_PLUG_MPP40_47,		
	RD_SHEEVA_PLUG_MPP48_55		
	}}};

MV_BOARD_INFO sheevaPlugInfo = {
	"SHEEVA PLUG",				                /* boardName[MAX_BOARD_NAME_LEN] */
	SHEEVA_PLUG_BOARD_MPP_GROUP_TYPE_NUN,		/* numBoardMppGroupType */
	sheevaPlugInfoBoardMppTypeInfo,
	SHEEVA_PLUG_BOARD_MPP_CONFIG_NUM,		    /* numBoardMppConfig */
	sheevaPlugInfoBoardMppConfigValue,
	0,						                    /* intsGppMaskLow */
	0,					                        /* intsGppMaskHigh */
	SHEEVA_PLUG_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDevIf */
	sheevaPlugInfoBoardDeCsInfo,
	SHEEVA_PLUG_BOARD_TWSI_DEF_NUM,			    /* numBoardTwsiDev */
	sheevaPlugInfoBoardTwsiDev,					
	SHEEVA_PLUG_BOARD_MAC_INFO_NUM,			    /* numBoardMacInfo */
	sheevaPlugInfoBoardMacInfo,
	SHEEVA_PLUG_BOARD_GPP_INFO_NUM,			    /* numBoardGppInfo */
	0,
	SHEEVA_PLUG_BOARD_DEBUG_LED_NUM,			/* activeLedsNumber */              
	sheevaPlugInfoBoardDebugLedIf,
	0,										/* ledsPolarity */		
	RD_SHEEVA_PLUG_OE_LOW,				            /* gppOutEnLow */
	RD_SHEEVA_PLUG_OE_HIGH,				        /* gppOutEnHigh */
	RD_SHEEVA_PLUG_OE_VAL_LOW,				        /* gppOutValLow */
	RD_SHEEVA_PLUG_OE_VAL_HIGH,				    /* gppOutValHigh */
	0,						                    /* gppPolarityValLow */
	0, 						                    /* gppPolarityValHigh */
	NULL,						/* pSwitchInfo */
    SHEEVA_PLUG_BOARD_NAND_READ_PARAMS,
    SHEEVA_PLUG_BOARD_NAND_WRITE_PARAMS,
    SHEEVA_PLUG_BOARD_NAND_CONTROL
};

/* Customer specific board place holder*/

#define DB_CUSTOMER_BOARD_PCI_IF_NUM		        0x0
#define DB_CUSTOMER_BOARD_TWSI_DEF_NUM		        0x0
#define DB_CUSTOMER_BOARD_MAC_INFO_NUM		        0x0
#define DB_CUSTOMER_BOARD_GPP_INFO_NUM		        0x0
#define DB_CUSTOMER_BOARD_MPP_GROUP_TYPE_NUN        0x0
#define DB_CUSTOMER_BOARD_MPP_CONFIG_NUM		    0x0
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
    #define DB_CUSTOMER_BOARD_DEVICE_CONFIG_NUM	    0x0
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
    #define DB_CUSTOMER_BOARD_DEVICE_CONFIG_NUM	    0x0
#else
    #define DB_CUSTOMER_BOARD_DEVICE_CONFIG_NUM	    0x0
#endif
#define DB_CUSTOMER_BOARD_DEBUG_LED_NUM		0x0
#define DB_CUSTOMER_BOARD_NAND_READ_PARAMS		    0x000E02C2
#define DB_CUSTOMER_BOARD_NAND_WRITE_PARAMS		    0x00010305
#define DB_CUSTOMER_BOARD_NAND_CONTROL		        0x01c00541

MV_U8	dbCustomerInfoBoardDebugLedIf[] =
	{0};

MV_BOARD_MAC_INFO dbCustomerInfoBoardMacInfo[] = 
    /* {{MV_BOARD_MAC_SPEED	boardMacSpeed,	MV_U8	boardEthSmiAddr}} */
	{{BOARD_MAC_SPEED_AUTO, 0x0}}; 

MV_BOARD_TWSI_INFO	dbCustomerInfoBoardTwsiDev[] =
	/* {{MV_BOARD_DEV_CLASS	devClass, MV_U8	twsiDevAddr, MV_U8 twsiDevAddrType}} */
	{{BOARD_TWSI_OTHER, 0x0, ADDR7_BIT}};

MV_BOARD_MPP_TYPE_INFO dbCustomerInfoBoardMppTypeInfo[] = 
	{{MV_BOARD_OTHER, MV_BOARD_1_8V, MV_BOARD_OTHER, MV_BOARD_1_8V}
	}; 

MV_DEV_CS_INFO dbCustomerInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
#if defined(MV_NAND) && defined(MV_NAND_BOOT)
		 {{0, N_A, BOARD_DEV_NAND_FLASH, 8}};	   /* NAND DEV */
#elif defined(MV_NAND) && defined(MV_SPI_BOOT)
		 {
         {0, N_A, BOARD_DEV_NAND_FLASH, 8},	   /* NAND DEV */
         {2, N_A, BOARD_DEV_SPI_FLASH, 8},	   /* SPI DEV */
         };
#else
		 {{2, N_A, BOARD_DEV_SPI_FLASH, 8}};	   /* SPI DEV */         
#endif

MV_BOARD_MPP_INFO	dbCustomerInfoBoardMppConfigValue[] = 
	{{{
	DB_CUSTOMER_MPP0_7,		
	DB_CUSTOMER_MPP8_15,		
	DB_CUSTOMER_MPP16_23,		
	DB_CUSTOMER_MPP24_31,		
	DB_CUSTOMER_MPP32_39,		
	DB_CUSTOMER_MPP40_47,		
	DB_CUSTOMER_MPP48_55		
	}}};

MV_BOARD_INFO dbCustomerInfo = {
	"DB-CUSTOMER",				                /* boardName[MAX_BOARD_NAME_LEN] */
	DB_CUSTOMER_BOARD_MPP_GROUP_TYPE_NUN,		/* numBoardMppGroupType */
	dbCustomerInfoBoardMppTypeInfo,
	DB_CUSTOMER_BOARD_MPP_CONFIG_NUM,		    /* numBoardMppConfig */
	dbCustomerInfoBoardMppConfigValue,
	0,						                    /* intsGppMaskLow */
	0,					                        /* intsGppMaskHigh */
	DB_CUSTOMER_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDevIf */
	dbCustomerInfoBoardDeCsInfo,
	DB_CUSTOMER_BOARD_TWSI_DEF_NUM,			    /* numBoardTwsiDev */
	dbCustomerInfoBoardTwsiDev,					
	DB_CUSTOMER_BOARD_MAC_INFO_NUM,			    /* numBoardMacInfo */
	dbCustomerInfoBoardMacInfo,
	DB_CUSTOMER_BOARD_GPP_INFO_NUM,			    /* numBoardGppInfo */
	0,
	DB_CUSTOMER_BOARD_DEBUG_LED_NUM,			/* activeLedsNumber */              
	NULL,
	0,										/* ledsPolarity */		
	DB_CUSTOMER_OE_LOW,				            /* gppOutEnLow */
	DB_CUSTOMER_OE_HIGH,				        /* gppOutEnHigh */
	DB_CUSTOMER_OE_VAL_LOW,				        /* gppOutValLow */
	DB_CUSTOMER_OE_VAL_HIGH,				    /* gppOutValHigh */
	0,						                    /* gppPolarityValLow */
	0, 						                    /* gppPolarityValHigh */
	NULL,						/* pSwitchInfo */
    DB_CUSTOMER_BOARD_NAND_READ_PARAMS,
    DB_CUSTOMER_BOARD_NAND_WRITE_PARAMS,
    DB_CUSTOMER_BOARD_NAND_CONTROL
};

MV_BOARD_INFO*	boardInfoTbl[] = 	{
                    &db88f6281AInfo,
                    &rd88f6281AInfo,
                    &db88f6192AInfo,
                    &rd88f6192AInfo,
                    &db88f6180AInfo,
                    &db88f6190AInfo,
                    &rd88f6190AInfo,
                    &rd88f6281APcacInfo,
                    &dbCustomerInfo,
                    &sheevaPlugInfo,
                    &db88f6280AInfo,
                    &db88f6282AInfo
					};


////////////////////////////////////////////////////////////////////////////////

#else // defined(CONFIG_BUFFALO_PLATFORM)

/****************************** MVLSXH ******************************/
MV_BOARD_MAC_INFO mvlsxhInfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{
		{BOARD_MAC_SPEED_AUTO, 0x0},
		{BOARD_MAC_SPEED_AUTO, 0x8}
	}; 

MV_BOARD_GPP_INFO mvlsxhInfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{
		{BOARD_GPP_HDD_POWER, 10},
		{BOARD_GPP_USB_VBUS_EN, 11},
		{BOARD_GPP_FAN_LOW, 18},
		{BOARD_GPP_FAN_HIGH, 19},
		{BOARD_GPP_FUNC_LED, 36},
		{BOARD_GPP_ALARM_LED, 37},
		{BOARD_GPP_INFO_LED, 38},
		{BOARD_GPP_PWR_LED, 39},
		{BOARD_GPP_FAN_LOCK, 40},
		{BOARD_GPP_FUNC_SW, 41},
		{BOARD_GPP_PWR_SW, 42},
		{BOARD_GPP_PWRAUTO_SW, 43},
		{BOARD_GPP_FUNC_RED_LED, 48},
		{BOARD_GPP_UART_EN, 49},
	};

MV_BOARD_MPP_TYPE_INFO mvlsxhInfoBoardMppTypeInfo[] = 
	/* {{MV_BOARD_MPP_TYPE_CLASS	boardMppGroup1,
 		MV_BOARD_MPP_TYPE_CLASS	boardMppGroup2}} */
	{{MV_BOARD_OTHER,MV_BOARD_RGMII}
	}; 

MV_BOARD_MPP_INFO	mvlsxhInfoBoardMppConfigValue[] = 
	{{{
	MVLSXH_MPP0_7,		
	MVLSXH_MPP8_15,		
	MVLSXH_MPP16_23,		
	MVLSXH_MPP24_31,		
	MVLSXH_MPP32_39,
	MVLSXH_MPP40_47,
	MVLSXH_MPP48_55
	}}};

MV_DEV_CS_INFO mvlsxhInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
		 {{1, N_A, BOARD_DEV_SPI_FLASH, 8}};	   /* SPI DEV */

#define MVLSXH_BOARD_PCI_IF_NUM			0x0
#define MVLSXH_BOARD_TWSI_DEF_NUM		0x0
#define MVLSXH_BOARD_MAC_INFO_NUM		0x2
#define MVLSXH_BOARD_GPP_INFO_NUM		(sizeof(mvlsxhInfoBoardGppInfo)/sizeof(MV_BOARD_GPP_INFO))
#define MVLSXH_BOARD_MPP_GROUP_TYPE_NUM		0x1
#define MVLSXH_BOARD_MPP_CONFIG_NUM		0x1
#define MVLSXH_BOARD_DEVICE_CONFIG_NUM		0x1
#define MVLSXH_BOARD_NAND_READ_PARAMS		0x003E07CF
#define MVLSXH_BOARD_NAND_WRITE_PARAMS		0x000F0F0F
#define MVLSXH_BOARD_NAND_CONTROL		0x01c7D943

MV_BOARD_INFO mvlsxhInfo = {
	"MVLSXH",				/* boardName[MAX_BOARD_NAME_LEN] */
	MVLSXH_BOARD_MPP_GROUP_TYPE_NUM,	/* numBoardMppTypeValue */
	mvlsxhInfoBoardMppTypeInfo,		/* pBoardMppTypeValue */
	MVLSXH_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfigValue */
	mvlsxhInfoBoardMppConfigValue,		/* pBoardMppConfigValue */
	0,					/* intsGppMaskLow */
	0,					/* intsGppMaskHigh */
	MVLSXH_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDeviceIf */
	mvlsxhInfoBoardDeCsInfo,		/* pDevCsInfo */
	MVLSXH_BOARD_TWSI_DEF_NUM,		/* numBoardTwsiDev */
	NULL,					/* pBoardTwsiDev */
	MVLSXH_BOARD_MAC_INFO_NUM,		/* numBoardMacInfo */
	mvlsxhInfoBoardMacInfo,			/* pBoardMacInfo */
	MVLSXH_BOARD_GPP_INFO_NUM,		/* numBoardGppInfo */
	mvlsxhInfoBoardGppInfo,			/* pBoardGppInfo */
	0,					/* activeLedsNumber */              
	NULL,					/* pLedGppPin */
	0,					/* ledsPolarity */		
	MVLSXH_OE_LOW,				/* gppOutEnLow */
	MVLSXH_OE_HIGH,				/* gppOutEnHigh */
	MVLSXH_OE_VAL_LOW,			/* gppOutValLow */
	MVLSXH_OE_VAL_HIGH,			/* gppOutValHigh */
	MVLSXH_POLARITY_VAL_LOW,		/* gppPolarityValLow */
	MVLSXH_POLARITY_VAL_HIGH,		/* gppPolarityValHigh */
	NULL,					/* pSwitchInfo */
	MVLSXH_BOARD_NAND_READ_PARAMS,		/* nandFlashReadParams */
	MVLSXH_BOARD_NAND_WRITE_PARAMS,		/* nandFlashWriteParams */
	MVLSXH_BOARD_NAND_CONTROL,		/* nandFlashControl */
};

MV_BOARD_INFO mvlsxlInfo = {
	"MVLSXL",				/* boardName[MAX_BOARD_NAME_LEN] */
	MVLSXH_BOARD_MPP_GROUP_TYPE_NUM,	/* numBoardMppTypeValue */
	mvlsxhInfoBoardMppTypeInfo,		/* pBoardMppTypeValue */
	MVLSXH_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfigValue */
	mvlsxhInfoBoardMppConfigValue,		/* pBoardMppConfigValue */
	0,					/* intsGppMaskLow */
	0,					/* intsGppMaskHigh */
	MVLSXH_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDeviceIf */
	mvlsxhInfoBoardDeCsInfo,		/* pDevCsInfo */
	MVLSXH_BOARD_TWSI_DEF_NUM,		/* numBoardTwsiDev */
	NULL,					/* pBoardTwsiDev */
	MVLSXH_BOARD_MAC_INFO_NUM,		/* numBoardMacInfo */
	mvlsxhInfoBoardMacInfo,			/* pBoardMacInfo */
	MVLSXH_BOARD_GPP_INFO_NUM,		/* numBoardGppInfo */
	mvlsxhInfoBoardGppInfo,			/* pBoardGppInfo */
	0,					/* activeLedsNumber */              
	NULL,					/* pLedGppPin */
	0,					/* ledsPolarity */		
	MVLSXH_OE_LOW,				/* gppOutEnLow */
	MVLSXH_OE_HIGH,				/* gppOutEnHigh */
	MVLSXH_OE_VAL_LOW,			/* gppOutValLow */
	MVLSXH_OE_VAL_HIGH,			/* gppOutValHigh */
	MVLSXH_POLARITY_VAL_LOW,		/* gppPolarityValLow */
	MVLSXH_POLARITY_VAL_HIGH,		/* gppPolarityValHigh */
	NULL,					/* pSwitchInfo */
	MVLSXH_BOARD_NAND_READ_PARAMS,		/* nandFlashReadParams */
	MVLSXH_BOARD_NAND_WRITE_PARAMS,		/* nandFlashWriteParams */
	MVLSXH_BOARD_NAND_CONTROL,		/* nandFlashControl */
};

/****************************** LS-XL-V2 ******************************/
MV_BOARD_MAC_INFO mvlsxlGeV2InfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{
		{BOARD_MAC_SPEED_AUTO, 0x0},
	};

MV_BOARD_GPP_INFO mvlsxlGeV2InfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{
		{BOARD_GPP_PWR_LED, 17},
		{BOARD_GPP_HDD_POWER, 14},
		{BOARD_GPP_ALARM_LED, 26},
		{BOARD_GPP_INFO_LED, 27},
	};

MV_BOARD_MPP_INFO	mvlsxlGeV2InfoBoardMppConfigValue[] = 
	{{{
	MVLSXL_GE_V2_MPP0_7,
	MVLSXL_GE_V2_MPP8_15,
	MVLSXL_GE_V2_MPP16_23,
	MVLSXL_GE_V2_MPP24_31,
	MVLSXL_GE_V2_MPP32_35
	}}};

MV_DEV_CS_INFO mvlsxlGeV2InfoBoardDeCsInfo[] = 
	/*{deviceCS, params, devType, devWidth}*/
	{
		{0, N_A, BOARD_DEV_NAND_FLASH, 8},	/* NAND DEV */
		{1, N_A, BOARD_DEV_SPI_FLASH, 8},	/* SPI DEV */
	};

#define MVLSXL_GE_V2_BOARD_PCI_IF_NUM		0x0
#define MVLSXL_GE_V2_BOARD_TWSI_DEF_NUM		0x0
#define MVLSXL_GE_V2_BOARD_MAC_INFO_NUM		0x1
#define MVLSXL_GE_V2_BOARD_GPP_INFO_NUM		(sizeof(mvlsxlGeV2InfoBoardGppInfo)/sizeof(MV_BOARD_GPP_INFO))
#define MVLSXL_GE_V2_BOARD_MPP_GROUP_TYPE_NUM	0x1
#define MVLSXL_GE_V2_BOARD_MPP_CONFIG_NUM	0x1
#define MVLSXL_GE_V2_BOARD_DEVICE_CONFIG_NUM	0x2
#define MVLSXL_GE_V2_BOARD_NAND_READ_PARAMS	0x003E07CF
#define MVLSXL_GE_V2_BOARD_NAND_WRITE_PARAMS	0x000F0F0F
#define MVLSXL_GE_V2_BOARD_NAND_CONTROL		0x01c7D943

MV_BOARD_MPP_TYPE_INFO mvlsxlv2InfoBoardMppTypeInfo[] = 
	{{MV_BOARD_AUTO, MV_BOARD_1_8V, MV_BOARD_OTHER, MV_BOARD_3_3V}
	}; 

MV_BOARD_INFO mvlsxlGeV2Info = {
	"MVLSXL-GE-V2",				/* boardName[MAX_BOARD_NAME_LEN] */
	MVLSXH_BOARD_MPP_GROUP_TYPE_NUM,	/* numBoardMppTypeValue */
	mvlsxlv2InfoBoardMppTypeInfo,		/* pBoardMppTypeValue */
	/* mvlsxhInfoBoardMppTypeInfo, */		/* pBoardMppTypeValue */
	MVLSXL_GE_V2_BOARD_MPP_CONFIG_NUM,	/* numBoardMppConfigValue */
	mvlsxlGeV2InfoBoardMppConfigValue,	/* pBoardMppConfigValue */
	0,					/* intsGppMaskLow */
	0,					/* intsGppMaskHigh */
	MVLSXL_GE_V2_BOARD_DEVICE_CONFIG_NUM,	/* numBoardDeviceIf */
	mvlsxlGeV2InfoBoardDeCsInfo,		/* pDevCsInfo */
	MVLSXH_BOARD_TWSI_DEF_NUM,		/* numBoardTwsiDev */
	NULL,					/* pBoardTwsiDev */
	MVLSXL_GE_V2_BOARD_MAC_INFO_NUM,	/* numBoardMacInfo */
	mvlsxlGeV2InfoBoardMacInfo,		/* pBoardMacInfo */
	MVLSXL_GE_V2_BOARD_GPP_INFO_NUM,	/* numBoardGppInfo */
	mvlsxlGeV2InfoBoardGppInfo,		/* pBoardGppInfo */
	0,					/* activeLedsNumber */
	NULL,					/* pLedGppPin */
	0,					/* ledsPolarity */
	MVLSXL_GE_V2_OE_LOW,			/* gppOutEnLow */
	MVLSXL_GE_V2_OE_HIGH,			/* gppOutEnHigh */
	MVLSXL_GE_V2_OE_VAL_LOW,		/* gppOutValLow */
	MVLSXL_GE_V2_OE_VAL_HIGH,		/* gppOutValHigh */
	MVLSXL_GE_V2_POLARITY_VAL_LOW,		/* gppPolarityValLow */
	MVLSXL_GE_V2_POLARITY_VAL_HIGH,		/* gppPolarityValHigh */
	NULL,					/* pSwitchInfo */
	MVLSXL_GE_V2_BOARD_NAND_READ_PARAMS,	/* nandFlashReadParams */
	MVLSXL_GE_V2_BOARD_NAND_WRITE_PARAMS,	/* nandFlashWriteParams */
	MVLSXL_GE_V2_BOARD_NAND_CONTROL,	/* nandFlashControl */
};

/****************************** LS-WXL ******************************/
MV_BOARD_GPP_INFO mvwxlInfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{
		{BOARD_GPP_HDD_ERR_LED,  8},
		{BOARD_GPP_HDD_POWER,	28},
		{BOARD_GPP_HDD_POWER,	29},
		{BOARD_GPP_FUNC_RED_LED,34},
		{BOARD_GPP_FUNC_LED,	36},
		{BOARD_GPP_USB_VBUS_EN, 37},
		{BOARD_GPP_INFO_LED,	38},
		{BOARD_GPP_PWR_LED,	39},
		{BOARD_GPP_FAN_LOCK,	40},
		{BOARD_GPP_FUNC_SW,	41},
		{BOARD_GPP_PWR_SW,	42},
		{BOARD_GPP_PWRAUTO_SW,	43},
		{BOARD_GPP_HDD_ERR_LED, 46},
		{BOARD_GPP_FAN_LOW,	47},
		{BOARD_GPP_FAN_HIGH,	48},
		{BOARD_GPP_ALARM_LED,	49},
		{BOARD_GPP_UART_EN,	49},
	};

MV_BOARD_MPP_INFO	mvwxlInfoBoardMppConfigValue[] = 
	{{{
	MVWXL_MPP0_7,		
	MVWXL_MPP8_15,		
	MVWXL_MPP16_23,		
	MVWXL_MPP24_31,		
	MVWXL_MPP32_39,
	MVWXL_MPP40_47,
	MVWXL_MPP48_55
	}}};

MV_DEV_CS_INFO mvwxlInfoBoardDeCsInfo[] = 
	/*{deviceCS, params, devType, devWidth}*/
	{
#if defined(MV_NAND)
		{0, N_A, BOARD_DEV_NAND_FLASH, 8},	/* NAND DEV */
#endif
#if defined(MV_SPI)
		{1, N_A, BOARD_DEV_SPI_FLASH, 8},	/* SPI DEV */
#endif
	};


#define MVWXL_BOARD_PCI_IF_NUM			0x0
#define MVWXL_BOARD_TWSI_DEF_NUM		0x0
#define MVWXL_BOARD_MAC_INFO_NUM		0x2
#define MVWXL_BOARD_GPP_INFO_NUM		(sizeof(mvwxlInfoBoardGppInfo)/sizeof(MV_BOARD_GPP_INFO))
#define MVWXL_BOARD_MPP_GROUP_TYPE_NUM		0x1
#define MVWXL_BOARD_MPP_CONFIG_NUM		0x1
#define MVWXL_BOARD_DEVICE_CONFIG_NUM		(sizeof(mvwxlInfoBoardDeCsInfo)/sizeof(MV_DEV_CS_INFO))
#define MVWXL_BOARD_NAND_READ_PARAMS		0x003E07CF
//#define MVWXL_BOARD_NAND_READ_PARAMS		0x000C0282
#define MVWXL_BOARD_NAND_WRITE_PARAMS		0x000F0F0F
//#define MVWXL_BOARD_NAND_WRITE_PARAMS		0x00010305
#define MVWXL_BOARD_NAND_CONTROL		0x01c7D943
//#define MVWXL_BOARD_NAND_CONTROL		0x01c00541

MV_BOARD_INFO mvwxlInfo = {
	"MVWXL",				/* boardName[MAX_BOARD_NAME_LEN] */
	MVWXL_BOARD_MPP_GROUP_TYPE_NUM,		/* numBoardMppTypeValue */
	mvlsxhInfoBoardMppTypeInfo,		/* pBoardMppTypeValue */
	MVWXL_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfigValue */
	mvwxlInfoBoardMppConfigValue,		/* pBoardMppConfigValue */
	0,					/* intsGppMaskLow */
	0,					/* intsGppMaskHigh */
	MVWXL_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDeviceIf */
	mvwxlInfoBoardDeCsInfo,			/* pDevCsInfo */
	MVWXL_BOARD_TWSI_DEF_NUM,		/* numBoardTwsiDev */
	NULL,					/* pBoardTwsiDev */
	MVWXL_BOARD_MAC_INFO_NUM,		/* numBoardMacInfo */
	mvlsxhInfoBoardMacInfo,			/* pBoardMacInfo */
	MVWXL_BOARD_GPP_INFO_NUM,		/* numBoardGppInfo */
	mvwxlInfoBoardGppInfo,			/* pBoardGppInfo */
	0,					/* activeLedsNumber */
	NULL,					/* pLedGppPin */
	0,					/* ledsPolarity */
	MVWXL_OE_LOW,				/* gppOutEnLow */
	MVWXL_OE_HIGH,				/* gppOutEnHigh */
	MVWXL_OE_VAL_LOW,			/* gppOutValLow */
	MVWXL_OE_VAL_HIGH,			/* gppOutValHigh */
	MVWXL_POLARITY_VAL_LOW,			/* gppPolarityValLow */
	MVWXL_POLARITY_VAL_HIGH,		/* gppPolarityValHigh */
	NULL,					/* pSwitchInfo */
	MVWXL_BOARD_NAND_READ_PARAMS,		/* nandFlashReadParams */
	MVWXL_BOARD_NAND_WRITE_PARAMS,		/* nandFlashWriteParams */
	MVWXL_BOARD_NAND_CONTROL,		/* nandFlashControl */
};

/****************************** LS-WSX ******************************/
MV_BOARD_GPP_INFO mvwssxInfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{
		{BOARD_GPP_HDD_POWER,		28},
		{BOARD_GPP_HDD_POWER,		35},
		{BOARD_GPP_FUNC_RED_LED,	34},
		{BOARD_GPP_FUNC_LED,		36},
		{BOARD_GPP_USB_VBUS_EN, 	37},
		{BOARD_GPP_INFO_LED,		38},
		{BOARD_GPP_PWR_LED,		39},
		{BOARD_GPP_FAN_LOCK,		40},
		{BOARD_GPP_FUNC_SW,		41},
		{BOARD_GPP_PWR_SW,		42},
		{BOARD_GPP_PWRAUTO_SW,		43},
		{BOARD_GPP_DAS_PWR_LED, 	46},
		{BOARD_GPP_USB_HOST_DEVICE,	47},
		{BOARD_GPP_ALARM_LED,		49},
		{BOARD_GPP_UART_EN,		49},
	};

MV_BOARD_MPP_INFO	mvwssxInfoBoardMppConfigValue[] = 
	{{{
	MVWSSX_MPP0_7,
	MVWSSX_MPP8_15,
	MVWSSX_MPP16_23,
	MVWSSX_MPP24_31,
	MVWSSX_MPP32_39,
	MVWSSX_MPP40_47,
	MVWSSX_MPP48_55
	}}};

MV_DEV_CS_INFO mvwssxInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/
	{
		{0, N_A, BOARD_DEV_NAND_FLASH, 8},	/* NAND DEV */
		{1, N_A, BOARD_DEV_SPI_FLASH, 8},	/* SPI DEV */
	};


#define MVWSSX_BOARD_PCI_IF_NUM			0x0
#define MVWSSX_BOARD_TWSI_DEF_NUM		0x0
#define MVWSSX_BOARD_MAC_INFO_NUM		0x2
#define MVWSSX_BOARD_GPP_INFO_NUM		(sizeof(mvwssxInfoBoardGppInfo)/sizeof(MV_BOARD_GPP_INFO))
#define MVWSSX_BOARD_MPP_GROUP_TYPE_NUM		0x1
#define MVWSSX_BOARD_MPP_CONFIG_NUM		0x1
#define MVWSSX_BOARD_DEVICE_CONFIG_NUM		0x2

MV_BOARD_INFO mvwssxInfo = {
	"MVWSSX",				/* boardName[MAX_BOARD_NAME_LEN] */
	MVWSSX_BOARD_MPP_GROUP_TYPE_NUM,	/* numBoardMppTypeValue */
	mvlsxhInfoBoardMppTypeInfo,		/* pBoardMppTypeValue */
	MVWSSX_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfigValue */
	mvwssxInfoBoardMppConfigValue,		/* pBoardMppConfigValue */
	0,					/* intsGppMaskLow */
	0,					/* intsGppMaskHigh */
	MVWSSX_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDeviceIf */
	mvwssxInfoBoardDeCsInfo,		/* pDevCsInfo */
	MVWSSX_BOARD_TWSI_DEF_NUM,		/* numBoardTwsiDev */
	NULL,					/* pBoardTwsiDev */
	MVWSSX_BOARD_MAC_INFO_NUM,		/* numBoardMacInfo */
	mvlsxhInfoBoardMacInfo,			/* pBoardMacInfo */
	MVWSSX_BOARD_GPP_INFO_NUM,		/* numBoardGppInfo */
	mvwssxInfoBoardGppInfo,			/* pBoardGppInfo */
	0,					/* activeLedsNumber */              
	NULL,					/* pLedGppPin */
	0,					/* ledsPolarity */		
	MVWSSX_OE_LOW,				/* gppOutEnLow */
	MVWSSX_OE_HIGH,				/* gppOutEnHigh */
	MVWSSX_OE_VAL_LOW,			/* gppOutValLow */
	MVWSSX_OE_VAL_HIGH,			/* gppOutValHigh */
	MVWSSX_POLARITY_VAL_LOW,		/* gppPolarityValLow */
	MVWSSX_POLARITY_VAL_HIGH,		/* gppPolarityValHigh */
	NULL,					/* pSwitchInfo */
	MVWXL_BOARD_NAND_READ_PARAMS,		/* nandFlashReadParams */
	MVWXL_BOARD_NAND_WRITE_PARAMS,		/* nandFlashWriteParams */
	MVWXL_BOARD_NAND_CONTROL,		/* nandFlashControl */
};

/****************************** MVLSVL ******************************/
MV_BOARD_MAC_INFO mvlsvlInfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{
		{BOARD_MAC_SPEED_AUTO, 0x0}
	}; 

MV_BOARD_GPP_INFO mvlsvlInfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{
		{BOARD_GPP_HDD_POWER, 8},
		{BOARD_GPP_USB_VBUS_EN, 12},
		{BOARD_GPP_FAN_LOW, 16},
		{BOARD_GPP_FAN_HIGH, 17},
		{BOARD_GPP_FUNC_LED, 39},
		{BOARD_GPP_ALARM_LED, 36},
		{BOARD_GPP_INFO_LED, 38},
		{BOARD_GPP_PWR_LED, 40},
		{BOARD_GPP_FAN_LOCK, 43},
		{BOARD_GPP_FUNC_SW, 45},
		{BOARD_GPP_PWR_SW, 46},
		{BOARD_GPP_PWRAUTO_SW, 47},
		{BOARD_GPP_FUNC_RED_LED, 37},
		{BOARD_GPP_UART_EN, 48},
	};

MV_BOARD_MPP_TYPE_INFO mvlsvlInfoBoardMppTypeInfo[] = 
	/* {{MV_BOARD_MPP_TYPE_CLASS	boardMppGroup1,
 		MV_BOARD_MPP_TYPE_CLASS	boardMppGroup2}} */
	{
		{MV_BOARD_OTHER, MV_BOARD_OTHER}
	//{MV_BOARD_OTHER,MV_BOARD_RGMII}
	}; 

MV_BOARD_MPP_INFO	mvlsvlInfoBoardMppConfigValue[] = 
	{{{
	MVLSVL_MPP0_7,		
	MVLSVL_MPP8_15,		
	MVLSVL_MPP16_23,		
	MVLSVL_MPP24_31,		
	MVLSVL_MPP32_39,
	MVLSVL_MPP40_47,
	MVLSVL_MPP48_55
	}}};

MV_DEV_CS_INFO mvlsvlInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
		 {{1, N_A, BOARD_DEV_SPI_FLASH, 8}};	   /* SPI DEV */

#define MVLSVL_BOARD_PCI_IF_NUM			0x0
#define MVLSVL_BOARD_TWSI_DEF_NUM		0x0
#define MVLSVL_BOARD_MAC_INFO_NUM		0x1
#define MVLSVL_BOARD_GPP_INFO_NUM		(sizeof(mvlsvlInfoBoardGppInfo)/sizeof(MV_BOARD_GPP_INFO))
#define MVLSVL_BOARD_MPP_GROUP_TYPE_NUM		0x0
#define MVLSVL_BOARD_MPP_CONFIG_NUM		0x1
#define MVLSVL_BOARD_DEVICE_CONFIG_NUM		0x1
#define MVLSVL_BOARD_NAND_READ_PARAMS		0x003E07CF
#define MVLSVL_BOARD_NAND_WRITE_PARAMS		0x000F0F0F
#define MVLSVL_BOARD_NAND_CONTROL		0x01c7D943

MV_BOARD_INFO mvlsvlInfo = {
	"MVLSVL",				/* boardName[MAX_BOARD_NAME_LEN] */
	MVLSVL_BOARD_MPP_GROUP_TYPE_NUM,	/* numBoardMppTypeValue */
	mvlsvlInfoBoardMppTypeInfo,		/* pBoardMppTypeValue */
	MVLSVL_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfigValue */
	mvlsvlInfoBoardMppConfigValue,		/* pBoardMppConfigValue */
	0,					/* intsGppMaskLow */
	0,					/* intsGppMaskHigh */
	MVLSVL_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDeviceIf */
	mvlsxhInfoBoardDeCsInfo,		/* pDevCsInfo */
	MVLSVL_BOARD_TWSI_DEF_NUM,		/* numBoardTwsiDev */
	NULL,					/* pBoardTwsiDev */
	MVLSVL_BOARD_MAC_INFO_NUM,		/* numBoardMacInfo */
	mvlsvlInfoBoardMacInfo,			/* pBoardMacInfo */
	MVLSVL_BOARD_GPP_INFO_NUM,		/* numBoardGppInfo */
	mvlsvlInfoBoardGppInfo,			/* pBoardGppInfo */
	0,					/* activeLedsNumber */              
	NULL,					/* pLedGppPin */
	0,					/* ledsPolarity */		
	MVLSVL_OE_LOW,				/* gppOutEnLow */
	MVLSVL_OE_HIGH,				/* gppOutEnHigh */
	MVLSVL_OE_VAL_LOW,			/* gppOutValLow */
	MVLSVL_OE_VAL_HIGH,			/* gppOutValHigh */
	MVLSVL_POLARITY_VAL_LOW,		/* gppPolarityValLow */
	MVLSVL_POLARITY_VAL_HIGH,		/* gppPolarityValHigh */
	NULL,					/* pSwitchInfo */
	MVLSVL_BOARD_NAND_READ_PARAMS,		/* nandFlashReadParams */
	MVLSVL_BOARD_NAND_WRITE_PARAMS,		/* nandFlashWriteParams */
	MVLSVL_BOARD_NAND_CONTROL,		/* nandFlashControl */
};

/****************************** MVLSWV ******************************/
MV_BOARD_MAC_INFO mvlswvInfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{
		{BOARD_MAC_SPEED_AUTO, 0x0}
	};

MV_BOARD_GPP_INFO mvlswvInfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{
		{BOARD_GPP_HDD_POWER, 8},
		{BOARD_GPP_HDD_POWER, 9},
		{BOARD_GPP_USB_VBUS_EN, 12},
		{BOARD_GPP_FAN_LOW, 16},
		{BOARD_GPP_FAN_HIGH, 17},
		{BOARD_GPP_FUNC_LED, 39},
		{BOARD_GPP_ALARM_LED, 36},
		{BOARD_GPP_INFO_LED, 38},
		{BOARD_GPP_PWR_LED, 40},
		{BOARD_GPP_FAN_LOCK, 43},
		{BOARD_GPP_FUNC_SW, 45},
		{BOARD_GPP_PWR_SW, 46},
		{BOARD_GPP_PWRAUTO_SW, 47},
		{BOARD_GPP_FUNC_RED_LED, 37},
		{BOARD_GPP_UART_EN, 48},
	};

MV_BOARD_MPP_TYPE_INFO mvlswvInfoBoardMppTypeInfo[] = 
	/* {{MV_BOARD_MPP_TYPE_CLASS	boardMppGroup1,
 		MV_BOARD_MPP_TYPE_CLASS	boardMppGroup2}} */
	{
		{MV_BOARD_OTHER, MV_BOARD_OTHER}
	//{MV_BOARD_OTHER,MV_BOARD_RGMII}
	}; 

MV_BOARD_MPP_INFO mvlswvInfoBoardMppConfigValue[] = 
	{{{
	MVLSWV_MPP0_7,
	MVLSWV_MPP8_15,
	MVLSWV_MPP16_23,
	MVLSWV_MPP24_31,
	MVLSWV_MPP32_39,
	MVLSWV_MPP40_47,
	MVLSWV_MPP48_55
	}}};

MV_DEV_CS_INFO mvlswvInfoBoardDeCsInfo[] = 
		/*{deviceCS, params, devType, devWidth}*/			   
		 {
			 {0, N_A, BOARD_DEV_NAND_FLASH, 8},	/* NAND DEV */
			 {1, N_A, BOARD_DEV_SPI_FLASH, 8}	/* SPI DEV */
		 };

#define MVLSWV_BOARD_PCI_IF_NUM			0x0
#define MVLSWV_BOARD_TWSI_DEF_NUM		0x0
#define MVLSWV_BOARD_MAC_INFO_NUM		0x1
#define MVLSWV_BOARD_GPP_INFO_NUM		(sizeof(mvlswvInfoBoardGppInfo)/sizeof(MV_BOARD_GPP_INFO))
#define MVLSWV_BOARD_MPP_GROUP_TYPE_NUM		0x0
#define MVLSWV_BOARD_MPP_CONFIG_NUM		0x1
#define MVLSWV_BOARD_DEVICE_CONFIG_NUM		0x2
#define MVLSWV_BOARD_NAND_READ_PARAMS		0x003E07CF
#define MVLSWV_BOARD_NAND_WRITE_PARAMS		0x000F0F0F
#define MVLSWV_BOARD_NAND_CONTROL		0x01c7D943

MV_BOARD_INFO mvlswvInfo = {
	"MVLSWV",				/* boardName[MAX_BOARD_NAME_LEN] */
	MVLSWV_BOARD_MPP_GROUP_TYPE_NUM,	/* numBoardMppTypeValue */
	mvlswvInfoBoardMppTypeInfo,		/* pBoardMppTypeValue */
	MVLSWV_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfigValue */
	mvlswvInfoBoardMppConfigValue,		/* pBoardMppConfigValue */
	0,					/* intsGppMaskLow */
	0,					/* intsGppMaskHigh */
	MVLSWV_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDeviceIf */
	mvlsxhInfoBoardDeCsInfo,		/* pDevCsInfo */
	MVLSWV_BOARD_TWSI_DEF_NUM,		/* numBoardTwsiDev */
	NULL,					/* pBoardTwsiDev */
	MVLSWV_BOARD_MAC_INFO_NUM,		/* numBoardMacInfo */
	mvlswvInfoBoardMacInfo,			/* pBoardMacInfo */
	MVLSWV_BOARD_GPP_INFO_NUM,		/* numBoardGppInfo */
	mvlswvInfoBoardGppInfo,			/* pBoardGppInfo */
	0,					/* activeLedsNumber */              
	NULL,					/* pLedGppPin */
	0,					/* ledsPolarity */		
	MVLSWV_OE_LOW,				/* gppOutEnLow */
	MVLSWV_OE_HIGH,				/* gppOutEnHigh */
	MVLSWV_OE_VAL_LOW,			/* gppOutValLow */
	MVLSWV_OE_VAL_HIGH,			/* gppOutValHigh */
	MVLSWV_POLARITY_VAL_LOW,		/* gppPolarityValLow */
	MVLSWV_POLARITY_VAL_HIGH,		/* gppPolarityValHigh */
	NULL,					/* pSwitchInfo */
	MVLSWV_BOARD_NAND_READ_PARAMS,		/* nandFlashReadParams */
	MVLSWV_BOARD_NAND_WRITE_PARAMS,		/* nandFlashWriteParams */
	MVLSWV_BOARD_NAND_CONTROL,		/* nandFlashControl */
};

/****************************** MVLSQV ******************************/
MV_BOARD_MAC_INFO mvlsqvInfoBoardMacInfo[] = 
	/* {{MV_BOARD_MAC_SPEED	boardMacSpeed, MV_U8 boardEthSmiAddr}} */
	{
		{BOARD_MAC_SPEED_AUTO, 0x0}
	}; 

MV_BOARD_GPP_INFO mvlsqvInfoBoardGppInfo[] = 
	/* {{MV_BOARD_GPP_CLASS	devClass, MV_U8	gppPinNum}} */
	{
		{BOARD_GPP_HDD_POWER,	8},	// HDDPWR_0
		{BOARD_GPP_HDD_POWER,	9},	// HDDPWR_1
		{BOARD_GPP_HDD_POWER,	22},	// HDDPWR_2
		{BOARD_GPP_HDD_POWER,	23},	// HDDPWR_3
		{BOARD_GPP_USB_VBUS_EN,	12},
		{BOARD_GPP_FAN_HIGH,	16},
		{BOARD_GPP_FAN_LOW,	17},
		{BOARD_GPP_HDD_ERR_LED, 34},	// HDD_ERROR_0#
		{BOARD_GPP_HDD_ERR_LED, 35},	// HDD_ERROR_1#
		{BOARD_GPP_HDD_ERR_LED, 24},	// HDD_ERROR_2#
		{BOARD_GPP_HDD_ERR_LED, 25},	// HDD_ERROR_3#
		{BOARD_GPP_FUNC_LED,	39},
		{BOARD_GPP_ALARM_LED,	36},
		{BOARD_GPP_INFO_LED,	38},
		{BOARD_GPP_PWR_LED,	40},
		{BOARD_GPP_FAN_LOCK,	43},
		{BOARD_GPP_FUNC_SW,	45},
		{BOARD_GPP_PWR_SW,	46},
		{BOARD_GPP_PWRAUTO_SW,	47},
		{BOARD_GPP_FUNC_RED_LED,37},
		{BOARD_GPP_UART_EN,	48},
	};

MV_BOARD_MPP_TYPE_INFO mvlsqvInfoBoardMppTypeInfo[] = 
	/* {{MV_BOARD_MPP_TYPE_CLASS	boardMppGroup1,
 		MV_BOARD_MPP_TYPE_CLASS	boardMppGroup2}} */
	{
		{MV_BOARD_OTHER, MV_BOARD_OTHER}
	//{MV_BOARD_OTHER,MV_BOARD_RGMII}
	}; 

MV_BOARD_MPP_INFO	mvlsqvInfoBoardMppConfigValue[] = 
	{{{
		MVLSQV_MPP0_7,		
		MVLSQV_MPP8_15,		
		MVLSQV_MPP16_23,		
		MVLSQV_MPP24_31,		
		MVLSQV_MPP32_39,
		MVLSQV_MPP40_47,
		MVLSQV_MPP48_55
	}}};

MV_DEV_CS_INFO mvlsqvInfoBoardDeCsInfo[] = 
	/*{deviceCS, params, devType, devWidth}*/
	{
		{0, N_A, BOARD_DEV_NAND_FLASH, 8},	/* NAND DEV */
		{1, N_A, BOARD_DEV_SPI_FLASH, 8},	/* SPI DEV */
	};

#define MVLSQV_BOARD_PCI_IF_NUM			0x0
#define MVLSQV_BOARD_TWSI_DEF_NUM		0x0
#define MVLSQV_BOARD_MAC_INFO_NUM		0x1
#define MVLSQV_BOARD_GPP_INFO_NUM		(sizeof(mvlsqvInfoBoardGppInfo)/sizeof(MV_BOARD_GPP_INFO))
#define MVLSQV_BOARD_MPP_GROUP_TYPE_NUM		0x1
#define MVLSQV_BOARD_MPP_CONFIG_NUM		0x1
#define MVLSQV_BOARD_DEVICE_CONFIG_NUM		0x2
#define MVLSQV_BOARD_NAND_READ_PARAMS		0x003E07CF
#define MVLSQV_BOARD_NAND_WRITE_PARAMS		0x000F0F0F
#define MVLSQV_BOARD_NAND_CONTROL		0x01c7D943

MV_BOARD_INFO mvlsqvInfo = {
	"MVLSQV",				/* boardName[MAX_BOARD_NAME_LEN] */
	MVLSQV_BOARD_MPP_GROUP_TYPE_NUM,	/* numBoardMppTypeValue */
	mvlsqvInfoBoardMppTypeInfo,		/* pBoardMppTypeValue */
	MVLSQV_BOARD_MPP_CONFIG_NUM,		/* numBoardMppConfigValue */
	mvlsqvInfoBoardMppConfigValue,		/* pBoardMppConfigValue */
	0,					/* intsGppMaskLow */
	0,					/* intsGppMaskHigh */
	MVLSQV_BOARD_DEVICE_CONFIG_NUM,		/* numBoardDeviceIf */
	mvlsqvInfoBoardDeCsInfo,		/* pDevCsInfo */
	MVLSQV_BOARD_TWSI_DEF_NUM,		/* numBoardTwsiDev */
	NULL,					/* pBoardTwsiDev */
	MVLSQV_BOARD_MAC_INFO_NUM,		/* numBoardMacInfo */
	mvlsqvInfoBoardMacInfo,			/* pBoardMacInfo */
	MVLSQV_BOARD_GPP_INFO_NUM,		/* numBoardGppInfo */
	mvlsqvInfoBoardGppInfo,			/* pBoardGppInfo */
	0,					/* activeLedsNumber */              
	NULL,					/* pLedGppPin */
	0,					/* ledsPolarity */		
	MVLSQV_OE_LOW,				/* gppOutEnLow */
	MVLSQV_OE_HIGH,				/* gppOutEnHigh */
	MVLSQV_OE_VAL_LOW,			/* gppOutValLow */
	MVLSQV_OE_VAL_HIGH,			/* gppOutValHigh */
	MVLSQV_POLARITY_VAL_LOW,		/* gppPolarityValLow */
	MVLSQV_POLARITY_VAL_HIGH,		/* gppPolarityValHigh */
	NULL,					/* pSwitchInfo */
	MVLSQV_BOARD_NAND_READ_PARAMS,		/* nandFlashReadParams */
	MVLSQV_BOARD_NAND_WRITE_PARAMS,		/* nandFlashWriteParams */
	MVLSQV_BOARD_NAND_CONTROL,		/* nandFlashControl */
};

MV_BOARD_INFO* boardInfoTbl[] = {
	&mvlsxhInfo,				/* 0x00: MVLSXH */
	&mvlsvlInfo,				/* 0x01: MVLSVL */
	&mvlswvInfo,				/* 0x02: MVLSWV */
	&mvlsqvInfo,				/* 0x03: MVLSQV */
	&mvlsxlInfo,				/* 0x04: MVLSXL */
	&mvlsxhInfo,				/* 0x05: dummy */
	&mvlsxlGeV2Info,			/* 0x06: MVLSXL-V2 */
	&mvlsxhInfo,				/* 0x07: dummy */
	&mvwxlInfo,				/* 0x08: MVWXL */
	&mvlsxhInfo,				/* 0x09: dummy */
	&mvlsxhInfo,				/* 0x0a: dummy */
	&mvlsxhInfo,				/* 0x0b: dummy */
	&mvlsxhInfo,				/* 0x0c: dummy */
	&mvlsxhInfo,				/* 0x0d: dummy */
	&mvlsxhInfo,				/* 0x0e: dummy */
	&mvlsxhInfo,				/* 0x0f: dummy */
	&mvwssxInfo,				/* 0x10: MVWSSX */
};

#endif // CONFIG_BUFFALO_PLATFORM
