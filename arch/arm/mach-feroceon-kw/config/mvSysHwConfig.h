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
/*******************************************************************************
* mvSysHwCfg.h - Marvell system HW configuration file
*
* DESCRIPTION:
*       None.
*
* DEPENDENCIES:
*       None.
*
*******************************************************************************/

#ifndef __INCmvSysHwConfigh
#define __INCmvSysHwConfigh

#include "../../../../include/linux/autoconf.h"
#include "../../../../arch/arm/mach-feroceon-kw/pss/pssBspApisInline.h"

#define CONFIG_MARVELL	1

/* includes */
#define _1K         0x00000400
#define _4K         0x00001000
#define _8K         0x00002000
#define _16K        0x00004000
#define _32K        0x00008000
#define _64K        0x00010000
#define _128K       0x00020000
#define _256K       0x00040000
#define _512K       0x00080000

#define _1M         0x00100000
#define _2M         0x00200000
#define _4M         0x00400000
#define _8M         0x00800000
#define _16M        0x01000000
#define _32M        0x02000000
#define _64M        0x04000000
#define _128M       0x08000000
#define _256M       0x10000000
#define _512M       0x20000000

#define _1G         0x40000000
#define _2G         0x80000000

/****************************************/
/* Soc supporeted Units definitions	*/
/****************************************/
#ifdef CONFIG_MV_INCLUDE_TWSI
#define MV_INCLUDE_TWSI
#endif
#ifdef CONFIG_MV_INCLUDE_CESA
#define MV_INCLUDE_CESA
#endif
#ifdef CONFIG_MV_INCLUDE_GIG_ETH
#define MV_INCLUDE_GIG_ETH
#endif
#ifdef CONFIG_MV_INCLUDE_INTEG_SATA
#define MV_INCLUDE_INTEG_SATA
#define MV_INCLUDE_SATA
#endif
#ifdef CONFIG_MV_INCLUDE_USB
#define MV_INCLUDE_USB
#define MV_USB_VOLTAGE_FIX
#endif
#ifdef CONFIG_MV_INCLUDE_NAND
#define MV_INCLUDE_NAND
#endif
#ifdef CONFIG_MV_INCLUDE_XOR
#define MV_INCLUDE_XOR
#endif
#ifdef CONFIG_MV_INCLUDE_TWSI
#define MV_INCLUDE_TWSI
#endif
#ifdef CONFIG_MV_INCLUDE_UART
#define MV_INCLUDE_UART
#endif
#ifdef CONFIG_MV_INCLUDE_SPI
#define MV_INCLUDE_SPI
#endif
#ifdef CONFIG_MV_INCLUDE_SFLASH_MTD
#define MV_INCLUDE_SFLASH_MTD
#endif
#ifdef CONFIG_MV_INCLUDE_AUDIO
#define MV_INCLUDE_AUDIO
#endif
#ifdef CONFIG_MV_INCLUDE_TS
#define MV_INCLUDE_TS
#endif
#ifdef CONFIG_MV_INCLUDE_SDIO
#define MV_INCLUDE_SDIO
#endif


/* NAND flash stuff */
#ifdef CONFIG_MV_NAND_BOOT
#define MV_NAND_BOOT
#endif
#ifdef CONFIG_MV_NAND
#define MV_NAND
#endif

/* SPI flash stuff */
#ifdef CONFIG_MV_SPI_BOOT
#define MV_SPI_BOOT
#endif


/****************************************************************/
/************* General    configuration ********************/
/****************************************************************/

/* Enable Clock Power Control */
#define MV_INCLUDE_CLK_PWR_CNTRL

/* Disable the DEVICE BAR in the PEX */
#define MV_DISABLE_PEX_DEVICE_BAR

/* Allow the usage of early printings during initialization */
#define MV_INCLUDE_EARLY_PRINTK

/****************************************************************/
/************* CESA configuration ********************/
/****************************************************************/

#ifdef MV_INCLUDE_CESA

#define MV_CESA_MAX_CHAN               4

/* Use 2K of SRAM */
#define MV_CESA_MAX_BUF_SIZE           1600

#endif /* MV_INCLUDE_CESA */

#if defined(CONFIG_MV_INCLUDE_GIG_ETH)

#ifdef CONFIG_MV_NFP_STATS
#define MV_FP_STATISTICS
#else
#undef MV_FP_STATISTICS
#endif
/* Default configuration for SKB_REUSE: 0 - Disabled, 1 - Enabled */
#define MV_ETH_SKB_REUSE_DEFAULT    1
/* Default configuration for TX_EN workaround: 0 - Disabled, 1 - Enabled */
#define MV_ETH_TX_EN_DEFAULT        0

/* un-comment if you want to perform tx_done from within the poll function */
/* #define ETH_TX_DONE_ISR */

/* put descriptors in uncached memory */
/* #define ETH_DESCR_UNCACHED */
#ifdef MV_PRESTERA_USE_MII_DRIVER
#define ETH_DESCR_UNCACHED
#endif

/* Descriptors location: DRAM/internal-SRAM */
#define ETH_DESCR_IN_SDRAM
#undef  ETH_DESCR_IN_SRAM    /* No integrated SRAM in 88Fxx81 devices */

#if defined(ETH_DESCR_IN_SRAM)
#if defined(ETH_DESCR_UNCACHED)
 #define ETH_DESCR_CONFIG_STR    "Uncached descriptors in integrated SRAM"
#else
 #define ETH_DESCR_CONFIG_STR    "Cached descriptors in integrated SRAM"
#endif
#elif defined(ETH_DESCR_IN_SDRAM)
#if defined(ETH_DESCR_UNCACHED)
 #define ETH_DESCR_CONFIG_STR    "Uncached descriptors in DRAM"
#else
 #define ETH_DESCR_CONFIG_STR    "Cached descriptors in DRAM"
#endif
#else 
 #error "Ethernet descriptors location undefined"
#endif /* ETH_DESCR_IN_SRAM or ETH_DESCR_IN_SDRAM*/

/* SW Sync-Barrier: not relevant for 88fxx81*/
/* Reasnable to define this macro when descriptors in SRAM and buffers in DRAM */
/* In RX the CPU theoretically might see himself as the descriptor owner,      */
/* although the buffer hadn't been written to DRAM yet. Performance cost.      */
/* #define INCLUDE_SYNC_BARR */

/* Buffers cache coherency method (buffers in DRAM) */
#ifndef MV_CACHE_COHER_SW
/* Taken from mvCommon.h */
/* Memory uncached, HW or SW cache coherency is not needed */
#define MV_UNCACHED             0   
/* Memory cached, HW cache coherency supported in WriteThrough mode */
#define MV_CACHE_COHER_HW_WT    1
/* Memory cached, HW cache coherency supported in WriteBack mode */
#define MV_CACHE_COHER_HW_WB    2
/* Memory cached, No HW cache coherency, Cache coherency must be in SW */
#define MV_CACHE_COHER_SW       3

#endif

/* DRAM cache coherency configuration */
#define MV_CACHE_COHERENCY  MV_CACHE_COHER_SW


#define ETHER_DRAM_COHER    MV_CACHE_COHER_SW   /* No HW coherency in 88Fxx81 devices */

#if (ETHER_DRAM_COHER == MV_CACHE_COHER_HW_WB)
 #define ETH_SDRAM_CONFIG_STR    "DRAM HW cache coherency (write-back)"
#elif (ETHER_DRAM_COHER == MV_CACHE_COHER_HW_WT)
 #define ETH_SDRAM_CONFIG_STR    "DRAM HW cache coherency (write-through)"
#elif (ETHER_DRAM_COHER == MV_CACHE_COHER_SW)
 #define ETH_SDRAM_CONFIG_STR    "DRAM SW cache-coherency"
#elif (ETHER_DRAM_COHER == MV_UNCACHED)
#   define ETH_SDRAM_CONFIG_STR  "DRAM uncached"
#else
 #error "Ethernet-DRAM undefined"
#endif /* ETHER_DRAM_COHER */


/****************************************************************/
/************* Ethernet driver configuration ********************/
/****************************************************************/

/* interrupt coalescing setting */
#define ETH_TX_COAL    		    200
#define ETH_RX_COAL    		    200

/* Checksum offloading */
#define TX_CSUM_OFFLOAD
#define RX_CSUM_OFFLOAD

#endif /* CONFIG_MV_INCLUDE_GIG_ETH */

/* We use the following registers to store DRAM interface pre configuration   */
/* auto-detection results													  */
/* IMPORTANT: We are using mask register for that purpose. Before writing     */
/* to units mask register, make sure main maks register is set to disable     */
/* all interrupts.                                                            */
#define DRAM_BUF_REG0   0x30810 /* sdram bank 0 size            */  
#define DRAM_BUF_REG1   0x30820 /* sdram config                 */
#define DRAM_BUF_REG2   0x30830 /* sdram mode                   */
#define DRAM_BUF_REG3   0x308c4 /* dunit control low            */          
#define DRAM_BUF_REG4   0x60a90 /* sdram address control        */
#define DRAM_BUF_REG5   0x60a94 /* sdram timing control low     */
#define DRAM_BUF_REG6   0x60a98 /* sdram timing control high    */
#define DRAM_BUF_REG7   0x60a9c /* sdram ODT control low        */
#define DRAM_BUF_REG8   0x60b90 /* sdram ODT control high       */
#define DRAM_BUF_REG9   0x60b94 /* sdram Dunit ODT control      */
#define DRAM_BUF_REG10  0x60b98 /* sdram Extended Mode          */
#define DRAM_BUF_REG11  0x60b9c /* sdram Ddr2 Time Low Reg      */
#define DRAM_BUF_REG12  0x60a00 /* sdram Ddr2 Time High Reg     */
#define DRAM_BUF_REG13  0x60a04 /* dunit Ctrl High              */
#define DRAM_BUF_REG14  0x60b00 /* sdram second DIMM exist      */

/* Following the pre-configuration registers default values restored after    */
/* auto-detection is done                                                     */
#define DRAM_BUF_REG_DV 0

/* System Mapping */
#define SDRAM_CS0_BASE  0x00000000
#define SDRAM_CS0_SIZE  _256M

#define SDRAM_CS1_BASE  0x10000000
#define SDRAM_CS1_SIZE  _256M

#define SDRAM_CS2_BASE  0x20000000
#define SDRAM_CS2_SIZE  _256M

#define SDRAM_CS3_BASE  0x30000000
#define SDRAM_CS3_SIZE  _256M

#define ETH_NUM_OF_RX_DESCR     128
//#define ETH_NUM_OF_TX_DESCR     ETH_NUM_OF_RX_DESCR*2

#define MV_CESA_MAX_BUF_SIZE	1600

/*
 * Default MRU and RGMII-1 buffer size
 * (needed for Scatter-Gather support)
 * (1536 + 2 + 8 + 4) explanation:
 * +2 : 2 bytes prepended by RGMII-1
 * +8 : extended DSA tag
 * +4 : CRC
 */
#define MII_RX_BUFF_SIZE_DEFAULT   (1536 + 2 + 8 + 4)
#define MII_RX_BUFF_SIZE_MIN       (8    + 2 + 8 + 4)
#define MII_RX_BUFF_SIZE_MAX       (_64KB - _1KB)

#define MII_MRU_DEFAULT            (1536 + 2 + 8 + 4)
#define MII_MRU_MIN                (1518 + 2 + 8 + 4)
#define MII_MRU_MAX                (9700 + 2 + 8 + 4)

#define SWITCH_MTU                  1528

/*
 * Generic Network Driver (GND)
 */
#define MV_GND_POLL_COUNT_MAX           50
#define MV_GND_POLL_TIMOUT_MSEC         10

/*
 * Scatter-Gather support
 */
/* #define ETH_RX_SCATTER_GATHER */
#define ETH_TX_SCATTER_GATHER

#ifdef ETH_RX_SCATTER_GATHER
    #define MII_RX_BUFF_SIZE        (1536 + 2 + 8 + 4)
    #define MII_MRU                 (9000 + 2 + 8 + 4)
#else
    #define MII_RX_BUFF_SIZE        MII_RX_BUFF_SIZE_DEFAULT
    #define MII_MRU                 MII_MRU_DEFAULT
#endif

/* port's default queueus */
#define ETH_DEF_TXQ         0
#define ETH_DEF_RXQ         0

#define MII_DEF_TXQ         0
#define MII_DEF_RXQ         0

#define MII_TX_Q_NUM        8
#define MV_ETH_TX_Q_NUM     8

#define MV_INCLUDE_SAGE

/* PEX */
#define PEX0_MEM_BASE 0xE8000000
#define PEX0_MEM_SIZE _128M

#define PEX0_IO_BASE (0xFC000000)
#define PEX0_IO_SIZE _1M

#define PRESTERA_DEV0_BASE     0xF4000000
#define PRESTERA_DEV1_BASE     0xE8000000


#ifdef MV_INCLUDE_SAGE
#define SAGE_UNIT_BASE  0xF4000000
#define SAGE_UNIT_SIZE  _64M
#endif

/* Device Chip Selects */
#define NFLASH_CS_BASE 0xd8000000
#define NFLASH_CS_SIZE _128M

#define SPI_CS_BASE 0xF8000000
#define SPI_CS_SIZE _16M

#define CRYPT_ENG_BASE	0xfB000000
#define CRYPT_ENG_SIZE	_64K

#define BOOTDEV_CS_BASE	0xff800000
#define BOOTDEV_CS_SIZE _8M

#define PP_DEV0_BASE            0xf4000000
#define PP_DEV1_BASE            PEX0_MEM_BASE + _64M
#define PP_DEV0_SIZE            _64M
#define PP_DEV1_SIZE            _64M

/* CS2 - BOOTROM */
#define DEVICE_CS2_BASE 0xff900000
#define DEVICE_CS2_SIZE _1M

/* PEX Work arround */
/* the target we will use for the workarround */
#define PEX_CONFIG_RW_WA_TARGET PEX0_MEM
/*a flag that indicates if we are going to use the 
size and base of the target we using for the workarround
window */
#define PEX_CONFIG_RW_WA_USE_ORIGINAL_WIN_VALUES 1
/* if the above flag is 0 then the following values
will be used for the workarround window base and size,
otherwise the following defines will be ignored */
#define PEX_CONFIG_RW_WA_BASE 0xF3000000
#define PEX_CONFIG_RW_WA_SIZE _16M

/* Internal registers: size is defined in Controllerenvironment */
#define INTER_REGS_BASE	0xF1000000

/* DRAM detection stuff */
#define MV_DRAM_AUTO_SIZE

/* Board clock detection */
#define TCLK_AUTO_DETECT    /* Use Tclk auto detection   */
#define SYSCLK_AUTO_DETECT	/* Use SysClk auto detection */
#define PCLCK_AUTO_DETECT  	/* Use PClk auto detection   */
#define L2CLK_AUTO_DETECT 	/* Use L2Clk auto detection   */

/* PEX-PCI\PCI-PCI Bridge*/
#define PCI0_IF_PTP		0		/* Bridge exist on pciIf0*/

#define INT_LVL_SWITCH_GPP_INT      (INT_LVL_PCI + SWITCH_INT_BIT)
/* PEX interrupt levels */
#define INT_LVL_PEX0_INTA		65
#define INT_LVL_PEX0_INTB		66
#define INT_LVL_PEX0_INTC		67
#define INT_LVL_PEX0_INTD	    68

#define INT_LVL_PCI             69

/* e.g. PEX1 int C = 76  |  PEX0 int C = 67 */
#define INT_LVL_PEX(pexIf, pin)	    (INT_LVL_PEX0_INTA - 1 + pin)

#define MV_ETH_RX_Q_NUM             8
#define MII_RX_Q_NUM                8
#define ETH_NUM_OF_RX_DESCR         128
#define MII_NUM_OF_RX_DESCR         128
#define ETH_NUM_OF_TX_DESCR         (ETH_NUM_OF_RX_DESCR*MV_ETH_RX_Q_NUM + MV_ETH_EXTRA_TX_DESCR)
#define MII_NUM_OF_TX_DESCR         (MII_NUM_OF_RX_DESCR*MII_RX_Q_NUM + MII_EXTRA_TX_DESCR)
#define ETH_RX_QUEUE_QUOTA          32   /* quota per Rx Q */

/*
 * Switch uses GPP interrupt number 49,
 * which is interrupt 17 of group two of GPP interrupts
 */
#define SWITCH_INT_BIT          49 /* should not be used independently */
#define INT_LVL_SWITCH_GPP_INT      (INT_LVL_PCI + SWITCH_INT_BIT)

#define INT_LVL_GBE_PORT_RX(port)   (INT_LVL_GBE0_RX + (4 * (port)))
#define INT_LVL_GBE_PORT_TX(port)   (INT_LVL_GBE0_TX + (4 * (port)))
#define INT_LVL_GBE_PORT_MISC(port) (INT_LVL_GBE0_MISC+ (4 * (port)))

/*
 * Generic Network Driver (GND) configurations.
 */
#define MV_GND_POLL_COUNT_MAX_DEFAULT     500
#define MV_GND_POLL_TIMOUT_USEC_DEFAULT   10
#define MV_GND_BUFF_ALIGN_DEFAULT         8
#define MV_GND_BUFF_MIN_SIZE_DEFAULT      8

/*
 * Generic Network Driver (GND) configurations.
 */
#define PP_DRV_POLL_COUNT_MAX             50
#define PP_DRV_POLL_TIMOUT_MSEC           10
#define PP_DRV_MAX_BUF_PER_PKT            10
#define PP_DRV_DEF_RX_Q                   MII_DEF_RXQ
#define PP_DRV_DEF_TX_Q                   MII_DEF_TXQ
#define PP_DRV_RX_BUFF_SIZE_DEFAULT       MII_RX_BUFF_SIZE_DEFAULT
#define PP_DRV_MRU_DEFAULT                MII_MRU_DEFAULT

#define PRESTERA_CACHED

#define NUM_OF_RXTX_QUEUES                (8)

#define PP_NUM_OF_RX_DESC_PER_Q           (PRESTERA_RXQ_LEN)
#define PP_NUM_OF_RX_DESC_TOTAL           (PRESTERA_RXQ_LEN * NUM_OF_RX_QUEUES)
#define PP_NUM_OF_TX_DESC_PER_Q           (PRESTERA_TXQ_LEN)
#define PP_NUM_OF_TX_DESC_TOTAL           (PRESTERA_TXQ_LEN * NUM_OF_TX_QUEUES)

#define MV_GND_POLL_TIMOUT_USEC_DEFAULT   10

 /*
 * Main Interrupt Cause Register bits
 */
#define INT_LVL_HIGHSUM  	   	0	/* Summary of Main Interrupt High Cause register*/
#define INT_LVL_BRIDGE         	1	/* Mbus-L to Mbus Bridge interrupt 	*/
#define INT_LVL_DB_IN 		    2	/* Summary of Inbound Doorbell Cause register  	*/
#define INT_LVL_DB_OUT		    3	/* Summary of Outbound Doorbell Cause register 	*/
#define INT_LVL_x4   		    4	/* 4 reserved	*/
#define INT_LVL_XOR0_CHAN0   	5	/* Xor 0 Channel0 interrupt */
#define INT_LVL_XOR0_CHAN1   	6	/* Xor 0 Channel1 interrupt */
#define INT_LVL_XOR1_CHAN0   	7	/* Xor 1 Channel0 interrupt */
#define INT_LVL_XOR1_CHAN1   	8	/* Xor 1 Channel1 interrupt */
#define INT_LVL_PEX00_ABCD	    9	/* PCI Express port0.0 INTA/B/C/D assert message interrupt. */
#define INT_LVL_x10   		    10	/* 10 reserved	*/
#define INT_LVL_GBE0_SUM   	    11	/* Gigabit Ethernet Port 0.0 summary	 	*/
#define INT_LVL_GBE0_RX		    12	/* Gigabit Ethernet Port 0.0 Rx summary    	*/
#define INT_LVL_GBE0_TX         13	/* Gigabit Ethernet Port 0.0 Tx summary    	*/
#define INT_LVL_GBE0_MISC       14	/* Gigabit Ethernet Port 0.0 misc. summary 	*/
#define INT_LVL_GBE1_SUM   	    15	/* Gigabit Ethernet Port 0.1 summary	 	*/
#define INT_LVL_GBE1_RX		    16	/* Gigabit Ethernet Port 0.1 Rx summary    	*/
#define INT_LVL_GBE1_TX         17	/* Gigabit Ethernet Port 0.1 Tx summary    	*/
#define INT_LVL_GBE1_MISC       18	/* Gigabit Ethernet Port 0.1 misc. summary 	*/
#define INT_LVL_USB0   	        19	/* USB0 interrupt   */
#define INT_LVL_x20   		    20	/* 20 reserved	*/
#define INT_LVL_SATA_CTRL  	    21	/* SATA controller interrupt     	*/
#define INT_LVL_CESA	   	    22	/* Security engine completion interrupt	*/
#define INT_LVL_SPI	   			23	/* SPI interrupt	*/
#define INT_LVL_AUDIO	   		24	/* Audio interrupt	*/
#define INT_LVL_x25   		    25	/* 25 reserved	*/
#define INT_LVL_TS0	   			26	/* TS0 interrupt	*/
#define INT_LVL_x27   		    27	/* 27 reserved	*/
#define INT_LVL_DSIO	   		28	/* SDIO interrupt	*/
#define INT_LVL_TWSI  			29	/* TWSI interrupt  */
#define INT_LVL_AVB  			30	/* AVB interrupt  */
#define INT_LVL_TDM  			31	/* TDM interrupt  */
#define INT_LVL_x32   		    32	/* 32 reserved	*/
#define INT_LVL_UART0   	    33 	/* UART0 interrupt  */
#define INT_LVL_UART1   	    34 	/* UART1 interrupt  */
#define INT_LVL_GPIOLO0_7	    35	/* GPIO LOW [00:07] Interrupt */
#define INT_LVL_GPIOLO8_15	    36	/* GPIO LOW [O8:15] Interrupt */
#define INT_LVL_GPIOLO16_23     37	/* GPIO LOW [16:23] Interrupt */
#define INT_LVL_GPIOLO24_31     38	/* GPIO LOW [24:31] Interrupt */
#define INT_LVL_GPIOHI0_7	    39	/* GPIO HIGH [00:07] Interrupt */
#define INT_LVL_GPIOHI8_15	    40	/* GPIO HIGH [O8:15] Interrupt */
#define INT_LVL_GPIOHI16_23     41	/* GPIO HIGH [16:23] Interrupt */
#define INT_LVL_XOR0_ERR	    42	/* XOR0 engine error 	 				 */
#define INT_LVL_XOR1_ERR	    43	/* XOR1 engine error 	 				 */
#define INT_LVL_PEX0_ERR	    44	/* PEX0 engine error 	 				 */
#define INT_LVL_x45   		    45	/* 45 reserved	*/
#define INT_LVL_GBE0_ERR	    46	/* GbE port0 error */
#define INT_LVL_GBE1_ERR	    47	/* GbE port1 error */
#define INT_LVL_USB_ERR	        48	/* USB error (address decoding) Summary of errors from all USB ports */
#define INT_LVL_SECURITY_ERR	49  /* Security engine completion error	*/
#define INT_LVL_AUDIO_ERR	   	50	/* Audio error	*/
#define INT_LVL_x51   		    51	/* 51 reserved	*/
#define INT_LVL_x52   		    52	/* 52 reserved	*/
#define INT_LVL_RTC   		    53	/* RTC interrupt */
#define INT_LVL_DRAGONITE       54	/* Dragonite interrupt */
#define INT_LVL_x55             55  /* 55-63 reserved */
#define INT_LVL_XCAT2_SWITCH    INT_LVL_x55 /* Switch MG interrupt */

#define DRAGONITE_DTCM_BASE 0xF2000000
/*
 * Actually DTCM size is 32K, but minimal hardware resolution is 64K
 */
#define DRAGONITE_DTCM_SIZE _64K

/*
 * FIQ interrupts support.
 *     Note: These defines enables to route the interrupts of both timers
 *     to chip FIQ. Uncomment to use it.
 */
/* #define MV_SYS_TIMER_THROUGH_GPIO */
#define MV_SYS_TIMER_THROUGH_GPIO_NUM    35

#define MV_PP_HAL_WRITE_LOG_ENABLE
#define MV_DEV_WRITE_LOG_ENABLE
#define MV_DEV_WRITE_LOG_EARLY

#define MV_WRITE_LOG_NUM_OF_INSTANCE      2
#define MV_WRITE_LOG_INSTANCE_LEN         1000
#define MV_WRITE_LOG_LEN_TOTAL            MV_WRITE_LOG_INSTANCE_LEN * MV_WRITE_LOG_NUM_OF_INSTANCE

#define MV_NET_NUM_OF_RX_Q                (8)
#define MV_NET_NUM_OF_TX_Q                (8)
#define MV_NET_NUM_OF_RX_DESC_PER_Q       (128)
#define MV_NET_NUM_OF_TX_DESC_PER_Q       (128)

#define MV_NET_NUM_OF_RX_DESC_TOTAL (MV_NET_NUM_OF_RX_Q*MV_NET_NUM_OF_RX_DESC_PER_Q)
#define MV_NET_NUM_OF_TX_DESC_TOTAL (MV_NET_NUM_OF_TX_Q*MV_NET_NUM_OF_TX_DESC_PER_Q)

#define NUM_OF_RX_QUEUES                  MV_NET_NUM_OF_RX_Q
#define NUM_OF_TX_QUEUES                  MV_NET_NUM_OF_TX_Q

/* rings length */
#define PRESTERA_RXQ_LEN                  MV_NET_NUM_OF_RX_DESC_PER_Q
#define PRESTERA_TXQ_LEN                  MV_NET_NUM_OF_TX_DESC_PER_Q

#define PP_NUM_OF_RX_DESC_PER_Q           (PRESTERA_RXQ_LEN)
#define PP_NUM_OF_RX_DESC_TOTAL           (PRESTERA_RXQ_LEN * NUM_OF_RX_QUEUES)
#define PP_NUM_OF_TX_DESC_PER_Q           (PRESTERA_TXQ_LEN)
#define PP_NUM_OF_TX_DESC_TOTAL           (PRESTERA_TXQ_LEN * NUM_OF_TX_QUEUES)

#endif /* __INCmvSysHwConfigh */

