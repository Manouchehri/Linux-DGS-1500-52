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

/*******************************************************************************
* pssBspApis.c - bsp APIs
*
* DESCRIPTION:
*       API's supported by BSP.
*
* DEPENDENCIES:
*       None.
*
* FILE REVISION NUMBER:
*       $Revision: 1.1.1.1.2.2 $
*
*******************************************************************************/

#include "mvTypes.h"
#include "mvXor.h"
#include "pssBspApis.h"
#include "mvBoardEnvSpec.h"
#include "mvDragonite.h"
#include "mvCtrlEnvLib.h"
#include "mvSysGbe.h"
#include "mii.h"
#include "mvEthPhy.h"
#include "mvSysHwConfig.h"
#include "bitOps.h"
#include "mvGndReg.h"
#include "mvGnd.h"
#include <linux/netdevice.h>
#include <linux/pci.h>

/* MG - Address Decoding registers */
#define PRESTERA_AD_BASE_ADDR(index) (0x30C + (index*8))
#define PRESTERA_AD_SIZE(index)      (0x310 + (index*8))
#define PRESTERA_AD_BARE              0x34C                /* Base address enable */

#define XCAT_INTERNAL_PP_BASE_ADDR 0xF4000000
MV_U32  XCAT_INTERNAL_PP_BASE_ADDR_extern = XCAT_INTERNAL_PP_BASE_ADDR;
#define PRESTERA_AD_BARE_BITS_SET(value, index)    value &= ~(1<<index)
#define PRESTERA_AD_BASE_ADDR_BITS_SET(base, attr, target) ((base & 0xFFFF0000) | (attr << 8) | target)
#define PRESTERA_AD_DRAM_TARGET_ID 0

static MV_U8 decodingTargetAttribures[] = {0xE, 0xD, 0xB, 0x7};

typedef struct
{
    MV_U32 data;
    MV_U32 mask;
} PEX_HEADER_DATA;

PEX_HEADER_DATA pp_configHdr[16] =
{
        {0x000011ab, 0x00000000}, /* 0x00 */
        {0x00100006, 0x00000000}, /* 0x04 */
        {0x05800000, 0x00000000}, /* 0x08 */
        {0x00000008, 0x00000000}, /* 0x0C */
        {0xF000000c, 0x00000000}, /* 0x10 */
        {0x00000000, 0x00000000}, /* 0x14 */
        {XCAT_INTERNAL_PP_BASE_ADDR | 0xC, 0x00000000}, /* 0x18 */
        {0x00000000, 0x00000000}, /* 0x1C */
        {0x00000000, 0x00000000}, /* 0x20 */
        {0x00000000, 0x00000000}, /* 0x24 */
        {0x00000000, 0x00000000}, /* 0x28 */
        {0x11ab11ab, 0x00000000}, /* 0x2C */
        {0x00000000, 0x00000000}, /* 0x30 */
        {0x00000040, 0x00000000}, /* 0x34 */
        {0x00000000, 0x00000000}, /* 0x38 */
        {0x00000100, 0x00000000}  /* 0x3C */
};

#define HEADER_WRITE(data, offset) pp_configHdr[offset/4].data = ((pp_configHdr[offset/4].data & ~pp_configHdr[offset/4].mask) | (data & pp_configHdr[offset/4].mask))
#define HEADER_READ(offset) pp_configHdr[offset/4].data
#define DB_XCAT_INTERNAL_PP_INT_GPP_PIN    49

#define STUB_FAIL printk("stub function %s returning MV_NOT_SUPPORTED\n", \
                         __FUNCTION__);  return MV_NOT_SUPPORTED

/* register that hold the device type*/
#define PRESTERA_DEV_ID_REG_OFFSET 0x4C
int PRESTERA_DEV_ID_REG_OFFSET_extern = PRESTERA_DEV_ID_REG_OFFSET;

MV_U32 bspReadRegisterInternal(MV_U32 address);
static MV_VOIDFUNCPTR   bspIsrRoutine   = NULL;
static MV_U32           bspIsrParameter = 0;
static MV_U16 gPPDevId = 0xFFFF;
static MV_U16 gtPPrevision = 0;

extern spinlock_t mii_rx_lock;

void* cpssTxCallback = NULL;
void* cpssRxCallback = NULL;
extern char pssBspApiPortMode[50];


/*Interrupt functions:*/
int (*pss_rxReadyIsrF)(void*);
int (*pss_txDoneIsrF)(void*);

irqreturn_t pss_miiTxDoneIsr(int irq , void *dev_id)
{
    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
	pss_txDoneIsrF(NULL);
  	return IRQ_HANDLED;
}

irqreturn_t pss_miiRxReadyIsr(int irq , void *dev_id)
{
    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
	pss_rxReadyIsrF(NULL);
  	return IRQ_HANDLED;
}

/*******************************************************************************
* bspReset
*
* DESCRIPTION:
*       This routine calls to reset of CPU.
*
* INPUTS:
*       none.
*
* OUTPUTS:
*       none.
*
* RETURNS:
*       MV_OK      - on success.
*       MV_FAIL    - otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS bspReset(MV_VOID)
{
	kernel_restart(NULL);
    	return  MV_OK;
}

/*** DMA ***/
/*******************************************************************************
* bspDmaRead
*
* DESCRIPTION:
*       Read a memory block from a given address.
*
* INPUTS:
*       address     - The address to read from.
*       length      - Length of the memory block to read (in words).
*       burstLimit  - Number of words to be read on each burst.
*
* OUTPUTS:
*       buffer  - The read data.
*
* RETURNS:
*       MV_OK   - on success,
*       MV_FAIL - othersise.
*
* COMMENTS:
*       1.  The given buffer is allways 4 bytes aligned, any further allignment
*           requirements should be handled internally by this function.
*       2.  The given buffer may be allocated from an uncached memory space, and
*           it's to the function to handle the cache flushing.
*       3.  The Prestera Driver assumes that the implementation of the DMA is
*           blocking, otherwise the Driver functionality might be damaged.
*
*******************************************************************************/
MV_STATUS bspDmaRead(IN  MV_U32   address,
                     IN  MV_U32   length,
                     IN  MV_U32   burstLimit,
                     OUT MV_U32  *buffer)
{
        STUB_FAIL;
}

/*** PCI ***/
/*******************************************************************************
 * bspPciConfigWriteReg
 *
 * DESCRIPTION:
 *       This routine write register to the PCI configuration space.
 *
 * INPUTS:
 *       busNo    - PCI bus number.
 *       devSel   - the device devSel.
 *       funcNo   - function number.
 *       regAddr  - Register offset in the configuration space.
 *       data     - data to write.
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK   - on success,
 *       MV_FAIL - othersise.
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspPciConfigWriteReg
(
 IN  MV_U32  busNo,
 IN  MV_U32  devSel,
 IN  MV_U32  funcNo,
 IN  MV_U32  regAddr,
 IN  MV_U32  data
 )
{
  struct pci_dev *dev;

  /* Emulate internal PP on PEX */
  if((0xFF == busNo) && (0xFF == devSel))
  {
    HEADER_WRITE(data, regAddr);
    return MV_OK;
  }
  /* Emulate end*/

  dev = pci_get_bus_and_slot(busNo, PCI_DEVFN(devSel,funcNo));

  if (dev)
  {
    pci_write_config_dword(dev, regAddr, data);
    return MV_OK;
  }
  else
    return MV_FAIL;
}

/*******************************************************************************
 * bspPciConfigReadReg
 *
 * DESCRIPTION:
 *       This routine read register from the PCI configuration space.
 *
 * INPUTS:
 *       busNo    - PCI bus number.
 *       devSel   - the device devSel.
 *       funcNo   - function number.
 *       regAddr  - Register offset in the configuration space.
 *
 * OUTPUTS:
 *       data     - the read data.
 *
 * RETURNS:
 *       MV_OK   - on success,
 *       MV_FAIL - othersise.
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspPciConfigReadReg
(
 IN  MV_U32  busNo,
 IN  MV_U32  devSel,
 IN  MV_U32  funcNo,
 IN  MV_U32  regAddr,
 OUT MV_U32  *data
 )
{
  struct pci_dev *dev;

  /* Emulate internal PP on PEX */
  if((0xFF == busNo) && (0xFF == devSel))
  {
    if ( ((regAddr & 0x00000003) != 0) || regAddr > 0x40)
      return MV_FAIL;

    if(0 == regAddr)
    {
      *data = 0x000011ab | (gPPDevId << 16);
    }
    else if (8 == regAddr)
    {
      *data = HEADER_READ(regAddr) | gtPPrevision;
    }
    else if( regAddr < 4*16 )
    {
      *data = HEADER_READ(regAddr);
    }
    else
    {
    	return MV_FAIL;
    }

    return MV_OK;
  }
  /* Emulate end*/

  dev = pci_get_bus_and_slot(busNo, PCI_DEVFN(devSel,funcNo));

  if (dev)
  {
    pci_read_config_dword(dev, regAddr, data);
    return MV_OK;
  }
  else
    return MV_FAIL;
}

/*******************************************************************************
 * bspReadRegisterInternal
 *
 * DESCRIPTION:
 *       This routine read register from given address.
 *
 * INPUTS:
 *       address  - Register address.
 *
 * OUTPUTS:
 *
 * RETURNS:
 *       data     - the read data.
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_U32 bspReadRegisterInternal
(
    IN MV_U32  address
)
{
  /* Endianess. */
#if defined(MV_CPU_BE)
  /* need to swap the bytes */
  MV_U8   *bytesPtr;
  MV_U32  registerValue = *((MV_U32*)address);

  bytesPtr = (MV_U8*)&registerValue;


  return ((MV_U32)(bytesPtr[3] << 24)) |
         ((MV_U32)(bytesPtr[2] << 16)) |
         ((MV_U32)(bytesPtr[1] << 8 )) |
         ((MV_U32)(bytesPtr[0]      )) ;
#else

  /* direct access - no swap needed */
  return *((MV_U32*)address);
#endif  /* MV_CPU_BE */ 
}

/*******************************************************************************
 * bspPciFindDev
 *
 * DESCRIPTION:
 *       This routine returns the next instance of the given device (defined by
 *       vendorId & devId).
 *
 * INPUTS:
 *       vendorId - The device vendor Id.
 *       devId    - The device Id.
 *       instance - The requested device instance.
 *
 * OUTPUTS:
 *       busNo    - PCI bus number.
 *       devSel   - the device devSel.
 *       funcNo   - function number.
 *
 * RETURNS:
 *       MV_OK   - on success,
 *       MV_FAIL - othersise.
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspPciFindDev
(
 IN  MV_U16  vendorId,
 IN  MV_U16  devId,
 IN  MV_U32  instance,
 OUT MV_U32  *busNo,
 OUT MV_U32  *devSel,
 OUT MV_U32  *funcNo
 )
{
  struct pci_dev *dev = NULL;
  int instanceExpectedinPex = (mvPpChipIsXCat2())?(1):(0);

  MV_U32 regValue;

  /* Emulate internal PP on PEX */
  /* If internal PP not found - read device ID from PP internal register space */

  if(0xFFFF == gPPDevId)
  {

    regValue = bspReadRegisterInternal(PRESTERA_DEV_ID_REG_OFFSET + XCAT_INTERNAL_PP_BASE_ADDR);
    gPPDevId = (MV_U16)((regValue >> 4) & 0xFFFF);
    gtPPrevision = (MV_U16)(regValue & 0xF);

    
    if(gPPDevId == devId)
    {
      *busNo  = 0xFF;
      *devSel = 0xFF;
      *funcNo = 0xFF;
      return MV_OK;
    }
    else
    {
      gPPDevId = 0xFFFF;
    }    
  }
  /* Emulate - end */

  *busNo = *devSel = *funcNo = 0;

  while ((dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL) 
  {
    if ((dev->vendor == vendorId) && 
        (dev->device == devId))
    {
      if (instance == instanceExpectedinPex)
      {
        *busNo = dev->bus->number;
        *devSel = PCI_SLOT(dev->devfn);
        *funcNo = PCI_FUNC(dev->devfn);
        return MV_OK;
      }
    }
  }

  return MV_FAIL;
}

/*******************************************************************************
 * bspPciFindDevReset
 *
 * DESCRIPTION:
 *       Reset gPPDevId to make chance to find internal PP again
 *
 * INPUTS:
 *       None
 *
 * OUTPUTS:
 *       None
 *
 * RETURNS:
 *       MV_OK   - on success,
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspPciFindDevReset(void)
{
  gPPDevId = 0xFFFF;
  gtPPrevision = 0;
 
 return MV_OK;
}

/*******************************************************************************
 * bspPciGetIntVec
 *
 * DESCRIPTION:
 *       This routine return the PCI interrupt vector.
 *
 * INPUTS:
 *       pciInt - PCI interrupt number.
 *
 * OUTPUTS:
 *       intVec - PCI interrupt vector.
 *
 * RETURNS:
 *       MV_OK      - on success.
 *       MV_FAIL    - otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS bspPciGetIntVec
(
 IN  bspPciInt_PCI_INT  pciInt,
 OUT void               **intVec
 )
{
  /* check parameters */
  if(intVec == NULL)
  {
    return MV_BAD_PARAM;
  }
  /* get the PCI interrupt vector */
  if ((pciInt == bspPciInt_PCI_INT_B) || (pciInt == bspPciInt_PCI_INT_D))
  {
        if (mvPpChipIsXCat2() == MV_FALSE)
        {
            /* The internal PP interrupt is connected to GPIO 49 of the CPU */
    		*intVec = (void *)IRQ_GPP_49;
        }
        else
        {
            /* In xCat2 PP int is connected to bit 23 of High Cause (23+32=55)*/
            *intVec = (void *)INT_LVL_XCAT2_SWITCH;
        }
  }
  else
  {
    /* The external PP interrupt is PEX0INT of the CPU */
    *intVec = (void *)IRQ_PEX0_INT;
  }
  
  return MV_OK;
}

/*******************************************************************************
 * bspPciGetIntMask
 *
 * DESCRIPTION:
 *       This routine return the PCI interrupt vector.
 *
 * INPUTS:
 *       pciInt - PCI interrupt number.
 *
 * OUTPUTS:
 *       intMask - PCI interrupt mask.
 *
 * RETURNS:
 *       MV_OK      - on success.
 *       MV_FAIL    - otherwise.
 *
 * COMMENTS:
 *       PCI interrupt mask should be used for interrupt disable/enable.
 *
 *******************************************************************************/
MV_STATUS bspPciGetIntMask
(
 IN  bspPciInt_PCI_INT  pciInt,
 OUT MV_U32             *intMask
 )
{
  STUB_FAIL;
}

/*******************************************************************************
 * bspPciEnableCombinedAccess
 *
 * DESCRIPTION:
 *       This function enables / disables the Pci writes / reads combining
 *       feature.
 *       Some system controllers support combining memory writes / reads. When a
 *       long burst write / read is required and combining is enabled, the master
 *       combines consecutive write / read transactions, if possible, and
 *       performs one burst on the Pci instead of two. (see comments)
 *
 * INPUTS:
 *       enWrCombine - MV_TRUE enables write requests combining.
 *       enRdCombine - MV_TRUE enables read requests combining.
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK               - on sucess,
 *       MV_NOT_SUPPORTED    - if the controller does not support this feature,
 *       MV_FAIL             - otherwise.
 *
 * COMMENTS:
 *       1.  Example for combined write scenario:
 *           The controller is required to write a 32-bit data to address 0x8000,
 *           while this transaction is still in progress, a request for a write
 *           operation to address 0x8004 arrives, in this case the two writes are
 *           combined into a single burst of 8-bytes.
 *
 *******************************************************************************/
MV_STATUS bspPciEnableCombinedAccess
(
 IN  MV_BOOL     enWrCombine,
 IN  MV_BOOL     enRdCombine
 )
{
  STUB_FAIL;
}


#ifdef MV_INCLUDE_DRAGONITE
/*******************************************************************************
* bspDragoniteGetIntVec
*
* DESCRIPTION:
*       This routine returns the Dragonite interrupt vector.
*
* INPUTS:
*       None
*
* OUTPUTS:
*       intVec - Dragonite interrupt vector.
*
* RETURNS:
*       MV_OK      - on success.
*       MV_FAIL    - otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS bspDragoniteGetIntVec(MV_U32 *intVec)
{
    *intVec = INT_LVL_DRAGONITE;

    return MV_OK;
}
#endif /* MV_INCLUDE_DRAGONITE */

/*
 * Ethernet Driver
 */

/*******************************************************************************
* bspEthIntInit
*
* DESCRIPTION:
*       Connects all supported by MII interrupts to driver ISRs.
*
* INPUTS:
*       drvCtrlP     - pointer to the driver control structure
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK if successful, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       BSP ETH driver internal API.
*       This function is different for BSP/LSP.
*
*******************************************************************************/
MV_STATUS bspEthIntInit(int (*bspEthRxReadyIsrF)(void*),
                        int (*bspEthTxDoneIsrF) (void*))
{
	MV_U32 rxInt, txInt;

	rxInt = (mvPpChipIsXCat2() == MV_FALSE)?(16):(INT_LVL_GBE0_RX);
	txInt = (mvPpChipIsXCat2() == MV_FALSE)?(17):(INT_LVL_GBE0_TX);

    /* unmask mgi1 specific interrupts */
    DB(mvOsPrintf("miiTxRxIsrConnect: enabled   Rx & Tx ok\n"));

        /* connect to port interrupt line(s) */
	if( bspEthRxReadyIsrF != NULL )
	{
			pss_rxReadyIsrF = bspEthRxReadyIsrF;
	    	if( request_irq( rxInt, pss_miiRxReadyIsr,
                	(IRQF_DISABLED | IRQF_SAMPLE_RANDOM) , "miiRxReady", NULL ) ) 
        	{
                	printk( KERN_ERR "IRQ: cannot assign irq%d\n", ETH_PORT_IRQ_NUM(1) );
        	}
	}

	if( (mvGndIsTxSyncModeGet() == bspEthTxMode_asynch_E) && (bspEthTxDoneIsrF != NULL))
	{
			pss_txDoneIsrF = bspEthTxDoneIsrF;
         	if( request_irq( txInt, pss_miiTxDoneIsr,
                  	(IRQF_DISABLED | IRQF_SAMPLE_RANDOM) , "miiTxDone", NULL ) ) 
         	{
                 	printk( KERN_ERR "IRQ: cannot assign irq%d\n", 17 );
         	}
	}

	return MV_OK;
}

/*******************************************************************************
* bspEthIntEnable
*
* DESCRIPTION:
*       Enables all relevant interrupt for bspEth driver.
*
* INPUTS:
*       None.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       None.
*
* COMMENTS:
*       BSP ETH driver internal API.
*       This function is different for BSP/LSP.
*
*******************************************************************************/
MV_VOID bspEthIntEnable(MV_BOOL rxIntEnable, MV_BOOL txIntEnable)
{
    /*
     * Enable Layer 1 interrupts (in xCat internal CPU)
     */
    if (rxIntEnable == MV_TRUE)
    {
//         intEnable(INT_LVL_GBE1_RX);
    }

    if (txIntEnable == MV_TRUE)
    {
//         intEnable(INT_LVL_GBE1_TX);
    }
}

/*******************************************************************************
* bspEthIntDisable
*
* DESCRIPTION:
*       Enables all relevant interrupt for bspEth driver.
*
* INPUTS:
*       None.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       None.
*
* COMMENTS:
*       BSP ETH driver internal API.
*       This function is different for BSP/LSP.
*
*******************************************************************************/
MV_VOID bspEthIntDisable(MV_BOOL rxIntEnable, MV_BOOL txIntEnable)
{
    /*
     * Disable Layer 1 interrupts (in xCat internal CPU)
     */
    if (rxIntEnable == MV_TRUE)
    {
//         intDisable(INT_LVL_GBE1_RX);
    }

    if (txIntEnable == MV_TRUE)
    {
//         intDisable(INT_LVL_GBE1_TX);
    }
}

void  *mv_base_p = NULL;
void  *mv_base_v;
void  *mv_top_v;
void  *mv_free_v;
void  *mv_base_descr_v;

/*******************************************************************************
* bspCacheDmaMalloc
*
* DESCRIPTION:
*       Allocate a cache free area for DMA devices.
*
* INPUTS:
*       size_t bytes - number of bytes to allocate
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       virtual address of area
*       NULL - per failure to allocate space
*
* COMMENTS:
*       None
*
*******************************************************************************/
void *bspCacheDmaMalloc(IN size_t bytes)
{
  void *ptr;
  
  if (!mv_base_p) // first time ?
  {
    mv_base_p = (void *)mv_high_memory_paddr;
    mv_base_v = (void *)mv_high_memory_vaddr; 
    mv_top_v = (void *)((MV_U32)mv_base_v + mv_high_memory_len);
    mv_free_v = mv_base_v;
    mv_base_descr_v = mv_top_v;
  }
  
  ptr = mv_free_v;
  mv_free_v = (void *)((MV_U32)mv_free_v + bytes);
  
  /* Marvell, Alex */
#if 0  
  if ((MV_U32)mv_base_descr_v <= (MV_U32)mv_free_v)
    panic(">>> mv area exahasted\n");
#else
  if ((MV_U32)mv_base_descr_v < (MV_U32)mv_free_v)
    panic(">>> mv area exahasted\n");
#endif

  return ptr;
}
/*******************************************************************************
 * bspCacheDmaMallocDescriptors
 *
 * DESCRIPTION:
 *       Allocate a cache free area for descriptors
 *
 * INPUTS:
 *       size_t bytes - number of bytes to allocate
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       virtual address of area
 *       NULL - per failure to allocate space
 *
 * COMMENTS:
 *       None
 *
 *******************************************************************************/
void *bspCacheDmaMallocDescriptors
(
    size_t bytes
)
{
  void *ptr;

  // allocate from mv_top_v downwards. 

  if (!mv_base_p) // first time ?
  {
    mv_base_p = (void *)mv_high_memory_paddr;
    mv_base_v = (void *)mv_high_memory_vaddr; 
    mv_top_v = (void *)((MV_U32)mv_base_v + mv_high_memory_len);
    mv_free_v = mv_base_v;
    mv_base_descr_v = mv_top_v;
  }
  
  ptr = (void *)((MV_U32)mv_base_descr_v - bytes);
  mv_base_descr_v = ptr;

  /* Marvell, Alex */
#if 0 
  if ((MV_U32)mv_base_descr_v <= (MV_U32)mv_free_v)
    panic("Error: mv area exhausted\n");
#else 
  if ((MV_U32)mv_base_descr_v < (MV_U32)mv_free_v)
    panic("Error: mv area exhausted\n");
#endif

  return ptr;  
}

/*******************************************************************************
* bspCacheDmaFree
*
* DESCRIPTION:
*       free a cache free area back to pool.
*
* INPUTS:
*       size_t bytes - number of bytes to allocate
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK   - on success
*       MV_FAIL - on error
*
* COMMENTS:
*       None
*
*******************************************************************************/
MV_STATUS bspCacheDmaFree(void * pBuf)
{
    /* a quick solution to free DMA area,
     * usable for cleanup only */
    if ((MV_U32)mv_free_v >= (MV_U32)pBuf)
    {
        /* it seems memory already freed */
        return MV_FAIL;
    }
    mv_free_v = pBuf;

    return MV_OK;
}


/*******************************************************************************
* bspIsr
*
* DESCRIPTION:
*       This is the ISR reponsible for PP.
*
* INPUTS:
*       irq     - the Interrupt ReQuest number
*       dev_id  - the client data used as argument to the handler
*       regs    - holds a snapshot of the CPU context before interrupt
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       IRQ_HANDLED allways
*
* COMMENTS:
*       None.
*
*******************************************************************************/
static irqreturn_t bspIsr
(
    int             irq,
    void            *dev_id,
    struct pt_regs  *regs
)
{
  if (bspIsrRoutine)
    bspIsrRoutine(/*dev_id*/);  

  return IRQ_HANDLED;
}

/*******************************************************************************
* bspIntConnect
*
* DESCRIPTION:
*       Connect a specified C routine to a specified interrupt vector.
*
* INPUTS:
*       vector    - interrupt vector number to attach to
*       routine   - routine to be called
*       parameter - parameter to be passed to routine
*
* OUTPUTS:
*       None
*
* RETURNS:
*       MV_OK   - on success
*       MV_FAIL - on error
*
* COMMENTS:
*       None
*
*******************************************************************************/
MV_STATUS bspIntConnect
(
    IN  MV_U32           vector,
    IN  MV_VOIDFUNCPTR   routine,
    IN  MV_U32           parameter
)
{
        int rc = 1;

 	bspIsrParameter = parameter;
  	bspIsrRoutine   = routine;

  	rc = request_irq(vector, 
                   (irq_handler_t)bspIsr, 
                   IRQF_DISABLED, "PP_interrupt", (void *)&bspIsrParameter);

  	return (0 == rc) ? MV_OK : MV_FAIL;
}
/*******************************************************************************
* mvSwitchDecodingInit
*
* DESCRIPTION:
*       Initialization of decoding windows for memory access
*       Purpose of this function is to open window to xBar for all enabled SDRAM ChipSelects
*
* INPUTS:
*       baseAddr  - Device Base address
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK      - on success
*       MV_FAIL      - on error
*
* COMMENTS:
*
*******************************************************************************/
MV_STATUS __init mvSwitchDecodingInit(void) 
{
    MV_U32 baseReg=0,sizeReg=0;
    MV_U32 baseToReg=0 , sizeToReg=0;
    MV_U32 target, decodingIndex = 0;
    MV_U32 initBare = 0x3F; /* init value of BaseAddressRegisterEnable - 0x3F - all off */
    MV_U32 baseAddr = 0;

    /* Check all SDRAM chip selects (0 to 3) and process enabled CS 
            We have two indexes - "target" for RAM CS and "decodingIndex" for decoding window number .*/
    for (target = 0; target < 4; target ++)
    {
        /* read size register */
        sizeReg = MV_REG_READ((0x1504 + (target * 8))/*SDRAM_SIZE_REG(target)*/);

        /* check if window enabled */
        if (!(sizeReg & BIT0/*SCSR_WIN_EN*/))
            continue;

        baseReg = MV_REG_READ((0x1500 + (target * 8))/*SDRAM_BASE_ADDR_REG(target)*/); /* read base register */

        baseToReg = PRESTERA_AD_BASE_ADDR_BITS_SET(baseReg, decodingTargetAttribures[target], PRESTERA_AD_DRAM_TARGET_ID);

        CHECK_STATUS(mvSwitchWriteReg(baseAddr, PRESTERA_AD_BASE_ADDR(decodingIndex), baseToReg)); /* Define window */

        sizeToReg = ((sizeReg & /*SCSR_SIZE_MASK*/(0xff << 24)) | 0xFF0000); /* Prestera size decoding fielld is 16 bit [32-16] and CPU size decoding  field is 8 bit [32-24] */
 
        CHECK_STATUS(mvSwitchWriteReg(baseAddr, PRESTERA_AD_SIZE(decodingIndex), sizeToReg)); /* Set window size */		
    
        PRESTERA_AD_BARE_BITS_SET(initBare, decodingIndex); /* set corresponding bit to 0 - enable windows */

        decodingIndex++;
    }
	
    CHECK_STATUS(mvSwitchWriteReg(baseAddr, PRESTERA_AD_BARE, initBare));/* Enable relevant windows */
 
    printk("Switch decoding windows init is done.\n");

    return MV_OK;
}

extern MV_STATUS bspCacheInvalidate(bspCacheType_ENT cacheType, void *address_PTR,
                                                         size_t size);
extern MV_STATUS bspCacheFlush(bspCacheType_ENT cacheType, void *addr, size_t size);

/* EXPORTS */
EXPORT_SYMBOL(bspCacheDmaFree);
EXPORT_SYMBOL(bspCacheDmaMalloc);
EXPORT_SYMBOL(bspCacheFlush);
EXPORT_SYMBOL(bspCacheInvalidate);
EXPORT_SYMBOL(bspDmaRead);
EXPORT_SYMBOL(bspDmaWrite);
EXPORT_SYMBOL(bspEthInputHookAdd);
EXPORT_SYMBOL(bspEthPortDisable);
EXPORT_SYMBOL(bspEthPortEnable);
EXPORT_SYMBOL(bspEthPortRxInit);
EXPORT_SYMBOL(bspEthPortTxQueue);
EXPORT_SYMBOL(bspEthPortTxInit);
EXPORT_SYMBOL(bspEthInit);
EXPORT_SYMBOL(bspEthTxCompleteHookAdd);
EXPORT_SYMBOL(bspIntConnect);
EXPORT_SYMBOL(bspPciConfigReadReg);
EXPORT_SYMBOL(bspPciConfigWriteReg);
EXPORT_SYMBOL(bspPciEnableCombinedAccess);
EXPORT_SYMBOL(bspPciFindDev);
EXPORT_SYMBOL(bspPciGetIntMask);
EXPORT_SYMBOL(bspPciGetIntVec);
EXPORT_SYMBOL(bspReset);
EXPORT_SYMBOL(bspResetInit);
EXPORT_SYMBOL(bspSmiInitDriver);
EXPORT_SYMBOL(bspSmiReadReg);
EXPORT_SYMBOL(bspSmiWriteReg);
EXPORT_SYMBOL(bspTwsiInitDriver);
EXPORT_SYMBOL(bspTwsiMasterReadTrans);
EXPORT_SYMBOL(bspTwsiMasterWriteTrans);
EXPORT_SYMBOL(bspTwsiWaitNotBusy);
#if 1 /* alex, add I2C driver, 20100929 */
EXPORT_SYMBOL(bspI2cRead);
EXPORT_SYMBOL(bspI2cWrite);
#endif
EXPORT_SYMBOL(bspEthPortTxModeSet);
EXPORT_SYMBOL(bspEthCpuCodeToQueue);
EXPORT_SYMBOL(bspPciFindDevReset);
EXPORT_SYMBOL(bspEthRxPacketFree);

EXPORT_SYMBOL(bspDragoniteSWDownload);
EXPORT_SYMBOL(bspDragoniteEnableSet);
EXPORT_SYMBOL(bspDragoniteInit);
EXPORT_SYMBOL(bspDragoniteSharedMemWrite);
EXPORT_SYMBOL(bspDragoniteSharedMemRead);
EXPORT_SYMBOL(bspDragoniteGetIntVec);
EXPORT_SYMBOL(bspDragoniteSharedMemoryBaseAddrGet);
EXPORT_SYMBOL(bspDragoniteFwCrcCheck);
