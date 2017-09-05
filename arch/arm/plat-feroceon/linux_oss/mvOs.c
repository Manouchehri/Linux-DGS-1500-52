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
* mvOsCpuArchLib.c - Marvell CPU architecture library
*
* DESCRIPTION:
*       This library introduce Marvell API for OS dependent CPU architecture 
*       APIs. This library introduce single CPU architecture services APKI 
*       cross OS.
*
* DEPENDENCIES:
*       None.
*
*******************************************************************************/

/* includes */
#include <asm/processor.h>
#include "mvOs.h"
#include "pssBspApis.h"

#ifdef MV_OS_DEBUG
#define DB(x) x
#else
#define DB(x)
#endif

static MV_U32 read_p15_c0 (void);
extern void * bspCacheDmaMalloc(IN size_t bytes); 

/* defines  */
#define ARM_ID_REVISION_OFFS	0
#define ARM_ID_REVISION_MASK	(0xf << ARM_ID_REVISION_OFFS)

#define ARM_ID_PART_NUM_OFFS	4
#define ARM_ID_PART_NUM_MASK	(0xfff << ARM_ID_PART_NUM_OFFS)

#define ARM_ID_ARCH_OFFS	16
#define ARM_ID_ARCH_MASK	(0xf << ARM_ID_ARCH_OFFS)

#define ARM_ID_VAR_OFFS		20
#define ARM_ID_VAR_MASK		(0xf << ARM_ID_VAR_OFFS)

#define ARM_ID_ASCII_OFFS	24
#define ARM_ID_ASCII_MASK	(0xff << ARM_ID_ASCII_OFFS)



void* mvOsIoCachedMalloc( void* osHandle, MV_U32 size, MV_ULONG* pPhyAddr,
			  MV_U32 *memHandle)
{
    void *p = kmalloc( size, GFP_KERNEL );
    *pPhyAddr = pci_map_single( osHandle, p, 0, PCI_DMA_BIDIRECTIONAL );
    return p;
}
void* mvOsIoUncachedMalloc( void* osHandle, MV_U32 size, MV_ULONG* pPhyAddr,
			    MV_U32 *memHandle)
{
    return pci_alloc_consistent( osHandle, size, (dma_addr_t *)pPhyAddr );
}
 
void mvOsIoUncachedFree( void* osHandle, MV_U32 size, MV_ULONG phyAddr, void* pVirtAddr,
			 MV_U32 memHandle)
{
    return pci_free_consistent( osHandle, size, pVirtAddr, (dma_addr_t)phyAddr );
} 
                                                                                                                                               
void mvOsIoCachedFree( void* osHandle, MV_U32 size, MV_ULONG phyAddr, void* pVirtAddr,
		       MV_U32 memHandle )
{
    return kfree( pVirtAddr );
}
 
int mvOsRand(void)
{
    int rand;
    get_random_bytes(&rand, sizeof(rand) );
    return rand;
}

/*
 * use of mvOsIoUncachedMemalign() is coupled with mvOsFree/free;
 * cacheDmaFree() frees the buffer returned by bspCacheDmaMalloc();
 * Alignment should be power of two.
 */
void *mvOsIoUncachedMemalign(void *pDev, MV_U32 size, MV_U32 alignment, MV_U32 *pPhysAddr)
{
    MV_U32 add = alignment - 1 + sizeof(void *);
    void *p, *alignedP;

     p = bspCacheDmaMalloc(size + add);
    if (!p)
        return NULL;

    alignedP = (void *)(((int)p + add) & ~(alignment - 1));
    *((int *)alignedP - 1) = (int)p;

    if (pPhysAddr)
        *pPhysAddr = mvOsIoVirtToPhy(pDev, alignedP);

    return alignedP;
}

extern int bspCacheDmaFree(void *ptr);
void mvOsIoUncachedAlignedFree(void *ptr)
{
	void *toFreeP;
	if( ptr == NULL )
	{
		printk("ERROR:NULL pointer in mvOsIoUncachedAlignedFree\n");
		BUG();
	}

	toFreeP = (void *)(*((int *)ptr - 1));

    	if (ptr) bspCacheDmaFree(toFreeP);
}


/*******************************************************************************
* mvOsCpuVerGet() - 
*
* DESCRIPTION:
*
* INPUT:
*       None.
*
* OUTPUT:
*       None.
*
* RETURN:
*       32bit CPU Revision
*
*******************************************************************************/
MV_U32 mvOsCpuRevGet( MV_VOID )
{
	return ((read_p15_c0() & ARM_ID_REVISION_MASK ) >> ARM_ID_REVISION_OFFS);
}
/*******************************************************************************
* mvOsCpuPartGet() - 
*
* DESCRIPTION:
*
* INPUT:
*       None.
*
* OUTPUT:
*       None.
*
* RETURN:
*       32bit CPU Part number
*
*******************************************************************************/
MV_U32 mvOsCpuPartGet( MV_VOID )
{
	return ((read_p15_c0() & ARM_ID_PART_NUM_MASK ) >> ARM_ID_PART_NUM_OFFS);
}
/*******************************************************************************
* mvOsCpuArchGet() - 
*
* DESCRIPTION:
*
* INPUT:
*       None.
*
* OUTPUT:
*       None.
*
* RETURN:
*       32bit CPU Architicture number
*
*******************************************************************************/
MV_U32 mvOsCpuArchGet( MV_VOID )
{
    return ((read_p15_c0() & ARM_ID_ARCH_MASK ) >> ARM_ID_ARCH_OFFS);
}
/*******************************************************************************
* mvOsCpuVarGet() - 
*
* DESCRIPTION:
*
* INPUT:
*       None.
*
* OUTPUT:
*       None.
*
* RETURN:
*       32bit CPU Variant number
*
*******************************************************************************/
MV_U32 mvOsCpuVarGet( MV_VOID )
{
    return ((read_p15_c0() & ARM_ID_VAR_MASK ) >> ARM_ID_VAR_OFFS);
}
/*******************************************************************************
* mvOsCpuAsciiGet() - 
*
* DESCRIPTION:
*
* INPUT:
*       None.
*
* OUTPUT:
*       None.
*
* RETURN:
*       32bit CPU Variant number
*
*******************************************************************************/
MV_U32 mvOsCpuAsciiGet( MV_VOID )
{
    return ((read_p15_c0() & ARM_ID_ASCII_MASK ) >> ARM_ID_ASCII_OFFS);
}



/*
static unsigned long read_p15_c0 (void)
*/
/* read co-processor 15, register #0 (ID register) */
static MV_U32 read_p15_c0 (void)
{
	MV_U32 value;

	__asm__ __volatile__(
		"mrc	p15, 0, %0, c0, c0, 0   @ read control reg\n"
		: "=r" (value)
		:
		: "memory");

	return value;
}


extern void mv_l2_inv_range(const void *start, const void *end);
void mvOsCacheUnmap(void* pDev, const void *virtAddr, MV_U32 size)
{
#ifdef CONFIG_MV_SP_I_FTCH_DB_INV
    mv_l2_inv_range(virtAddr, (virtAddr + size));
#endif
}

/*******************/
/*  Semaphores & Tasks     */
/*******************/
static struct tasklet_struct rx_tasklet;
static struct tasklet_struct tx_tasklet;
DEFINE_SPINLOCK(rxReadyTaskSemLock);
DEFINE_SPINLOCK(txDoneTaskSemLock);
DEFINE_SPINLOCK(genSyncPoolLock3);
DEFINE_SPINLOCK(genSyncPoolLock4);
DEFINE_SPINLOCK(genSyncPoolLock5);
DEFINE_SPINLOCK(genSyncPoolLock6);
DEFINE_SPINLOCK(genSyncPoolLock7);
int genSyncPoolLockCtr = 3;
unsigned long rxReadyTaskSemLockFlag;
unsigned long txDoneTaskSemLockFlag;
unsigned long genSyncPoolLockFlags3;
unsigned long genSyncPoolLockFlags4;
unsigned long genSyncPoolLockFlags5;
unsigned long genSyncPoolLockFlags6;
unsigned long genSyncPoolLockFlags7;

DEFINE_SPINLOCK(rxPathSemIdLock);
DEFINE_SPINLOCK(txPathSemIdLock);
unsigned long rxPathSemIdLockFlag;
unsigned long txPathSemIdLockFlag;


/*******************************************************************************
 * mvOsSemCreate
 */
MV_STATUS mvOsSemCreate(MV_8 *name, MV_U32 init, MV_U32 count, MV_U32 *smid)
{
	if( 0==strcmp(name,"rxReadyTaskSem") )
	{
         	*smid = 1;
	}
	if( 0==strcmp(name,"txDoneTaskSem") )
	{
         	*smid = 2;
	}
	if( 0==strcmp(name,"genSyncPool") )
	{
        *smid = genSyncPoolLockCtr;
		genSyncPoolLockCtr++;
	}
	if( 0==strcmp(name,"rxPathSemId") )
	{
		*smid = 10;
	}
	if( 0==strcmp(name,"txPathSemId") )
	{
        *smid = 11;
	}
	
    return MV_OK;
}

/*******************************************************************************
 * mvOsSemDelete
 */
MV_STATUS mvOsSemDelete(MV_U32 smid)
{
    return MV_OK;
}

/*******************************************************************************
 * mvOsSemWait
 */
MV_STATUS mvOsSemWait(MV_U32 smid, MV_U32 time_out)
{
	switch(smid)
	{
	case 1:
// 		spin_lock_irqsave(&rxReadyTaskSemLock, rxReadyTaskSemLockFlag); 
		break;
	case 2:
// 		spin_lock_irqsave(&txDoneTaskSemLock, txDoneTaskSemLockFlag); 
		break;
	case 3:
		spin_lock_irqsave(&genSyncPoolLock3, genSyncPoolLockFlags3); break;
	case 4:
		spin_lock_irqsave(&genSyncPoolLock4, genSyncPoolLockFlags4); break;
	case 5:
		spin_lock_irqsave(&genSyncPoolLock5, genSyncPoolLockFlags5); break;
	case 6:
		spin_lock_irqsave(&genSyncPoolLock6, genSyncPoolLockFlags6); break;
	case 7:
		spin_lock_irqsave(&genSyncPoolLock7, genSyncPoolLockFlags7); break;
	case 10:
		spin_lock_irqsave(&rxPathSemIdLock, rxPathSemIdLockFlag); break;
	case 11:
		spin_lock_irqsave(&txPathSemIdLock, txPathSemIdLockFlag); break;
	default:
		break;
	}

    return MV_OK;
}


/*******************************************************************************
 * mvOsSemSignal
 */
MV_STATUS mvOsSemSignal(MV_U32 smid)
{
	if( smid == 1)
	{
//     	DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
 		tasklet_hi_schedule(&rx_tasklet);
    	return MV_OK;
	}	
	if( smid == 2)
	{
//     	DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
 		tasklet_hi_schedule(&tx_tasklet);
    	return MV_OK;
	}	

//     DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
	switch(smid)
	{
	case 3:
	  	spin_unlock_irqrestore(&genSyncPoolLock3, genSyncPoolLockFlags3);break;
	case 4:
	  	spin_unlock_irqrestore(&genSyncPoolLock4, genSyncPoolLockFlags4);break;
	case 5:
	  	spin_unlock_irqrestore(&genSyncPoolLock5, genSyncPoolLockFlags5);break;
	case 6:
	  	spin_unlock_irqrestore(&genSyncPoolLock6, genSyncPoolLockFlags6);break;
	case 7:
	  	spin_unlock_irqrestore(&genSyncPoolLock7, genSyncPoolLockFlags7);break;
	case 10:
	  	spin_unlock_irqrestore(&rxPathSemIdLock, rxPathSemIdLockFlag);break;
	case 11:
	  	spin_unlock_irqrestore(&txPathSemIdLock, txPathSemIdLockFlag);break;
	default:
		break;
	}
//     DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );

    	return MV_OK;
}


/*******************************************************************************
 * mvOsTaskCreate
 */
MV_STATUS mvOsTaskCreate(char          *name,
                         unsigned long  prio,
                         unsigned long  stack,
                         void* start_addr,
                         MV_VOID       *arglist,
                         unsigned long *tid)
{
	if( 0==strcmp(name,"BSP_rx") )
	{
    	DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
        tasklet_init(&rx_tasklet, start_addr, (unsigned int) 0);
		*tid = 1;
		return MV_OK;
	}
	if( 0==strcmp(name,"BSP_tx") )
	{
    	DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
        tasklet_init(&tx_tasklet, start_addr, (unsigned int) 0);
		*tid = 2;
		return MV_OK;
	}
	return MV_FAIL;
}

/*******************************************************************************
 * mvOsTaskIdent
 */
MV_STATUS mvOsTaskIdent(char *name, unsigned long *tid)
{
    return MV_OK;
}

/*******************************************************************************
 * mvOsTaskDelete
 */
MV_STATUS mvOsTaskDelete(unsigned long tid)
{
	// here possible set global flags to disable schedule of the tasklet in SemSignal
    return MV_OK;
}

/*******************************************************************************
 * mvOsTaskSuspend
 */
MV_STATUS mvOsTaskSuspend(unsigned long tid)
{
    return MV_OK;
}

/*******************************************************************************
 * mvOsTaskResume
 */
MV_STATUS mvOsTaskResume(unsigned long tid)
{
    return MV_OK;
}

inline void *mvOsCalloc(int _n_, int _size_)
{  
	void* ptr;
	ptr = kmalloc(_n_*_size_,GFP_ATOMIC);
	if( ptr == NULL){
		ptr = vmalloc(_n_*_size_);
	}

	if( ptr == NULL)
		return NULL;

	memset(ptr, 0, _n_*_size_);
	return ptr;
}



