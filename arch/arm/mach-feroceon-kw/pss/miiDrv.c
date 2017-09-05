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

#include "mii.h"
#include "miiDrv.h"
#include "pssBspApis.h"

/*******************************************************************************
 * Globals
 */
MII_ETH_TX_MODE G_miiTxMode;
MV_U32 mii_rx_is_ready_on_queue = 0;

static struct tasklet_struct rx_tasklet;
static struct tasklet_struct tx_tasklet;
DEFINE_SPINLOCK(mii_rx_lock);
extern spinlock_t tx_lock;
int mii_rx_work_in_progress = 0;

/*******************************************************************************
 * Declarations
 */
void miiRxReadyJob(void);

void miiTxDoneJob(void);
irqreturn_t miiTxDoneIsr(int irq , void *dev_id);

void miiSignalRxReady(void);
void signal_rx_was_processed(void);

void BSP_traffic_dispatch_tx(unsigned long ignored);
void BSP_traffic_dispatch(unsigned long ignored);

/*******************************************************************************
 * Externs
 */
extern bspEthTxMode_ENT        bspEthTxMode;


void BSP_traffic_dispatch_task(unsigned long ignored)
{
        DB(printk("Started BSP_traffic_dispatch_task"));
        
        miiRxReadyJob();
}

void BSP_traffic_dispatch_tx(unsigned long ignored)
{
        DB(printk("Started BSP_traffic_dispatch_tx"));
        
        miiTxDoneJob();
}

void miiTxDoneJob(void)
{
  unsigned long flags;

  	if(bspEthTxMode == bspEthTxMode_asynch_E)
  	{
    		spin_lock_irqsave(&tx_lock,flags);
    		miiEthPortTxDone();
     		spin_unlock_irqrestore(&tx_lock,flags);

    		/* Re-enable TxDone interrupt !!! Remember to renable MISC interrupts too*/
    		ENABLE_MGI_TX_DONE_INTERRUPT(EGIGA_CPU_PORT);
  	}
        return;
}

void miiRxReadyJob(void)
{
   	unsigned long flags;

  	spin_lock_irqsave(&mii_rx_lock, flags);
  	mii_rx_work_in_progress = 0;
  	spin_unlock_irqrestore(&mii_rx_lock, flags);

  	miiRxReady();

  	/* re-unmask mgi1 specific interrupts */
  	ENABLE_MGI_RX_ALL_INTERRUPTS;

  	return;
}

irqreturn_t  miiRxReadyIsr(int irq , void *dev_id)
{
   	unsigned long flags;

  	/* Disable RX interrupts */
  	DISABLE_MGI_RX_ALL_INTERRUPTS;

       /* Clear bits for RX */
        MV_REG_WRITE(ETH_INTR_CAUSE_REG(EGIGA_CPU_PORT), 0/*~MGI_RX_READY_MASK*/);

        /* signal Rx task to start working */
  	spin_lock_irqsave(&mii_rx_lock, flags);
  	if (mii_rx_work_in_progress == 1)
  	{
    	spin_unlock_irqrestore(&mii_rx_lock, flags);
    	return IRQ_HANDLED;    
  	}
  
  	mii_rx_work_in_progress = 1;
  tasklet_hi_schedule(&rx_tasklet);
  	spin_unlock_irqrestore(&mii_rx_lock, flags);
  
  	return IRQ_HANDLED;    
}

irqreturn_t miiTxDoneIsr(int irq , void *dev_id)
{
    	/* Disable TxDone interrupt (all interrupts in Isr Ext register) */
    	DISABLE_MGI_TX_DONE_INTERRUPT(EGIGA_CPU_PORT);

    	if(bspEthTxMode == bspEthTxMode_asynch_E)
    	{
      		/* signal Tx task to start working */
      tasklet_hi_schedule(&tx_tasklet);
      
      		/* Acknowledge mgi-tx-done by Clearing bits for TX_DONE */
      		MV_REG_WRITE(ETH_INTR_CAUSE_EXT_REG(EGIGA_CPU_PORT), ~MGI_TX_DONE_MASK);
    	}
    	else
    	{
      		// leave disabled for sync mode
    	}

        return IRQ_HANDLED;    
}

MV_STATUS miiIsrConnect(void)
{

        /* Unicast Promiscuous mode - to enable the recieving of unknown unicast for GBE 1 */
        MV_REG_BIT_SET(0x76400, 0x1);

        /* unmask mgi1 specific interrupts */
        MV_REG_WRITE( ETH_INTR_MASK_REG(EGIGA_CPU_PORT), 0x00003FC);//0x7F803FD);
        DB(mvOsPrintf("miiTxRxIsrConnect: enabled   Rx & Tx ok\n"));

        /* connect to port interrupt line(s) */
        if( request_irq( 16, miiRxReadyIsr,
                (IRQF_DISABLED | IRQF_SAMPLE_RANDOM) , "miiRxReady", NULL ) ) 
        {
                printk( KERN_ERR "IRQ: cannot assign irq%d\n", ETH_PORT_IRQ_NUM(1) );
        }
        tasklet_init(&rx_tasklet, BSP_traffic_dispatch_task, (unsigned int) 0);

	if(bspEthTxMode == bspEthTxMode_asynch_E)
	{
         	if( request_irq( 17, miiTxDoneIsr,
                  	(IRQF_DISABLED | IRQF_SAMPLE_RANDOM) , "miiTxDone", NULL ) ) 
         	{
                 	printk( KERN_ERR "IRQ: cannot assign irq%d\n", 17 );
         	}
        tasklet_init(&tx_tasklet, BSP_traffic_dispatch_tx, (unsigned int) 0);
	}

	return MV_OK;
}



