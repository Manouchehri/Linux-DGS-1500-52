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
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/init.h>
	
#if defined (MV_PRESTERA_SWITCH)
#include "mvOs.h"
#include "mii.h"
#include "eth/mvEth.h"
#include "mvNetDrvCommon.h"
#include "mvGnd.h"
#include "ctrlEnv/mvCtrlEnvLib.h"
#include "mvPex.h"
#include "mvCtrlEnvRegs.h"

//#define MV_DEBUG
#ifdef MV_DEBUG
#define DB(x) x
#else
#define DB(x)
#endif

/****************************************************** 
* driver internal definitions --                     *
******************************************************/ 
#define PRESTERA_MAC_HEADER_SIZE        12
#define PRESTERA_PORT_NUM               26
#define MV_DEFAULT_MTU  1468

MV_U32 G_switchTxQ = 0;

typedef struct _switchPriv
{
    int port;
    int ifindex;
	MV_VOID *halPriv;
	MV_U32 rxqCount;
	MV_U32 txqCount;
	MV_BOOL devInit;
} switchPriv;
	
/****************************************************** 
* functions prototype --                             *
******************************************************/
static int mvMuxLoad( int port, char *name, char *enet_addr );
static int mvMuxInit( struct net_device *dev );
static int mvMuxHalt( struct net_device *dev );
static int mvMuxSend( struct sk_buff *skb , struct net_device *dev );
static int mvPortLoad( int port, char *name, char *enet_addr );
static int mvPortInit( struct net_device *dev );
static int mvPortHalt( struct net_device *dev );
static int mvPortTx( struct sk_buff *skb, struct net_device *dev );
static int mvPortForward( struct sk_buff *skb, struct net_device *dev );
static int mv_set_mac_addr( struct net_device *dev, void *addr );

MV_STATUS switchDrvOsPktFree(MV_VOID *osPkt);
MV_STATUS switchFwdRxPktToOs       (MV_GND_PKT_INFO *pktInfoP, MV_U32 rxQ);
MV_STATUS muxEthCreateTxTask(void);
MV_STATUS muxEthCreateRxTask(void);
MV_STATUS switchFwdRxPkt(MV_GND_PKT_INFO *pktInfoP, MV_U32 rxQ);

extern void* cpssTxCallback;
extern void* cpssRxCallback;
extern MV_32  mvBoardSwitchGpioPinGet(int device);
extern unsigned long mv_real_memory;
extern MV_U32 _prestera_dev_base_addr[];

int G_rxReadySemId;
int G_txDoneSemId;
unsigned long G_rxTaskId;
unsigned long G_txTaskId;

/****************************************************************************** 
Get U-Boot configuration enabling full PP configuration by network device
******************************************************************************/
MV_BOOL standalone_network_device = MV_FALSE;
static int __init standalone_network_device_env(char *str)
{
    standalone_network_device = MV_TRUE;
	return 0;
}
__setup("standalone_network_device", standalone_network_device_env);

/****************************************************************************** 
Get U-Boot configuration defining engine used by traffic from PP to CPU
******************************************************************************/
MV_BOOL standalone_network_mode_RGMII_for_PP = MV_TRUE;
static int __init egiga0_env(char *str)
{
	printk("OOB used via eth0\n");
    standalone_network_mode_RGMII_for_PP = MV_FALSE;
	return 0;
}
__setup("egiga0", egiga0_env);
static int __init ppsdma_env(char *str)
{
	printk("OOB used via SDMA\n");
    standalone_network_mode_RGMII_for_PP = MV_FALSE;
	return 0;
}
__setup("ppsdma", ppsdma_env);
static int __init ppmii_env(char *str)
{
	printk("OOB used via RGMII\n");
    standalone_network_mode_RGMII_for_PP = MV_TRUE;
	return 0;
}
__setup("ppmii", ppmii_env);

/*********************************************************** 
* mv_linux_mux_initialize --                               *
*   main driver initialization. loading the interfaces.   *
***********************************************************/
int mv_linux_mux_initialize(void)
{
    int dev, port;
    MV_8 macaddr[18];
    MV_8 enet_addr[6];
    int NumberOfDevices, portsNumber;
    MV_8 name[IFNAMSIZ+1];
    NumberOfDevices = mvSwitchGetDevicesNum(); 

    /*Create MUX device(s)*/
    for( dev=0; dev < NumberOfDevices; dev++ )
    {
        /* interface name */
        sprintf( name, "mux%d", dev );

        mvMuxLoad( dev, name, enet_addr );
    }

    portsNumber = PRESTERA_PORT_NUM * NumberOfDevices;
    /* Create port logical interfaces */
    for( port=0; port < portsNumber; port++ )
    {
        /* interface name */
        sprintf( name, "p%d", port );
        /* interface MAC addr extract */
        enet_addr[0] = 0;
        enet_addr[1] = enet_addr[2] = enet_addr[3] = enet_addr[4] =
            (port/PRESTERA_PORT_NUM);
        enet_addr[5]=port+1;

		/*Add port MAC address to Switch with Forward to CPU command*/
        sprintf(macaddr,"%02x:%02x:%02x:%02x:%02x:%02x", 
                    enet_addr[0],enet_addr[1],enet_addr[2],
                    enet_addr[3],enet_addr[4],enet_addr[5]);
        if(setCPUAddressInMACTAble( port/PRESTERA_PORT_NUM,
                                        macaddr,1) != MV_OK)
        {
            printk( "\nError: (Prestera) Unable to teach CPU MAC address\n");
            return -1;
        }
        mvPortLoad( port, name, enet_addr );
    } 

    return 0;
}
	
/**************************************************************** 
* mvMuxLoad --                                               *
*   load a SDMA network interface into Linux network core.      *
*   initialize sw structures e.g. private, rings, etc.          *
*****************************************************************/
static int mvMuxLoad( int port, char *name, char *enet_addr ) 
{
    struct net_device   *dev = NULL;
    int                 ret = 0;
    switchPriv *priv = NULL;

    DB( printk( "%s: %s load - ", __FUNCTION__, name ) );

    dev = alloc_etherdev(sizeof(switchPriv));

    if( !dev ) {
        ret = -ENOMEM;
        goto error;
    }

    priv = (switchPriv *)netdev_priv(dev);
    if( !priv ) {
        DB( printk( "%s: %s falied to alloc egiga_priv (error)\n",
                        __FUNCTION__, name ) );
        goto error;
    }
    memset( priv, 0, sizeof(switchPriv) );

    /* init device methods */
    strcpy( dev->name, name );
    dev->base_addr = 0;
    dev->irq = mvBoardSwitchGpioPinGet(port);
    dev->open = mvMuxInit;
    dev->stop = mvMuxHalt;
    dev->hard_start_xmit = mvMuxSend;
    dev->watchdog_timeo = 5*HZ;
    dev->tx_queue_len = PRESTERA_TXQ_LEN;
    memcpy(dev->dev_addr, enet_addr,6);
    dev->mtu = MV_DEFAULT_MTU;

    if(register_netdev(dev)) {
        printk( KERN_ERR "%s: register failed\n", dev->name );
        ret = -ENODEV;
        goto error;
    }

    DB( printk( "%s: %s load ok\n", __FUNCTION__, name ) );
    return 0;

    error:
    printk( "%s: %s load failed\n", __FUNCTION__, name );
    if( dev ) unregister_netdevice(dev);
    if( dev ) kfree( dev );
    return -1;
}

/**************************************************************** 
* switchFwdRxPkt --                              *
*   Interrupt handler callback the RX process for MII  mode     *
*****************************************************************/
MV_STATUS switchFwdRxPkt(MV_GND_PKT_INFO *pktInfoP, MV_U32 rxQ)
{
    struct net_device *dev;
    MV_U32          buf_size;
    struct sk_buff  *skb;
    MV_8 name[IFNAMSIZ+1];

    //Extend here for48 ports
    dev = __dev_get_by_name("mux0");
    if( dev == NULL )
    {
        printk("Failed to find network device\n");
        return MV_FAIL;
    }

    /*Allocate skb*/
    buf_size = dev->mtu + PRESTERA_MAC_HEADER_SIZE + 4 +
               CPU_D_CACHE_LINE_SIZE /* 32(extra for cache prefetch) */ + 
               8/* +8 to align on 8B */;
    skb = dev_alloc_skb(buf_size);

    memcpy( skb->data, 
		pktInfoP->pFrags->bufVirtPtr, 
		pktInfoP->pktSize);

    skb_put(skb, pktInfoP->pktSize);
    sprintf( name, "p%d", mvSwitchGetPortNumFromExtDsa(pktInfoP) );
    skb->dev = __dev_get_by_name(name);
    if( skb->dev == NULL )
    {
        printk("Failed to find network device\n");
        kfree_skb(skb);
        return MV_FAIL;
    }

    skb->protocol = eth_type_trans(skb, skb->dev);
    skb->ip_summed = CHECKSUM_PARTIAL;
	skb->len = pktInfoP->pktSize - PRESTERA_MAC_HEADER_SIZE - ETH_MV_HEADER_SIZE;

    /*Send the packet to port device*/
    mvPortForward(skb,skb->dev);

    dev->stats.rx_packets++;
    if (switchGenFreeRxPkt(pktInfoP, rxQ) != MV_OK)
    {
       	mvOsPrintf("%s: switchGenFreeRxPkt failed.\n", __func__);
        /*kfree_skb(skb);*/ /*Note - kfree_skb alredy called by mvPortForward*/
       	return MV_FAIL;
    }

    return 0;
}

MV_STATUS mvSwitchDefaultPPConfig(void)
{
    MV_SWITCH_GEN_HOOK_HW_INIT_PORTS fP;

    fP = mvBoardSwitchPortsInitFuncGet();

    mvOsPrintf("Initializing switch ports...");

    if (fP != NULL)
    {
        if (fP() != MV_OK)
        {
            mvOsPrintf("PP-EEPROM simulation failed.\n");
            return 1; /* failure */
        }
        mvOsPrintf("Done.\n");
    }
    else
    {
        mvOsPrintf("PP-EEPROM simulation failed (no function found).\n");
        return 1; /* failure */
    }

    return 0;
}


/*******************************************************************************
* bspEthRxReadyTask
*
* DESCRIPTION:
*       This is the main funtion of RX task, which is needed in DSR mode
*       (old approach). The task processes received by MII packets.
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
*       None.
*
*******************************************************************************/
void muxEthRxReadyTask(unsigned long ignored)
{
    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
    if (G_rxReadySemId == 0)
    {
        mvOsPrintf("%s: rxReadySemId doesn't exist.\n", __func__);
        return;
    }

    if (switchGenRxJob() != MV_OK)
    {
        mvOsPrintf("%s: mvGndRxBottomHalf failed.\n", __func__);
        return;
    }

    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
    return;
}

/*******************************************************************************
* bspEthTxDoneTask
*
* DESCRIPTION:
*       This is the main funtion of TX task, which is needed in DSR mode
*       (old approach). The task processes transmitted by MII packets.
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
*       None.
*
*******************************************************************************/
void muxEthTxDoneTask(unsigned long ignored)
{
    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
    if (G_txDoneSemId == 0)
    {
        mvOsPrintf("%s: txDoneSemId doesn't exist.\n", __func__);
        return ;
    }

    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
    if (mvGndTxBottomHalf() != MV_OK)
    {
        mvOsPrintf("%s: mvGndRxBottomHalf failed.\n", __func__);
        return ;
    }

    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
    return ;
}

/*******************************************************************************
* bspEthCreateRxTask
*
* DESCRIPTION:
*       Creates RX task for driver DSR mode (old approach).
*       Each tasks processes RX_READY queues and calls for each
*       frame the registered callback.
*
* INPUTS:
*       None.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK if successful, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS muxEthCreateRxTask()
{
    MV_STATUS         status;

    status = mvOsTaskCreate("BSP_rx",
                            50                               /* priority    */,
                            0x2000                           /* stackSize   */,
                            muxEthRxReadyTask                /* entry point */,
                            NULL                             /* arg list    */,
                            &G_rxTaskId);
    if (status != MV_OK)
    {
        mvOsPrintf("%s: Could not spawn Rx task.\n", __func__);
        return MV_FAIL;
    }

    /*
     * Create semaphores for RX_READY task (DSR mode - old approach).
     */
    if (mvOsSemCreate("rxReadyTaskSem",
                      0, /* init sem value is busy  */
                      1, /* 1 ==> create binary sem */
                      &G_rxReadySemId) != MV_OK)
    {
        mvOsPrintf("%s: mvOsSemCreate failed.\n", __func__);
        return MV_FAIL;
    }

    return MV_OK;
}

/*******************************************************************************
* bspEthCreateTxTask
*
* DESCRIPTION:
*       Creates TX task for driver DSR mode (old approach).
*       Each tasks processes TX_DONE queues and calls for each
*       frame the registered callback.
*
* INPUTS:
*       None.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK if successful, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS muxEthCreateTxTask(void)
{
    MV_STATUS         status;

    status = mvOsTaskCreate("BSP_tx",
                            50                               /* priority    */,
                            0x2000                           /* stackSize   */,
                            muxEthTxDoneTask                 /* entry point */,
                            NULL                             /* arg list    */,
                            &G_txTaskId);
    if (status != MV_OK)
    {
        mvOsPrintf("%s: Could not spawn Tx task.\n", __func__);
        return MV_FAIL;
    }

    /*
     * Create semaphores for TX_DONE task (DSR mode - old approach).
     */
    if (mvOsSemCreate("txDoneTaskSem",
                      0, /* init sem value is busy  */
                      1, /* 1 ==> create binary sem */
                      &G_txDoneSemId) != MV_OK)
    {
        mvOsPrintf("%s: mvOsSemCreate failed.\n", __func__);
        return MV_FAIL;
    }

    return MV_OK;
}

/*
 * Struct needed to init Generic Network Driver (GND).
 */
static MV_SWITCH_GEN_INIT G_swGenInit;
extern MV_STATUS bspEthUserTxDone(MV_GND_PKT_INFO *pktInfoP);
extern MV_STATUS bspEthFwdRxPkt(MV_GND_PKT_INFO *pktInfoP, MV_U32 rxQ);
/*******************************************************************************
 * switchEndInitSwitchGen
 */
MV_STATUS switchEndInitSwitchGen(void)
{
    MV_SW_GEN_RX_HOOKS    rxHooks;
    MV_SW_GEN_TX_HOOKS    txHooks;
    MV_U32                hookId;
    MV_CHIP_FEATURES *featureSetP = mvChipFeaturesGet();

    if (switchDrvGenIsInited() == MV_TRUE)
    {
        return MV_FAIL;
    }

    /*
     * Init and configure lower layer (including hardware).
     */
    if(standalone_network_mode_RGMII_for_PP == MV_TRUE)
      	G_swGenInit.isMiiMode = MV_TRUE;
    else
      	G_swGenInit.isMiiMode = MV_FALSE;

    G_swGenInit.gndInitP  = NULL; /* use defaults */
    G_swGenInit.isTxSyncMode = MV_TRUE;
    G_swGenInit.gbeDefaultIndex = featureSetP->miiGbeIndex;
    if (switchDrvGenInit(&G_swGenInit) != MV_OK)
    {
        mvOsPrintf("%s: switchDrvGenInit failed.\n", __func__);
        return MV_FAIL;
    }

    /*
     * Register RX_READY callbacks.
     */
    if (switchDrvGenHookSetCalcFwdRxPktHookId(
            (MV_SWITCH_GEN_HOOK_CALC_PKT_ID)mvSwitchGetPortNumFromExtDsa)
        != MV_OK)
        {
        mvOsPrintf("%s: switchDrvGenHookSetCalcFwdRxPktHookId failed.\n", __func__);
        return MV_FAIL;
    }

    if (switchGenHooksInitRx(MV_PP_NUM_OF_PORTS) != MV_OK)
    {
        mvOsPrintf("%s: switchGenHooksInitRx failed.\n", __func__);
        return MV_FAIL;
    }

    rxHooks.fwdRxReadyPktHookF               = switchFwdRxPktToOs;
    rxHooks.hdrAltBeforeFwdRxPktHookF        = mvSwitchExtractExtDsa;
    rxHooks.hdrAltBeforeFwdRxFreePktHookF    = mvSwitchInjectExtDsa;

    for (hookId = 0; hookId < MV_PP_NUM_OF_PORTS; hookId++)
    {
        if (switchGenHooksFillRx(&rxHooks, hookId) != MV_OK)
        {
            mvOsPrintf("%s: switchGenHooksFillRx failed.\n", __func__);
            return MV_FAIL;
        }
    }

    /*
     * Register TX_DONE callbacks.
     */
    if (switchGenHooksInitTx(MV_NET_OWN_TOTAL_NUM) != MV_OK)
    {
        mvOsPrintf("%s: switchGenHooksInitTx failed.\n", __func__);
        return MV_FAIL;
    }

    txHooks.hdrAltBeforeFwdTxDonePktHookF    = /*mvSwitchRemoveExtDsa*/NULL;
    txHooks.txDonePktFreeF                   =
            (MV_SWITCH_GEN_HOOK_DRV_OS_PKT_FREE)switchDrvOsPktFree;
    txHooks.hdrAltBeforeTxF = mvSwitchInjectExtDsa;

    if (switchGenHooksFillTx(&txHooks, MV_NET_OWN_NET_DRV_RX_REMOVE_TX_ADD_DSA) != MV_OK)
    {
        mvOsPrintf("%s: switchGenHooksFillTx failed.\n", __func__);
        return MV_FAIL;
    }

    return MV_OK;
}

/**************************************************************** 
* mvMuxInit --                                               *
*   load a SDMA network interface into Linux network core.      *
*   initialize sw structures e.g. private, rings, etc.          *
*   connect to interrupt, enable traffic                        *
*****************************************************************/
static int mvMuxInit( struct net_device *dev)
{
    static int mvPortInit_time=0;

    if( dev == NULL )
    {
        printk("Failed to find network device\n");
        return MV_FAIL;
    }
    DB( printk( "%s: %s init - ", __FUNCTION__, dev->name ) );

    if ( mvPortInit_time == 1)
        return 0;
    mvPortInit_time = 1;

    if(standalone_network_device == MV_TRUE)
        CHECK_STATUS(mvSwitchDefaultPPConfig());

    /* in default link is down */
    netif_carrier_off( dev );

    /* Stop the TX queue -
	it will be enabled upon PHY status change after link-up interrupt/timer */
    netif_stop_queue( dev );

    if (muxEthCreateRxTask() != MV_OK)
    {
        mvOsPrintf("%s: bspEthCreateRxTask failed.\n", __func__);
        return MV_FAIL;
    }

    if (muxEthCreateTxTask() != MV_OK)
    {
        mvOsPrintf("%s: bspEthCreateTxTask failed.\n", __func__);
        return MV_FAIL;
    }

    if(standalone_network_device == MV_TRUE)
    {
        switchEndInitSwitchGen();
        if (switchGenStart() != MV_OK)
        {
            mvOsPrintf("%s: switchGenStart failed.\n", __func__);
            goto error;
        }
    }

    /*Do Linux Stuff for full enable*/
    netif_carrier_on( dev );
    netif_wake_queue( dev );

    if(!netif_running(dev))
    {
        printk("MUX device not UP, port devices used\n");
    }

    DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );
    return 0;

    error:
    printk( "%s: %s failed\n", __FUNCTION__, dev->name );
    return 1;
}

	
/**************************************************************** 
* mvMuxHalt --                                               *
*   Stop port device                                       *
*****************************************************************/
static int mvMuxHalt( struct net_device *dev )
{
	DB( printk( "%s: %s halt - ", __FUNCTION__, dev->name ) );
	DB( printk( "%s: %s complete\n", __FUNCTION__, dev->name ) );
	return 0;
}

/**************************************************************** 
* skbToPktInfo --                                                 *
*   Convert skb to BUF_INFO-Buffer list                           *
*****************************************************************/
MV_STATUS skbToPktInfo(struct sk_buff *skb,
			MV_U8	*txPacketData_,
			MV_U32  txPacketPhyData_,
                        MV_PKT_INFO  *pPktInfo)
{
    MV_BUF_INFO *pBufInfo = pPktInfo->pFrags;

    /* set a BufInfo */
    pBufInfo[0].bufVirtPtr = txPacketData_;
    pBufInfo[0].bufPhysAddr = txPacketPhyData_;
    pBufInfo[0].dataSize   = skb->len;

    /* Store pointer to Mblk structure in MV_PKT_INFO */
    pPktInfo->osInfo   = (MV_ULONG)skb;
    pPktInfo->numFrags = 1;
    pPktInfo->pktSize  = skb->len+4;

    pPktInfo->status = 0;

    return MV_OK;
}

/**************************************************************** 
* mvMuxSend --                                                 *
*   Perform MII  TX                                             *
*****************************************************************/
static int mvMuxSend( struct sk_buff *skb , struct net_device *dev )
{
    MV_GND_PKT_INFO  *pktInfo;
    MV_U32 txQ = G_switchTxQ;
    DB(mvOsPrintf("ENTER %s: %s\n", __func__, dev->name));

    pktInfo = switchGenTxPktInfoGet();
    if (pktInfo == NULL)
    {
        mvOsPrintf("%s: switchGenTxPktInfoGet failed.\n", __func__);
        goto error;
    }

    netif_stop_queue( dev );

    if (skbToPktInfo(skb, skb->data, mvOsIoVirtToPhy(NULL, (MV_U32)skb->data),
                     (MV_PKT_INFO *)pktInfo) != MV_OK)
    {
        mvOsPrintf("%s: mBlkToPktInfo failed.\n", __func__);
        goto error;
    }

    pktInfo->ownerId = MV_NET_OWN_NET_DRV_RX_REMOVE_TX_ADD_DSA;

    /* send the packet */
    if (switchGenSendPkt(pktInfo, txQ) != MV_OK)
    {
        mvOsPrintf("%s: mvGndSendPkt failed.\n", __func__);
        goto error;
    }

    if(netif_queue_stopped(dev))
        netif_wake_queue( dev );

    dev->stats.tx_packets++;

    DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );

    return 0;

    error:
    DB( printk( "%s: %s failed\n", __FUNCTION__, dev->name ) );
    return 1;
}

/************************************************************************
* mv_set_mac_addr_internals --                                          *
*   Set port mac address in PP                                          *
*************************************************************************/
static int mv_set_mac_addr_internals(struct net_device *dev, void *addr )
{
    /* skip on first 2B (ether HW addr type) */
    u8* mac = &(((u8*)addr)[2]);
    int i;
    MV_U32  devNum = dev->name[4] - '0';

    /* set new addr in hw */
    if( setCPUAddressInMACTAble( devNum, mac, 1) != MV_OK ) {
        printk( KERN_ERR "%s: ethSetMacAddr failed\n", dev->name );
        return -1;
    }

    /* set addr in the device */
    for( i = 0; i < 6; i++ )
        dev->dev_addr[i] = mac[i];

    printk( KERN_NOTICE "%s: mac address changed\n", dev->name );
    return 0;
}

/*******************************************************************************
* mv_set_mac_addr --                                                           *
*   Change MAC address for network device                                      *
*******************************************************************************/
static int mv_set_mac_addr( struct net_device *dev, void *addr )
{
    if(!netif_running(dev)) {
        if(mv_set_mac_addr_internals(dev, addr) == -1)
            goto error;
        return 0;
    }

    if( mvMuxHalt( dev )) {
        printk( KERN_ERR "%s: stop interface failed\n", dev->name );
        goto error;
    }

    if(mv_set_mac_addr_internals(dev, addr) == -1)
        goto error;

    if(mvMuxInit( dev )) {
        printk( KERN_ERR "%s: start interface failed\n", dev->name );
        goto error;
    }

    return 0;

    error:
    printk( "%s: set mac addr failed\n", dev->name );
    return -1;
}


/*******************************************************************************
 * switchDrvIntEnableSdmaRx
 */
MV_STATUS switchDrvIntEnableSdmaRx(MV_U32 dev)
{
    return MV_OK;
}
/*******************************************************************************
 * switchDrvIntEnableSdmaTx
 */
MV_STATUS switchDrvIntEnableSdmaTx(MV_U32 dev)
{
    return MV_OK;
}
/*******************************************************************************
 * switchDrvOsPktFree
 */
MV_STATUS switchDrvOsPktFree(MV_VOID *osPkt)
{
    struct sk_buff *skb = (struct sk_buff*)osPkt;
    kfree_skb(skb);
    return MV_OK;
}

void (*G_rxReadyIsrF)(void *);
void (*G_txDoneIsrF)(void *);
void (*G_rxReadyIsrF_sdma)(void *);
void (*G_txDoneIsrF_sdma)(void *);

/*******************************************************************************
 * switchRxReadyIsrCb
 */
MV_VOID switchRxReadyIsrCb(void)
{
    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
	mvOsSemSignal(G_rxReadySemId);
}

/*******************************************************************************
 * switchTxDoneIsrCb
 */
MV_VOID switchTxDoneIsrCb(void)
{
    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
	mvOsSemSignal(G_txDoneSemId);
}

/*******************************************************************************
 * switchDrvIntEnableMiiTx
 */
MV_STATUS switchDrvIntEnableMiiTx(void)
{
    return MV_OK;
}
/*******************************************************************************
 * switchDrvIntEnableMiiRx
 */
MV_STATUS switchDrvIntEnableMiiRx(void)
{
    return MV_OK;
}

irqreturn_t  miiTxDoneIsr(int irq , void *dev_id)
{
    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
    G_txDoneIsrF(NULL);
    return IRQ_HANDLED;
}


irqreturn_t  miiRxReadyIsr(int irq , void *dev_id)
{
    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
    G_rxReadyIsrF(NULL);
    return IRQ_HANDLED;
}

irqreturn_t  sdmaTxDoneIsr(int irq , void *dev_id)
{
    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
    G_txDoneIsrF_sdma(NULL);
    return IRQ_HANDLED;
}


irqreturn_t  sdmaRxReadyIsr(int irq , void *dev_id)
{
    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
    G_rxReadyIsrF_sdma(NULL);
    return IRQ_HANDLED;
}

/*******************************************************************************
 * switchDrvIntInitMiiTx
 */
MV_STATUS switchDrvIntInitMiiTx(void (*txDoneIsrF)(void *))
{
    int intVect = (mvPpChipIsXCat2() == MV_FALSE)?(17):(INT_LVL_GBE0_TX);
    G_txDoneIsrF = txDoneIsrF;

    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );

    if( request_irq( intVect, miiTxDoneIsr,
                     (IRQF_DISABLED | IRQF_SAMPLE_RANDOM) , "miiTxDone", NULL ) )
    {
        printk( KERN_ERR "IRQ: cannot assign irq%d\n", 17 );
    }

    return MV_OK;
}

/*******************************************************************************
 * switchDrvIntInitMiiRx
 */
MV_STATUS switchDrvIntInitMiiRx(void (*rxReadyIsrF)(void *))
{
    int intVect = (mvPpChipIsXCat2() == MV_FALSE)?(16):(INT_LVL_GBE0_RX);
    G_rxReadyIsrF = rxReadyIsrF;

    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );

    if( request_irq( intVect, miiRxReadyIsr,
                     (IRQF_DISABLED | IRQF_SAMPLE_RANDOM) , "miiRxReady", NULL ) )
    {
        printk( KERN_ERR "IRQ: cannot assign irq%d\n", ETH_PORT_IRQ_NUM(1) );
        return MV_FAIL;
    }

    return  MV_OK;
}

/*******************************************************************************
 * switchDrvIntInitSdmaTx
 */
MV_STATUS switchDrvIntInitSdmaTx(void (*txDoneIsrF) (void *),
                               MV_U32 dev)
{
    DB( printk( "%s: %d called\n", __FUNCTION__, __LINE__ ) );
    G_txDoneIsrF_sdma = txDoneIsrF;

    /* connect to port interrupt line */
    if( request_irq( 113, sdmaTxDoneIsr,
                     (IRQF_DISABLED | IRQF_SAMPLE_RANDOM) , "sdmaTx", (void*)dev ) ) {
        printk( KERN_ERR "IRQ: cannot assign irq%d to dev %d\n",
                mvBoardSwitchGpioPinGet(dev), dev );
        return MV_FAIL;
    }

    return MV_OK;
}
/*******************************************************************************
 * switchDrvIntInitSdmaRx
 */
MV_STATUS switchDrvIntInitSdmaRx(void (*rxReadyIsrF) (void *),
                               MV_U32 dev)
{
    int irqNum =
            mvPpChipIsXCat2()?(INT_LVL_XCAT2_SWITCH):(mvBoardSwitchGpioPinGet(dev));

    DB( printk( "%s: %d called %d\n", __FUNCTION__, __LINE__ , irqNum) );
    G_rxReadyIsrF_sdma = rxReadyIsrF;

    /* connect to port interrupt line */
    if( request_irq( irqNum, sdmaRxReadyIsr,
                     (IRQF_DISABLED | IRQF_SAMPLE_RANDOM) , "sdmaRx", (void*)dev ) ) {
        printk( KERN_ERR "IRQ: cannot assign irq%d to dev %d\n",
                mvBoardSwitchGpioPinGet(dev), dev );
        return MV_FAIL;
    }

    return MV_OK;
}

MV_STATUS switchFwdRxPktToOs       (MV_GND_PKT_INFO *pktInfoP, MV_U32 rxQ)
{
    switchFwdRxPkt(pktInfoP, rxQ);
    return MV_OK;
}

DEFINE_SPINLOCK(switchEndRxPathSpinLock);
unsigned long switchEndRxPathSpinLockFlag;
DEFINE_SPINLOCK(switchEndTxPathSpinLock);
unsigned long switchEndTxPathSpinLockFlag;

/*******************************************************************************
 * switchEndRxPathLock
 */
MV_VOID switchEndRxPathLock()
{
// 	spin_lock_irqsave(&switchEndRxPathSpinLock, switchEndRxPathSpinLockFlag);
}

/*******************************************************************************
 * switchEndRxPathUnlock
 */
MV_VOID switchEndRxPathUnlock()
{
// 	spin_unlock_irqrestore(&switchEndRxPathSpinLock, switchEndRxPathSpinLockFlag);
}

/*******************************************************************************
 * switchEndTxPathLock
 */
MV_VOID switchEndTxPathLock()
{
// 	spin_lock_irqsave(&switchEndTxPathSpinLock, switchEndTxPathSpinLockFlag);
}

/*******************************************************************************
 * switchEndTxPathUnlock
 */
MV_VOID switchEndTxPathUnlock()
{
// 	spin_unlock_irqrestore(&switchEndTxPathSpinLock, switchEndTxPathSpinLockFlag);
}


/**************************************************************** 
* mvPortLoad --                                                 *
*   load a port network interface into Linux network core.      *
*****************************************************************/
static int mvPortLoad( int port, char *name, char *enet_addr ) 
{
    struct net_device   *dev = NULL;
    int                 ret = 0;
    switchPriv *priv = NULL;

    DB( printk( "%s: %s load - ", __FUNCTION__, name ) );

    dev = alloc_etherdev(sizeof(switchPriv));

    if( !dev ) {
        ret = -ENOMEM;
        goto error;
    }

    priv = (switchPriv *)netdev_priv(dev);
    if( !priv ) {
        DB( printk( "%s: %s falied to alloc egiga_priv (error)\n", __FUNCTION__, name ) );
        goto error;
    }
    memset( priv, 0, sizeof(switchPriv) );

    /* init device methods */
    strcpy( dev->name, name );
    dev->base_addr = 0;
    dev->open = mvPortInit;
    dev->stop = mvPortHalt;
    dev->hard_start_xmit = mvPortTx;
    dev->watchdog_timeo = 5*HZ;
    dev->tx_queue_len = PRESTERA_RXQ_LEN;
    dev->priv = priv;
    priv->port = port;
    memcpy(dev->dev_addr, enet_addr, 6);
    dev->set_mac_address = mv_set_mac_addr;
    memset( priv, 0, sizeof(switchPriv) );
    dev->mtu = MV_DEFAULT_MTU;

    if(register_netdev(dev)) {
        printk( KERN_ERR "%s: register failed\n", dev->name );
        ret = -ENODEV;
        goto error;
    }

    DB( printk( "%s: %s load ok\n", __FUNCTION__, name ) );

    return 0;

    error:
    printk( "%s: %s load failed\n", __FUNCTION__, name );
    if( dev ) unregister_netdevice(dev);
    if( dev ) kfree( dev );
    return -1;
}

/**************************************************************** 
* mvPortInit --                                                 *
*   connect to interrupt, enable traffic on physycal port device*
*****************************************************************/
static int mvPortInit( struct net_device *dev)
{
    switchPriv *priv = (switchPriv *)netdev_priv(dev);

    DB( printk( "%s: %s init - ", __FUNCTION__, dev->name ) );

    mvMuxInit(__dev_get_by_name("mux0"));

    /* in default link is down */
    netif_carrier_off( dev );

    /* Stop the TX queue - it will be enabled upon PHY status change after link-up interrupt/timer */
    netif_stop_queue( dev );

    /* enable polling on the port, must be used after netif_poll_disable */
    netif_poll_enable(dev);

    priv->port = 0;
    if(dev->name[2] == '\0')
        priv->port = dev->name[1]-'0';
    else
        priv->port += (dev->name[1]-'0')*10 + (dev->name[2]-'0');

    /*Do Linux Stuff for full enable*/
    netif_poll_enable(dev);
    netif_carrier_on( dev );
    netif_wake_queue( dev );

    if(!netif_running(dev))
    {
        printk("Device not running\n");
        printk( "%s: %s failed\n", __FUNCTION__, dev->name );
        return 1;
    }

    DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );
    return 0;
}

/**************************************************************** 
* mvPortHalt --                                                 *
*   Stop physycal port device                                   *
*****************************************************************/
static int mvPortHalt( struct net_device *dev )
{
        DB( printk( "%s: %s complete(Do nothing)\n", __FUNCTION__, dev->name ) );
        return 0;
}

/**************************************************************** 
* mvPortTx --                                                   *
*   Physycal port TX, extract port num and call SDMA TX         *
*****************************************************************/
static int mvPortTx( struct sk_buff *skb , struct net_device *dev )
{
    struct net_device *newDev;
    int port=0;
    MV_8 name[IFNAMSIZ+1];

    DB( printk( "%s: %s TX started ok\n", __FUNCTION__, dev->name ) );

    /*Extract port number from device name*/
    if(dev->name[2] == '\0')port = dev->name[1]-'0';
    else port += (dev->name[1]-'0')*10 + (dev->name[2]-'0');

    sprintf( name, "mux%d", port/PRESTERA_PORT_NUM );
    newDev = __dev_get_by_name(name);
    if( newDev == NULL )
    {
        printk("Failed to find network device\n");
        return MV_FAIL;
    }

    newDev->hard_start_xmit(skb, newDev);
    dev->stats.tx_packets++;

    DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );
    return 0;
}

/************************************************************************
* mvPortForward --                                                           *
*   Send the received packet to Linux stack                             *
*************************************************************************/
static int mvPortForward( struct sk_buff *skb, struct net_device *dev )
{
    int ret;
    dev->stats.rx_packets++;

    ret = netif_receive_skb(skb);

    DB( printk( "%s: %s complete status = %d \n", __FUNCTION__, dev->name,  ret) );
    return ret;
}

#endif /* #if defined (MV_PRESTERA_SWITCH) */
