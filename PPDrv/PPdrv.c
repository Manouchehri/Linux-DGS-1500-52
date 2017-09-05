#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>

/* Defines the license for this LKM */
MODULE_LICENSE ( "MRVL" );

// #include "../arch/arm/plat-feroceon/mv_hal/prestera/hwIf/mvHwIf.c"
//#include "../arch/arm/plat-feroceon/mv_hal/prestera/util/mvUtils.c"

#include "../arch/arm/plat-feroceon/mv_hal/twsi/mvTwsi.c"

//#define PP_FAST_PATH

#ifdef PP_FAST_PATH
#include "../arch/arm/plat-feroceon/mv_hal/eth/nfp/mvNfp.h"
#endif

static const char proc_dir_name[] = "ppdrv";
static const char proc_write_name[] = "write";
static const char proc_read_name[] = "read";
static const char proc_iwrite_name[] = "i-write";
static const char proc_iread_name[] = "i-read";
static const char proc_eewrite_name[] = "ee-write";
static const char proc_eeread_name[] = "ee-read";

#ifdef PCI_RW_FUNCTIONS
static const char proc_pciwrite_name[] = "pci-write";
static const char proc_pciread_name[] = "pci-read";
#endif

static const char proc_test_name[] = "test";

#ifdef GET_MIB_COUNTERS
static const char proc_mib_name[] = "mib";
#endif

#ifdef PP_FAST_PATH
static const char proc_fp_add_name[] = "fp-add-ip-rule";
static const char proc_fp_del_name[] = "fp-del-ip-rule";
static const char proc_fp_list_name[] = "fp-list-ip-rule";
#endif

MV_VOID   mvCtrlPwrClckSet ( MV_UNIT_ID unitId, MV_U32 index, MV_BOOL enable ) {return;}
int mvOsRand ( void ) {int rand;get_random_bytes ( &rand, sizeof ( rand ) );return rand;}
void    mvDebugPrintIpAddr ( MV_U32 ipAddr )
{
        mvOsPrintf ( "%d.%d.%d.%d", ( ( ipAddr >> 24 ) & 0xFF ), ( ( ipAddr >> 16 ) & 0xFF ),
                     ( ( ipAddr >> 8 ) & 0xFF ), ( ( ipAddr >> 0 ) & 0xFF ) );
}
miiInfCpuPortConfig()
{return;}

#ifndef PCI_RW_FUNCTIONS
MV_U32 mvPexConfigRead (MV_U32 pexIf, MV_U32 bus, MV_U32 dev, MV_U32 func, 
                        MV_U32 regOff){return 0;}
#endif

static int convStr2Num ( char* buf, int index, unsigned int *Num )
{
        char* kbuf = &buf[index];
        int  i, ii;

        Num[0] = i = ii = 0;
        while ( kbuf[i] == ' ' ) i++;
        while ( kbuf[i] != ' ' )
        {
                switch ( kbuf[i] )
                {
                        case '0'...'9':
                                Num[0]  = 16 * Num[0] + ( kbuf[i]-'0' );
                                break;
                        case 'A'...'F':
                                Num[0]  = 16 * Num[0] + ( kbuf[i]-'A' ) + 10;
                                break;
                        case 'a'...'f':
                                Num[0]  = 16 * Num[0] + ( kbuf[i]-'a' ) + 10;
                                break;
                        default:
                                break;
                }
                i++;
                ii++;
                if ( ii == 8 ) break;
        }
        return ( index + i );
}

static int convStr2NumDec ( char* buf, int index, unsigned int *Num )
{
        char* kbuf = &buf[index];
        int  i, ii;

        Num[0] = i = ii = 0;
        while ( kbuf[i] == ' ' ) i++;
        while ( kbuf[i] != ' ' )
        {
                switch ( kbuf[i] )
                {
                        case '0'...'9':
                                Num[0]  = 10 * Num[0] + ( kbuf[i]-'0' );
                                break;
                        default:
                                break;
                }
                i++;
                ii++;
                if ( ii == 8 ) break;
        }
        return ( index + i );
}

int read_setAddr_func ( struct file *file, const char __user *buffer,
                        unsigned int count, void *data )
{
        unsigned int ppdrvDev = 0;
        unsigned int ppdrvAddr= 0;
        unsigned int regVal;
        unsigned int status;
        char kbuf[8];
        int  i;

        if ( copy_from_user ( kbuf, buffer, count ) )
        {
                printk ( " failed on copy from user\n" );
                return -EFAULT;
        }

        i = convStr2Num ( kbuf, 0, &ppdrvDev );
        convStr2Num ( kbuf, i, &ppdrvAddr );

        status = mvSwitchReadReg ( ppdrvDev, ppdrvAddr, &regVal );
        if ( status != MV_OK )
        {
                printk ( "Error: Problem reading register 0x%X, status 0x%X\n",
                         ppdrvAddr, status );
                return 0;
        }

        printk ( "Read %d 0x%X - 0x%X\n", ppdrvDev, ppdrvAddr, regVal );
        return count;
}

int write_SetAddrData_func ( struct file *file, const char __user *buffer,
                             unsigned int count, void *data )
{
        unsigned int ppdrvDev = 0;
        unsigned int ppdrvAddr = 0;
        unsigned int ppdrvData = 0;
        unsigned int status;
        char kbuf[128];
        int  i= 0;

        if ( copy_from_user ( kbuf, buffer, count ) )
        {
                printk ( " failed on copy from user\n" );
                return -EFAULT;
        }

        i = convStr2Num ( kbuf, 0, &ppdrvDev );
        i = convStr2Num ( kbuf, i, &ppdrvAddr );
        convStr2Num ( kbuf, i, &ppdrvData );
        ppdrvData = ppdrvData;

        status = mvSwitchWriteReg ( ppdrvDev, ppdrvAddr, ppdrvData );
        if ( status != MV_OK )
        {
                printk ( "Error: Problem writing register 0x%X, status 0x%X\n",
                         ppdrvAddr, status );
                return 0;
        }

        printk ( "Write %d 0x%X - 0x%X\n", ppdrvDev, ppdrvAddr, ppdrvData );
        return count;
}

int write_getStatus_func ( struct file *file, const char __user *buffer,
                           unsigned int count, void *data )
{
        printk ( "Do Nothing - write performed on Device, Address and Data Set\n" );
        return count;
}
int read_data_func ( struct file *file, const char __user *buffer,
                     unsigned int count, void *data )
{
        printk ( "Do Nothing - read performed on Device and Address Set\n" );
        return count;
}

int test_read_func ( struct file *file, const char __user *buffer,
                     unsigned int count, void *data )
{
        unsigned int ppdrvAddr = 0;
        unsigned int ppdrvData = 0;
        unsigned int status, i;
        unsigned int regVal;

        ppdrvAddr = 0x50;
        status = mvSwitchReadReg ( 0, ppdrvAddr, &regVal );
        if ( status != MV_OK )
        {
                printk ( "Error: Problem reading register 0x%X, status 0x%X\n",
                         ppdrvAddr, ppdrvAddr );
                return 0;
        }

        if ( ( regVal != 0x11ab ) && ( regVal != 0xab110000 ) )
        {
                printk ( "Error: Problem reading register 0x%X, expected 0x11ab, read 0x%X\n",
                         ppdrvAddr, regVal );
                return 0;
        }


        ppdrvAddr = 0x34;
        for ( i = 0; i< 100; i++ )
        {
                status = mvSwitchWriteReg ( 0, ppdrvAddr, ppdrvData );
                if ( status != MV_OK )
                {
                        printk ( "Error: Problem writing register 0x%X, status 0x%X\n",
                                 ppdrvAddr, status );
                        return 0;
                }

                status = mvSwitchReadReg ( 0, ppdrvAddr, &regVal );
                if ( status != MV_OK )
                {
                        printk ( "Error: Problem reading register 0x%X, status 0x%X\n",
                                 ppdrvAddr, status );
                        return 0;
                }

                if ( regVal != ppdrvData )
                {
                        printk ( "Error: Problem reading register 0x%X, expected 0x%X, read 0x%X\n",
                                 ppdrvAddr,ppdrvData, regVal );
                        return 0;
                }

                ppdrvData = ppdrvData + 0x11111111;
        }

        printk ( "Test Passed.\n" );

        return count;
}

int test_write_func ( struct file *file, const char __user *buffer,
                      unsigned int count, void *data )
{
        printk ( "Do Nothing - test done on entry read\n" );
        return count;
}

#ifdef GET_MIB_COUNTERS
int mib_write_func ( struct file *file, const char __user *buffer,
                     unsigned int count, void *data )
{
        unsigned int port = 0;
        char kbuf[128];

        if ( copy_from_user ( kbuf, buffer, count ) )
        {
                printk ( " failed on copy from user\n" );
                return -EFAULT;
        }

        convStr2NumDec ( kbuf, 0, &port );

	if( port == 100 )
	{
		ethPortCounters(0);
		return count;
	}
	if( port == 101 )
	{
		ethPortCounters(1);
		return count;
	}

        mvPresteraReadPortMibCounters ( port );

        return count;
}

int mib_read_func ( struct file *file, const char __user *buffer,
                    unsigned int count, void *data )
{
        printk ( "Do Nothing - mib read done on entry read\n" );
        return count;
}
#endif

int storedFreq = 50000;
int iread_setAddr_func ( struct file *file, const char __user *buffer,
                         unsigned int count, void *data )
{
        char kbuf[8];
        int  i;
        unsigned int SlaveAddress;
        unsigned int RegAdress;
        unsigned int status;
        char wArray[4] = {0,0,0,0};
        char rArray[4] = {0,0,0,0};
        MV_TWSI_SLAVE pTwsiSlave;

        if ( copy_from_user ( kbuf, buffer, count ) )
        {
                printk ( " failed on copy from user\n" );
                return -EFAULT;
        }

        i = convStr2Num ( kbuf, 0, &SlaveAddress );
        convStr2Num ( kbuf, i, &RegAdress );


        wArray[3] =  RegAdress & 0x000000FF;
        wArray[2] = ( RegAdress & 0x0000FF00 ) >> 8;
        wArray[1] = ( RegAdress & 0x00FF0000 ) >> 16;
        wArray[0] = ( RegAdress & 0xFF000000 ) >> 24;

        pTwsiSlave.slaveAddr.address = SlaveAddress;
        pTwsiSlave.slaveAddr.type = ADDR7_BIT;
        pTwsiSlave.validOffset = MV_FALSE;
        pTwsiSlave.offset = 0;
        pTwsiSlave.moreThen256 = MV_FALSE;

        mvTwsiInit ( 0, TWSI_SPEED, mvBoardTclkGet(), &pTwsiSlave, 0 );

        status = mvTwsiWrite ( 0, &pTwsiSlave, wArray, 4 );
        if ( status != MV_OK ) goto out;

        status = mvTwsiRead ( 0, &pTwsiSlave, rArray, 4 );

out:
        printk ( "Data: 0x%02X%02X%02X%02X\n", rArray[0], rArray[1], rArray[2], rArray[3] );
        if ( status == MV_OK ) printk ( "Status: MV_OK\n" );
        else if ( status == MV_TIMEOUT ) printk ( "Status: MV_TIMEOUT\n" );
        else if ( status == MV_FAIL ) printk ( "Status: MV_FAIL\n" );
        else if ( status == MV_BAD_PARAM ) printk ( "Status: MV_BAD_PARAM\n" );
        else printk ( "Status: Undefined\n" );

        return count;
}

#define TWSI_MORE_THEN_256 MV_TRUE
int iwrite_SetAddrData_func ( struct file *file, const char __user *buffer,
                              unsigned int count, void *data )
{
        unsigned int SlaveAddress;
        unsigned int RegAdress, RegData;
        char wArray[8] = {0,0,0,0,0,0,0,0};
        MV_TWSI_SLAVE pTwsiSlave;
        char kbuf[128];
        int  i= 0;
        unsigned int status;

        if ( copy_from_user ( kbuf, buffer, count ) )
        {
                printk ( " failed on copy from user\n" );
                return -EFAULT;
        }

        i = convStr2Num ( kbuf, 0, &SlaveAddress );
        i = convStr2Num ( kbuf, i, &RegAdress );
        convStr2Num ( kbuf, i, &RegData );
        RegData = RegData/16;

        wArray[3] =  RegAdress & 0x000000FF;
        wArray[2] = ( RegAdress & 0x0000FF00 ) >> 8;
        wArray[1] = ( RegAdress & 0x00FF0000 ) >> 16;
        wArray[0] = ( RegAdress & 0xFF000000 ) >> 24;

        wArray[7] =  RegData & 0x000000FF;
        wArray[6] = ( RegData & 0x0000FF00 ) >> 8;
        wArray[5] = ( RegData & 0x00FF0000 ) >> 16;
        wArray[4] = ( RegData & 0xFF000000 ) >> 24;

        pTwsiSlave.slaveAddr.address = SlaveAddress;
        pTwsiSlave.slaveAddr.type = ADDR7_BIT;
        pTwsiSlave.validOffset = MV_FALSE;
        pTwsiSlave.offset = 0;
        pTwsiSlave.moreThen256 = MV_FALSE;

        mvTwsiInit ( 0, TWSI_SPEED, mvBoardTclkGet(), &pTwsiSlave, 0 );

        status = mvTwsiWrite ( 0, &pTwsiSlave, wArray, 8 );

        if ( status == MV_OK ) printk ( "Status: MV_OK\n" );
        else if ( status == MV_TIMEOUT ) printk ( "Status: MV_TIMEOUT\n" );
        else if ( status == MV_FAIL ) printk ( "Status: MV_FAIL\n" );
        else if ( status == MV_BAD_PARAM ) printk ( "Status: MV_BAD_PARAM\n" );
        else printk ( "Status: Undefined\n" );

        return count;
}

int iwrite_getStatus_func ( struct file *file, const char __user *buffer,
                            unsigned int count, void *data )
{
        printk ( "Do Nothing - write performed on Device, Address and Data Set\n" );
        return count;
}
int iread_data_func ( struct file *file, const char __user *buffer,
                      unsigned int count, void *data )
{
        printk ( "Do Nothing - read performed on Device and Address Set\n" );
        return count;
}

int eeread_setAddr_func ( struct file *file, const char __user *buffer,
                         unsigned int count, void *data )
{
        char kbuf[8];
        int  i;
        unsigned int SlaveAddress;
        unsigned int size;
        unsigned int status;
        char wArray[4] = {0,0,0,0};
        char rArray[4] = {0,0,0,0};
        MV_U8 readRegVal = 0;
        MV_TWSI_SLAVE twsiSlave;

        if ( copy_from_user ( kbuf, buffer, count ) )
        {
                printk ( " failed on copy from user\n" );
                return -EFAULT;
        }

        i = convStr2Num ( kbuf, 0, &SlaveAddress );
        convStr2Num ( kbuf, i, &size );

        twsiSlave.slaveAddr.address = SlaveAddress;
        twsiSlave.slaveAddr.type = ADDR7_BIT;
        twsiSlave.moreThen256 = MV_TRUE;

        mvTwsiInit ( 0, TWSI_SPEED*2, mvBoardTclkGet(), &twsiSlave, 0 );

        printf("\n"); 
        for (i = 0; i < (size); i++)
        {
                twsiSlave.offset = i;

                if( MV_OK != mvTwsiRead (0, &twsiSlave, &readRegVal, 1) )
                {
                        printf("TWSI: Read fail offset %d\n",twsiSlave.offset);
                        return MV_ERROR;
                }

                printf("%02X ",readRegVal); 
        }
        printf("\n%d bytes\n",size); 

out:
//        printk ( "Data: 0x%02X%02X%02X%02X\n", rArray[0], rArray[1], rArray[2], rArray[3] );
        if ( status == MV_OK ) printk ( "Status: MV_OK\n" );
        else if ( status == MV_TIMEOUT ) printk ( "Status: MV_TIMEOUT\n" );
        else if ( status == MV_FAIL ) printk ( "Status: MV_FAIL\n" );
        else if ( status == MV_BAD_PARAM ) printk ( "Status: MV_BAD_PARAM\n" );
        else printk ( "Status: Undefined\n" );

        return count;
}

int eewrite_SetAddrData_func ( struct file *file, const char __user *buffer,
                              unsigned int count, void *data )
{
        unsigned int SlaveAddress;
        unsigned int RegAdress, RegData;
        char wArray[8] = {0,0,0,0,0,0,0,0};
        MV_TWSI_SLAVE twsiSlave;
        char kbuf[128];
        int  i= 0;
        unsigned int status;

        if ( copy_from_user ( kbuf, buffer, count ) )
        {
                printk ( " failed on copy from user\n" );
                return -EFAULT;
        }

        i = convStr2Num ( kbuf, 0, &SlaveAddress );
        i = convStr2Num ( kbuf, i, &RegAdress );
        convStr2Num ( kbuf, i, &RegData );
        RegData = RegData/16;

        wArray[3] =  RegAdress & 0x000000FF;
        wArray[2] = ( RegAdress & 0x0000FF00 ) >> 8;
        wArray[1] = ( RegAdress & 0x00FF0000 ) >> 16;
        wArray[0] = ( RegAdress & 0xFF000000 ) >> 24;

        wArray[7] =  RegData & 0x000000FF;
        wArray[6] = ( RegData & 0x0000FF00 ) >> 8;
        wArray[5] = ( RegData & 0x00FF0000 ) >> 16;
        wArray[4] = ( RegData & 0xFF000000 ) >> 24;

        twsiSlave.slaveAddr.address = SlaveAddress;
        twsiSlave.slaveAddr.type = ADDR7_BIT;
        twsiSlave.validOffset = MV_TRUE;
        twsiSlave.moreThen256 = TWSI_MORE_THEN_256;

        twsiSlave.offset = RegAdress;
        
        printf("TWSI: Write dev %X offset %X data %X\n",
		twsiSlave.slaveAddr.address,twsiSlave.offset, RegData);

        mvTwsiInit ( 0, TWSI_SPEED*2, mvBoardTclkGet(), &twsiSlave, 0 );

        if( MV_OK != (status=mvTwsiWrite (0, &twsiSlave, &RegData, 4)) )
        {
                printf("TWSI: Write fail offset %d\n",twsiSlave.offset);
                return MV_ERROR;
        }

        if ( status == MV_OK ) printk ( "Status: MV_OK\n" );
        else if ( status == MV_TIMEOUT ) printk ( "Status: MV_TIMEOUT\n" );
        else if ( status == MV_FAIL ) printk ( "Status: MV_FAIL\n" );
        else if ( status == MV_BAD_PARAM ) printk ( "Status: MV_BAD_PARAM\n" );
        else printk ( "Status: Undefined\n" );

        return count;
}

int eewrite_getStatus_func ( struct file *file, const char __user *buffer,
                            unsigned int count, void *data )
{
        printk ( "Do Nothing - write performed on Device, Address and Data Set\n" );
        return count;
}
int eeread_data_func ( struct file *file, const char __user *buffer,
                      unsigned int count, void *data )
{
        printk ( "Do Nothing - read performed on Device and Address Set\n" );
        return count;
}

#ifdef PCI_RW_FUNCTIONS
int pciread_setAddr_func ( struct file *file, const char __user *buffer,
                         unsigned int count, void *data )
{
        char kbuf[8];
        int  i = 0;
        unsigned int pexIf;
        unsigned int bus;
        unsigned int dev;
        unsigned int addr;
        unsigned int pexData;

        if ( copy_from_user ( kbuf, buffer, count ) )
        {
                printk ( " failed on copy from user\n" );
                return -EFAULT;
        }

        i = convStr2Num ( kbuf, i, &pexIf );
        i = convStr2Num ( kbuf, i, &bus );
        i = convStr2Num ( kbuf, i, &dev );
        i = convStr2Num ( kbuf, i, &addr );

printk("+++ mvPexConfigWrite %d %d %d %d\n", pexIf, bus, dev, addr);

        pexData = mvPexConfigRead(pexIf, bus, dev, 0, addr );
        printk("Pex Header reg %d = 0x%X\n", addr, pexData);

        return count;
}

int pciwrite_SetAddrData_func ( struct file *file, const char __user *buffer,
                              unsigned int count, void *data )
{
        char kbuf[128];
        int  i= 0;
        unsigned int pexIf;
        unsigned int bus;
        unsigned int dev;
        unsigned int addr;
        unsigned int pexData;

        if ( copy_from_user ( kbuf, buffer, count ) )
        {
                printk ( " failed on copy from user\n" );
                return -EFAULT;
        }

        i = convStr2Num ( kbuf, i, &pexIf );
        i = convStr2Num ( kbuf, i, &bus );
        i = convStr2Num ( kbuf, i, &dev );
        i = convStr2Num ( kbuf, i, &addr );
        i = convStr2Num ( kbuf, i, &pexData );
printk("+++ mvPexConfigWrite %d %d %d %d %d\n", pexIf, bus, dev, addr, pexData);

        mvPexConfigWrite(pexIf, bus, dev, 0, addr, pexData );
        printk("Pex Header reg %d = 0x%X\n", addr, pexData);

        return count;
}

int pciwrite_getStatus_func ( struct file *file, const char __user *buffer,
                            unsigned int count, void *data )
{
        printk ( "Do Nothing - write performed on Device, Address and Data Set\n" );
        return count;
}
int pciread_data_func ( struct file *file, const char __user *buffer,
                      unsigned int count, void *data , int is_user )
{
        printk ( "Do Nothing - read performed on Device and Address Set\n" );
        return count;
}
#endif

#ifdef PP_FAST_PATH
int fp_add_func ( struct file *file, const char __user *buffer,
                  unsigned int count, void *data )
{
        unsigned int port = 0;
        unsigned int dip, sip, commandCode;
        char kbuf[128], cmd[10];
        int  i= 0;
        MV_FP_RULE SetRule;
        MV_STATUS status;

        if ( copy_from_user ( kbuf, buffer, count ) )
        {
                printk ( " failed on copy from user\n" );
                return -EFAULT;
        }

        i = convStr2NumDec ( kbuf, i, &commandCode );
        i = convStr2NumDec ( kbuf, i, &port );
        i = convStr2Num ( kbuf, i, &sip );
        convStr2Num ( kbuf, i, &dip );

        memzero ( &SetRule, sizeof ( MV_FP_RULE ) );

        if ( commandCode == 0 )
        {
                SetRule.mgmtInfo.actionType = MV_FP_DROP_CMD;
                SetRule.routingInfo.outIfIndex = PRESTERA_FP_DROP_INTERFACE;
                printk ( "cmd = DROP" );
        }
        else if ( commandCode == 1 )
        {
                SetRule.mgmtInfo.actionType = MV_FP_ROUTE_CMD;
                SetRule.routingInfo.outIfIndex = port;
                printk ( "cmd = MIRR" );
        }
        else
        {
                printk ( "Undefined command code. No rule added.\n" );
                return count;
        }

//                SetRule.mgmtInfo.old_count
//                SetRule.mgmtInfo.new_count
        SetRule.mgmtInfo.ruleType = MV_FP_STATIC_RULE;
//                SetRule.mgmtInfo.dnat_aware_refcnt
//                SetRule.mgmtInfo.snat_aware_refcnt

        SetRule.routingInfo.dstIp = dip;
        SetRule.routingInfo.srcIp = sip;
//                SetRule.routingInfo.aware_flags
        /*                SetRule.routingInfo.dstMac[0] = 0x00;
                        SetRule.routingInfo.dstMac[1] = 0x00;
                        SetRule.routingInfo.dstMac[2] = 0x00;
                        SetRule.routingInfo.dstMac[3] = 0x00;
                        SetRule.routingInfo.dstMac[4] = 0x00;
                        SetRule.routingInfo.dstMac[5] = 0x00;
                        SetRule.routingInfo.srcMac[0] = 0x00;
                        SetRule.routingInfo.srcMac[1] = 0x00;
                        SetRule.routingInfo.srcMac[2] = 0x00;
                        SetRule.routingInfo.srcMac[3] = 0x00;
                        SetRule.routingInfo.srcMac[4] = 0x00;
                        SetRule.routingInfo.srcMac[5] = 0x00;*/
//                SetRule.routingInfo.inIfIndex = dev->ifindex;
        printk ( "ifindex=%d srcIP=0x%X dstIP=0x%X\n", SetRule.routingInfo.outIfIndex,
                 SetRule.routingInfo.srcIp, SetRule.routingInfo.dstIp );

//                ruleDbSize = 1024;
        status = mvFpRuleSet ( &SetRule );
        if ( status != MV_OK ) printk ( "mvFpRuleSet failed with code 0x%X",status );
        else printk ( "Rule Ok\n" );

        return count;
}

int fp_del_func ( struct file *file, const char __user *buffer,
                  unsigned int count, void *data )
{
        unsigned int port = 0;
        unsigned int dip, sip, commandCode;
        char kbuf[128], cmd[10];
        int  i= 0;
        MV_FP_RULE SetRule;
        MV_STATUS status;

        if ( copy_from_user ( kbuf, buffer, count ) )
        {
                printk ( " failed on copy from user\n" );
                return -EFAULT;
        }

        i = convStr2NumDec ( kbuf, i, &commandCode );
        i = convStr2NumDec ( kbuf, i, &port );
        i = convStr2Num ( kbuf, i, &sip );
        convStr2Num ( kbuf, i, &dip );

        memzero ( &SetRule, sizeof ( MV_FP_RULE ) );

        if ( commandCode == 0 )
        {
                SetRule.mgmtInfo.actionType = MV_FP_DROP_CMD;
                SetRule.routingInfo.outIfIndex = PRESTERA_FP_DROP_INTERFACE;
                printk ( "cmd = DROP\n" );
        }
        else if ( commandCode == 1 )
        {
                SetRule.mgmtInfo.actionType = MV_FP_ROUTE_CMD;
                SetRule.routingInfo.outIfIndex = port;
                printk ( "cmd = MIRR\n" );
        }
        else
        {
                printk ( "Undefined command code. No rule added.\n" );
                return count;
        }

        SetRule.mgmtInfo.ruleType = MV_FP_STATIC_RULE;
        SetRule.routingInfo.dstIp = dip;
        SetRule.routingInfo.srcIp = sip;

        status = mvFpRuleDelete ( &SetRule );
        if ( status != MV_OK ) printk ( "mvFpRuleDelete failed with code 0x%X",status );
        else printk ( "Rule Ok\n" );

        return count;
}

int fp_list_func ( struct file *file, const char __user *buffer,
                   unsigned int count, void *data )
{
        mvFpRuleDbPrint();
        return count;
}
int fp_do_nothing_func ( struct file *file, const char __user *buffer,
                         unsigned int count, void *data, int is_user )
{
        printk ( "Do Nothing - FP function\n" );
        return count;
}
#endif

/* Init function called on module entry */
struct proc_dir_entry *procDir;
struct proc_dir_entry *readEnt;
struct proc_dir_entry *writeEnt;
struct proc_dir_entry *ireadEnt;
struct proc_dir_entry *iwriteEnt;
struct proc_dir_entry *eereadEnt;
struct proc_dir_entry *eewriteEnt;

#ifdef PCI_RW_FUNCTIONS
struct proc_dir_entry *pcireadEnt;
struct proc_dir_entry *pciwriteEnt;
#endif

struct proc_dir_entry *testEnt;

#ifdef GET_MIB_COUNTERS
struct proc_dir_entry *mibEnt;
#endif

#ifdef PP_FAST_PATH
struct proc_dir_entry *fpAdd;
struct proc_dir_entry *fpDel;
struct proc_dir_entry *fpList;
#endif

int PPdrv_module_init ( void )
{
        procDir = proc_mkdir ( proc_dir_name, NULL );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "ppdrv create FAILED\n" );
        }

        readEnt = create_proc_entry ( proc_read_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "read create FAILED\n" );
        }

        writeEnt = create_proc_entry ( proc_write_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "write create FAILED\n" );
        }

        ireadEnt = create_proc_entry ( proc_iread_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "i-read create FAILED\n" );
        }

        iwriteEnt = create_proc_entry ( proc_iwrite_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "i-write create FAILED\n" );
        }

        eereadEnt = create_proc_entry ( proc_eeread_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "ee-read create FAILED\n" );
        }

        eewriteEnt = create_proc_entry ( proc_eewrite_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "ee-write create FAILED\n" );
        }

#ifdef PCI_RW_FUNCTIONS
        pcireadEnt = create_proc_entry ( proc_pciread_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "pci-read create FAILED\n" );
        }

        pciwriteEnt = create_proc_entry ( proc_pciwrite_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "pci-write create FAILED\n" );
        }
#endif

        testEnt = create_proc_entry ( proc_test_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "write create FAILED\n" );
        }

#ifdef GET_MIB_COUNTERS
        mibEnt = create_proc_entry ( proc_mib_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "FP Add create FAILED\n" );
        }
#endif

#ifdef PP_FAST_PATH
        fpAdd = create_proc_entry ( proc_fp_add_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "FP Del create FAILED\n" );
        }

        fpDel = create_proc_entry ( proc_fp_del_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "FP list create FAILED\n" );
        }

        fpList = create_proc_entry ( proc_fp_list_name,S_IFREG|S_IRUSR|S_IWUSR, procDir );
        if ( procDir == NULL )
        {
                printk ( KERN_INFO "MIB create FAILED\n" );
        }

        fpAdd->write_proc = fp_add_func;
        fpAdd->read_proc = fp_do_nothing_func;
        fpDel->write_proc = fp_del_func;
        fpDel->read_proc = fp_do_nothing_func;
        fpList->write_proc = fp_do_nothing_func;
        fpList->read_proc = fp_list_func;

        fpAdd->owner = THIS_MODULE;
        fpDel->owner = THIS_MODULE;
        fpList->owner = THIS_MODULE;
#endif

        readEnt->read_proc = read_data_func;
        readEnt->write_proc = read_setAddr_func;

        writeEnt->write_proc = write_SetAddrData_func;
        writeEnt->read_proc = write_getStatus_func;

        ireadEnt->read_proc = iread_data_func;
        ireadEnt->write_proc = iread_setAddr_func;
        iwriteEnt->write_proc = iwrite_SetAddrData_func;
        iwriteEnt->read_proc = iwrite_getStatus_func;

        eereadEnt->read_proc = eeread_data_func;
        eereadEnt->write_proc = eeread_setAddr_func;
        eewriteEnt->write_proc = eewrite_SetAddrData_func;
        eewriteEnt->read_proc = eewrite_getStatus_func;

#ifdef PCI_RW_FUNCTIONS
        pcireadEnt->read_proc = pciread_data_func;
        pcireadEnt->write_proc = pciread_setAddr_func;

        pciwriteEnt->write_proc = pciwrite_SetAddrData_func;
        pciwriteEnt->read_proc = pciwrite_getStatus_func;
#endif

        testEnt->write_proc = test_write_func;
        testEnt->read_proc = test_read_func;

#ifdef GET_MIB_COUNTERS
        mibEnt->write_proc = mib_write_func;
        mibEnt->read_proc = mib_read_func;
#endif

        readEnt->owner = THIS_MODULE;
        writeEnt->owner = THIS_MODULE;
        ireadEnt->owner = THIS_MODULE;
        iwriteEnt->owner = THIS_MODULE;
        eereadEnt->owner = THIS_MODULE;
        eewriteEnt->owner = THIS_MODULE;

#ifdef PCI_RW_FUNCTIONS
        pcireadEnt->owner = THIS_MODULE;
        pciwriteEnt->owner = THIS_MODULE;
#endif
        testEnt->owner = THIS_MODULE;
#ifdef GET_MIB_COUNTERS
        mibEnt->owner = THIS_MODULE;
#endif

        printk ( KERN_INFO "PPdrv_module_init called.  Module is now loaded.\n" );
        return 0;
}

/* Cleanup function called on module exit */
void PPdrv_module_cleanup ( void )
{
        remove_proc_entry ( proc_test_name, procDir );
        remove_proc_entry ( proc_read_name, procDir );
        remove_proc_entry ( proc_write_name, procDir );
        remove_proc_entry ( proc_iread_name, procDir );
        remove_proc_entry ( proc_iwrite_name, procDir );
        remove_proc_entry ( proc_eeread_name, procDir );
        remove_proc_entry ( proc_eewrite_name, procDir );

#ifdef PCI_RW_FUNCTIONS
        remove_proc_entry ( proc_pciread_name, procDir );
        remove_proc_entry ( proc_pciwrite_name, procDir );
#endif

#ifdef GET_MIB_COUNTERS
        remove_proc_entry ( proc_mib_name, procDir );
#endif

#ifdef PP_FAST_PATH
        remove_proc_entry ( proc_fp_add_name, procDir );
        remove_proc_entry ( proc_fp_del_name, procDir );
        remove_proc_entry ( proc_fp_list_name, procDir );
#endif

        remove_proc_entry ( proc_dir_name, NULL );

        printk ( KERN_INFO "PPdrv_module_cleanup called.  Module is now unloaded.\n" );
        return;
}

/* Declare entry and exit functions */
module_init ( PPdrv_module_init );
module_exit ( PPdrv_module_cleanup );
