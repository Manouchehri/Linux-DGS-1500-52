#*******************************************************************************
# Marvell GPL License Option
#
# If you received this File from Marvell, you may opt to use, redistribute and/or 
# modify this File in accordance with the terms and conditions of the General 
# Public License Version 2, June 1991 (the "GPL License"), a copy of which is 
# available along with the File in the license.txt file or by writing to the Free 
# Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or 
# on the worldwide web at http://www.gnu.org/licenses/gpl.txt. 
#
# THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY 
# DISCLAIMED.  The GPL License provides additional details about this warranty 
# disclaimer.
#*******************************************************************************/
include 	  $(TOPDIR)/arch/arm/mach-feroceon-kw/config/mvRules.mk

# Objects list
COMMON_OBJS	= $(COMMON_DIR)/mvDebug.o $(COMMON_DIR)/mvCommon.o $(COMMON_DIR)/mvStack.o

OSSERVICES_OBJS	= $(OSSERV_DIR)/mvOs.o

HAL_OBJS	=  	  $(HAL_DRAM_SPD_DIR)/mvSpd.o	\
                  $(HAL_CNTMR_DIR)/mvCntmr.o $(HAL_TWSI_DIR)/mvTwsi.o         	\
                  $(HAL_UART_DIR)/mvUart.o $(HAL_GPP_DIR)/mvGpp.o               \
                  $(HAL_DRAM_DIR)/mvDramIf.o  					\
		  		  $(HAL_ETHPHY_DIR)/mvEthPhy.o $(HAL_ETH_DIR)/mvEth.o 		\
		  		  $(HAL_ETH_DIR)/mvEthDebug.o 

KW_FAM_OBJS	= $(BOARD_ENV_DIR)/mvBoardEnvSpec.o $(SOC_ENV_DIR)/mvCtrlEnvLib.o	\
		  $(BOARD_ENV_DIR)/mvBoardEnvLib.o $(SOC_ENV_DIR)/mvCtrlEnvAddrDec.o 	\
		  $(SOC_SYS_DIR)/mvAhbToMbus.o $(SOC_SYS_DIR)/mvCpuIf.o 		\
		  $(SOC_CPU_DIR)/mvCpu.o 			\
		  $(SOC_DEVICE_DIR)/mvDevice.o	$(SOC_SYS_DIR)/mvSysDram.o	

ifeq ($(CONFIG_MV_INCLUDE_PEX), y)
	KW_FAM_OBJS	+= $(SOC_SYS_DIR)/mvSysPex.o
endif

LSP_OBJS        = $(LSP_DIR)/core.o $(LSP_DIR)/irq.o $(LSP_DIR)/time.o  	\
                  $(LSP_DIR)/leds.o $(LSP_DIR)/sysmap.o  \
		  		  $(LSP_DIR)/export.o $(LSP_DIR)/btns_device.o
		  
LSP_TESTS_OBJS  = $(LSP_TESTS_DIR_BASIC)/mv_timer_tests/timer_test_driver.o \
                  $(LSP_TESTS_DIR_BASIC)/mv_memory_tests/memory_test_driver.o \
                  $(LSP_TESTS_DIR_BASIC)/mv_smi_tests/smi_test_driver.o \
                  $(LSP_TESTS_DIR_BASIC)/mv_cache_tests/cache_test_driver.o \
                  $(LSP_TESTS_DIR_BASIC)/mv_twsi_tests/twsi_test_driver.o \
                  $(LSP_TESTS_DIR_BASIC)/mv_led_tests/led_test_driver.o \
                  $(LSP_TESTS_DIR_BASIC)/mv_mmu_tests/mmu_test_driver.o \
                  $(LSP_TESTS_DIR_BASIC)/mv_oob_tests/oob_test_driver.o \
                  $(LSP_TESTS_DIR_BASIC)/mv_pex_tests/pex_test_driver.o \
                  $(LSP_TESTS_DIR_BASIC)/mv_nand_tests/nand_test_driver.o \
                  $(LSP_TESTS_DIR_BASIC)/mv_spi_tests/spi_test_driver.o \
                  $(LSP_TESTS_DIR_BASIC)/mv_xor_tests/xor_test_driver.o \
                  $(LSP_TESTS_DIR_BASIC)/mv_usb_tests/usb_test_driver.o  \
                  $(LSP_TESTS_DIR_CMPLX)/xcat_mv_mem_tests/complex_mem_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/xcat_mv_UnAligned_mem_tests/CmplxMemTest_FreeMemWriteRead.o \
                  $(LSP_TESTS_DIR_CMPLX)/xcat_mv_UnAligned_mem_tests/complex_UnAligned_mem_test_driver.o \
                  $(LSP_TESTS_DIR)/mvTestsCpuCtrlReg.o \
                  $(LSP_TESTS_DIR_CMPLX)/xcat_mv_UnAligned_mem_tests/CmplxMemTest_FreeMemWriteRead_U.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_mmu_tests/cmplx_mmu_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_cache_tests/cmplx_cache_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_interrupts_tests/cmplx_interrupts_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_spi_tests/cmplx_spi_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_smi_tests/cmplx_smi_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_twsi_tests/cmplx_twsi_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_oob_tests/cmplx_oob_test_driver.o  \
                  $(LSP_TESTS_DIR_CMPLX)/mv_uart_tests/cmplx_uart_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_pssApi_tests/cmplx_pssApi_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_L2cache_tests/cmplx_L2cache_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_pex_tests/cmplx_pex_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_gpio_tests/cmplx_gpio_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_usb_tests/cmplx_usb_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_timer_tests/cmplx_timer_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_crypto_tests/cmplx_crypto_test_driver.o \
                  $(LSP_TESTS_DIR_CMPLX)/mv_network_tests/cmplx_network_scatter_gather_test_driver.o

ifeq ($(CONFIG_MV_PRESTERA_SWITCH), y)
HAL_OBJS     += $(HAL_PRESTERA_DIR)/mvPrestera.o $(HAL_PRESTERA_DIR)/mvPresteraEEPROM.o \
		$(HAL_MII_DIR)/mii.o \
		$(LSP_DIR)/pss/pssBspApis.o \
		$(COMMON_DIR)/bitOps.o  \
		$(COMMON_DIR)/mvHookReg.o  \
		$(COMMON_DIR)/mvWriteLog.o  \
		$(COMMON_DIR)/pool/mvGenSyncPool.o \
		$(COMMON_DIR)/pool/mvGenBuffPool.o \
		$(COMMON_DIR)/pool/mvGenPool.o \
		$(COMMON_DIR)/gnd/mvGnd.o  \
		$(COMMON_DIR)/mvNetDrvCommon.o  \
		$(COMMON_DIR)/gnd/mvGndReg.o \
  		$(PLAT_DRIVERS)/mv_prestera_net/mv_prestera_mux.o 
ifeq ($(CONFIG_MV_DRAGONITE_SWITCH), y)
HAL_OBJS     += $(HAL_DRAGONITE_DIR)/mvDragonite.o
endif
HAL_OBJS     += $(LSP_DIR)/pss/pssBspApisLinux.o
endif

obj-y   				:=  feroceon.o
feroceon-objs  				:=$(LSP_OBJS) $(COMMON_OBJS) $(OSSERVICES_OBJS) \
  					  $(HAL_OBJS) 	\
                      $(KW_FAM_OBJS) #$(LSP_TESTS_OBJS)

feroceon-$(CONFIG_MV_INCLUDE_SDIO) 	+= $(SOC_SYS_DIR)/mvSysSdmmc.o
feroceon-$(CONFIG_MV_INCLUDE_XOR) 	+= $(HAL_XOR_DIR)/mvXor.o $(SOC_SYS_DIR)/mvSysXor.o
feroceon-$(CONFIG_MV_INCLUDE_PEX) 	+= $(HAL_PEX_DIR)/mvPex.o
feroceon-$(CONFIG_MV_INCLUDE_USB) 	+= $(HAL_USB_DIR)/mvUsb.o $(SOC_SYS_DIR)/mvSysUsb.o
feroceon-$(CONFIG_MV_INCLUDE_GIG_ETH)   += $(SOC_SYS_DIR)/mvSysGbe.o
feroceon-$(CONFIG_MV_ETH_NFP)           += $(HAL_ETH_NFP_DIR)/mvNfp.o 
feroceon-$(CONFIG_MV_ETH_NFP_NAT_SUPPORT) += $(HAL_ETH_NFP_DIR)/mvNfpNat.o
feroceon-$(CONFIG_MV_ETH_NFP_FDB_SUPPORT) += $(HAL_ETH_NFP_DIR)/mvNfpFdb.o  
feroceon-$(CONFIG_MV_INCLUDE_CESA) 	+= $(HAL_CESA_DIR)/mvCesa.o $(HAL_CESA_DIR)/mvCesaDebug.o 	\
                        		   $(HAL_CESA_DIR)/mvMD5.o $(HAL_CESA_DIR)/mvSHA1.o 		\
					   $(HAL_CESA_AES_DIR)/mvAesAlg.o $(HAL_CESA_AES_DIR)/mvAesApi.o\
					   $(SOC_SYS_DIR)/mvSysCesa.o $(HAL_CESA_DIR)/mvCesaTest.o
feroceon-$(CONFIG_MV_INCLUDE_INTEG_SATA)+= $(SOC_SYS_DIR)/mvSysSata.o $(HAL_SATA_DIR)/mvSataSoc.o
feroceon-$(CONFIG_MV_INCLUDE_SPI) 	+= $(HAL_SPI_DIR)/mvSpi.o $(HAL_SPI_DIR)/mvSpiCmnd.o 		\
                         		   $(HAL_SFLASH_DIR)/mvSFlash.o
feroceon-$(CONFIG_MV_INCLUDE_TS) 	+= $(HAL_TS_DIR)/mvTsu.o $(SOC_SYS_DIR)/mvSysTs.o
feroceon-$(CONFIG_MV_CPU_PERF_CNTRS)    += $(HAL_CPU_DIR)/mvCpuCntrs.o

ifeq ($(CONFIG_MV_INCLUDE_PEX), y)
feroceon-$(CONFIG_PCIE_VIRTUAL_BRIDGE_SUPPORT) += $(HAL_PEX_DIR)/mvVrtBrgPex.o
endif

feroceon-$(CONFIG_MV_INCLUDE_SAGE) 	+= $(HAL_PP_DIR)/hwIf/gtHwIf.o $(HAL_PP_DIR)/sdma/gtSdma.o	\
 					   $(HAL_PP_DIR)/sdma/gtTxHandling.o $(HAL_PP_DIR)/sdma/gtBridge.o \
 					   $(HAL_PP_DIR)/sdma/gtNetIf.o $(HAL_PP_DIR)/vlan/gtVlan.o	\
 					   $(HAL_PP_DIR)/util/gtUtils.o $(HAL_PP_DIR)/util/gtNetTransmit.o \
 					   $(HAL_PP_DIR)/util/gtNetReceive.o \
 					   $(HAL_PP_DIR)/../mv_drivers_lsp/mv_prestera_net/mv_prestera_switch.o

# drivers part
obj-$(CONFIG_MV_USE_XOR_ENGINE) 	+= ../plat-feroceon/mv_drivers_lsp/mv_xor/
obj-$(CONFIG_MV_CESA) 			+= ../plat-feroceon/mv_drivers_lsp/mv_cesa/
feroceon-$(CONFIG_MV_ETH_NFP) 	        += ../plat-feroceon/mv_drivers_lsp/mv_network/nfp_mgr/mv_nfp_mgr.o
obj-$(CONFIG_MV_TSU) 			+= ../plat-feroceon/mv_drivers_lsp/mv_tsu/
obj-y					+= ../plat-feroceon/mv_drivers_lsp/mv_btns/
# The rest of the drivers are compiled through the driver dir directly.


# LSP part
feroceon-$(CONFIG_MV_INCLUDE_USB)    	+= $(LSP_DIR)/usb.o
ifeq ($(CONFIG_MV_INCLUDE_PEX), y)
	feroceon-$(CONFIG_PCI)  		+= $(LSP_DIR)/pci.o
endif
feroceon-$(CONFIG_FEROCEON_PROC) 	+= $(LSP_DIR)/proc.o
feroceon-$(CONFIG_MV_DBG_TRACE) 	+= $(LSP_DIR)/dbg-trace.o
