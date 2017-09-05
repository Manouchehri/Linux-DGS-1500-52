# This flags will be used only by the Marvell arch files compilation.

###################################################################################################
# General definitions
###################################################################################################
CPU_ARCH    = ARM
CHIP        = 88F6281
VENDOR      = Marvell
ifeq ($(CONFIG_CPU_BIG_ENDIAN),y)
ENDIAN      = BE
else
ENDIAN      = LE
endif

###################################################################################################
# directory structure
###################################################################################################
# Main directory structure
SRC_PATH          = .
PLAT_PATH	  = ../plat-feroceon
PLAT_DRIVERS	  = $(PLAT_PATH)/mv_drivers_lsp
HAL_DIR           = $(PLAT_PATH)/mv_hal
COMMON_DIR        = $(PLAT_PATH)/common
OSSERV_DIR        = $(PLAT_PATH)/linux_oss
LSP_DIR           = $(SRC_PATH)
CONFIG_DIR        = $(LSP_DIR)/config
LSP_TESTS_DIR     = lsp_tests
LSP_TESTS_DIR_BASIC = $(LSP_TESTS_DIR)/Basic
LSP_TESTS_DIR_CMPLX = $(LSP_TESTS_DIR)/Complex

# HALs
HAL_ETHPHY_DIR    = $(HAL_DIR)/eth-phy
HAL_FLASH_DIR     = $(HAL_DIR)/flash
HAL_SFLASH_DIR    = $(HAL_DIR)/sflash
HAL_CNTMR_DIR     = $(HAL_DIR)/cntmr
HAL_DRAM_DIR      = $(HAL_DIR)/ddr2
HAL_DRAM_SPD_DIR  = $(HAL_DRAM_DIR)/spd
HAL_DRAM_ARCH_DIR = $(HAL_DRAM_DIR)/Arch$(CPU_ARCH)
HAL_GPP_DIR       = $(HAL_DIR)/gpp
HAL_TWSI_DIR      = $(HAL_DIR)/twsi
HAL_TWSI_ARCH_DIR = $(SOC_TWSI_DIR)/Arch$(CPU_ARCH)
HAL_UART_DIR      = $(HAL_DIR)/uart
HAL_ETH_DIR       = $(HAL_DIR)/eth/gbe
HAL_ETH_P_DIR     = $(HAL_DIR)/eth
HAL_ETH_NFP_DIR	  = $(HAL_DIR)/eth/nfp
HAL_CPU_DIR       = $(HAL_DIR)/cpu

ifeq ($(CONFIG_MV_INCLUDE_PEX),y)
HAL_PEX_DIR       = $(HAL_DIR)/pex
endif

ifeq ($(CONFIG_MV_INCLUDE_USB),y)
HAL_USB_DIR       = $(HAL_DIR)/usb
endif
ifeq ($(CONFIG_MV_INCLUDE_CESA),y)
HAL_CESA_DIR	  = $(HAL_DIR)/cesa
HAL_CESA_AES_DIR  = $(HAL_DIR)/cesa/AES
endif
ifeq ($(CONFIG_MV_INCLUDE_XOR),y)
HAL_XOR_DIR       = $(HAL_DIR)/xor
endif
ifeq ($(CONFIG_MV_INCLUDE_SPI),y)
HAL_SPI_DIR       = $(HAL_DIR)/spi
endif

HAL_PRESTERA_DIR = $(HAL_DIR)/prestera
HAL_DRAGONITE_DIR = $(HAL_DIR)/dragonite
HAL_MII_DIR       = $(HAL_DIR)/mii


# Environment components
KW_FAM_DIR	    = $(LSP_DIR)/kw_family
SOC_DEVICE_DIR      = $(KW_FAM_DIR)/device
SOC_CPU_DIR         = $(KW_FAM_DIR)/cpu
BOARD_ENV_DIR       = $(KW_FAM_DIR)/boardEnv
SOC_ENV_DIR         = $(KW_FAM_DIR)/ctrlEnv
SOC_SYS_DIR	    = $(KW_FAM_DIR)/ctrlEnv/sys

#####################################################################################################
# Include path
###################################################################################################

LSP_PATH_I      = $(TOPDIR)/arch/arm/mach-feroceon-kw
PLAT_PATH_I		= $(TOPDIR)/arch/arm/plat-feroceon

HAL_PATH        = -I$(PLAT_PATH_I)/$(HAL_DIR) -I$(PLAT_PATH_I)/$(HAL_SATA_DIR) \
 		  -I$(PLAT_PATH_I)/$(HAL_PRESTERA_DIR) -I$(PLAT_PATH_I)/$(HAL_DRAGONITE_DIR) \
		  -I$(PLAT_PATH_I)/$(HAL_MII_DIR) -I$(LSP_PATH_I)/pss \
		  -I$(PLAT_PATH_I)/$(HAL_PRESTERA_DIR)/vlan \
		  -I$(PLAT_PATH_I)/$(HAL_PRESTERA_DIR)/sdma \
		  -I$(PLAT_PATH_I)/$(HAL_ETH_P_DIR) -I$(PLAT_PATH_I)/include \
		  -I$(PLAT_PATH_I)/$(HAL_DIR)/eth/gbe	\
		  -I$(PLAT_PATH_I)/$(HAL_DIR)/pex -I$(PLAT_PATH_I)/$(HAL_DIR)/xor	\
 		  -I$(PLAT_PATH_I)/$(HAL_DIR)/eth-phy


KW_FAM_PATH  = 	  -I$(LSP_PATH_I)/$(KW_FAM_DIR)

COMMON_PATH   	= -I$(PLAT_PATH_I)/$(COMMON_DIR)
OSSERV_PATH     = -I$(PLAT_PATH_I)/$(OSSERV_DIR)
LSP_PATH        = -I$(LSP_PATH_I)/$(LSP_DIR)
CONFIG_PATH     = -I$(LSP_PATH_I)/$(CONFIG_DIR)

EXTRA_INCLUDE  	= $(OSSERV_PATH) $(COMMON_PATH) $(HAL_PATH)  $(KW_FAM_PATH) \
                  $(LSP_PATH) $(CONFIG_PATH) \
		  $(COMMON_PATH) $(COMMON_PATH)/gnd $(COMMON_PATH)/pool \
		  -I$(LSP_PATH_I)/kw_family/ctrlEnv -I$(LSP_PATH_I)/kw_family/ctrlEnv/sys \
		  -I$(LSP_PATH_I)/kw_family/boardEnv
		   

###################################################################################################
# defines
###################################################################################################
MV_DEFINE = -DMV_LINUX -DMV_CPU_$(ENDIAN) -DMV_$(CPU_ARCH) 

ifeq ($(CONFIG_MV_CESA_TEST),y)
EXTRA_CFLAGS 	+= -DCONFIG_MV_CESA_TEST
endif

ifeq ($(CONFIG_MV88F6281),y)
EXTRA_CFLAGS    += -DMV88F6281
endif

ifeq ($(CONFIG_MV_CESA_CHAIN_MODE_SUPPORT),y)
EXTRA_CFLAGS    += -DMV_CESA_CHAIN_MODE_SUPPORT
endif

EXTRA_CFLAGS 	+= $(EXTRA_INCLUDE) $(MV_DEFINE)

ifeq ($(CONFIG_MV98DX3121),y)
EXTRA_CFLAGS    += -DRD_98DX3121
endif

ifeq ($(CONFIG_MV98DX4122),y)
EXTRA_CFLAGS    += -DDB_98DX4122
endif

ifeq ($(CONFIG_MV_PRESTERA_SWITCH),y)
EXTRA_CFLAGS    += -DMV_PRESTERA_SWITCH
endif

ifeq ($(CONFIG_MV_DRAGONITE_SWITCH),y)
EXTRA_CFLAGS    += -DMV_INCLUDE_DRAGONITE
endif

ifeq ($(CONFIG_PCIE_VIRTUAL_BRIDGE_SUPPORT),y)
EXTRA_CFLAGS    +=-DPCIE_VIRTUAL_BRIDGE_SUPPORT
endif

ifeq ($(CONFIG_MV_INCLUDE_PEX),y)
EXTRA_CFLAGS    += -DMV_INCLUDE_PEX
endif

ifeq ($(CONFIG_MV_PRESTERA_SWITCH_DECODING),y)
EXTRA_CFLAGS    += -DMV_PRESTERA_SWITCH_DECODING
endif

ifeq ($(CONFIG_ETH_DESCR_IN_HIGH_MEM),y)
EXTRA_CFLAGS    += -DETH_DESCR_IN_HIGH_MEM
endif

ifeq ($(CONFIG_MV_INCLUDE_SAGE),y)
EXTRA_CFLAGS    += -DMV_INCLUDE_SAGE
endif
