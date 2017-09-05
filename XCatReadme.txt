-- Binaries build for BE and LE.
-- Same kernel will work with SPI and NAND flash.
-- Config files are :
	mv88fxcat_le_defconfig
	mv88fxcat_be_defconfig
-- See mtd-utils section in LspReadme.txt how to format and use flashes.
-- Added simple test module to PPdrv.
	To use the module :
		Compile it with comm.sh script
		Run 'insmod PPdrv.ko'
		Run commands like 
		'echo '0 2040000' > /proc/ppdrv/read' and 
		'echo '0 2040000 69220511' > /proc/ppdrv/write'
		Also used for FP rule management, 
			MIB counters read, and I2C r/w, see examples :
		echo 5 > proc/ppdrv/mib
		echo 0 40 > proc/ppdrv/i-read
		echo 0 40 40 > proc/ppdrv/i-write
-- Standalone network driver:
	By default Kernel configured in CPSS mode, the meaning for Linux user is standalone PP ports not functional.
	To enable standalone network driver the cmdline passed from U-Boot should consist:
		"standalone_network_device ppsdma" - to configure PP ports in SDMA mode
		"standalone_network_device ppmii" - to configure PP ports in RGMII mode
	In this mode user can configure port with "ifconfig p0 10.10.10.10" and use it as usual network port.