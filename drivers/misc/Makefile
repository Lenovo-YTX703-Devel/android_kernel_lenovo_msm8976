#
# Makefile for misc devices that really don't fit anywhere else.
#

obj-$(CONFIG_IBM_ASM)		+= ibmasm/
obj-$(CONFIG_AD525X_DPOT)	+= ad525x_dpot.o
obj-$(CONFIG_AD525X_DPOT_I2C)	+= ad525x_dpot-i2c.o
obj-$(CONFIG_AD525X_DPOT_SPI)	+= ad525x_dpot-spi.o
obj-$(CONFIG_INTEL_MID_PTI)	+= pti.o
obj-$(CONFIG_ATMEL_PWM)		+= atmel_pwm.o
obj-$(CONFIG_ATMEL_SSC)		+= atmel-ssc.o
obj-$(CONFIG_ATMEL_TCLIB)	+= atmel_tclib.o
obj-$(CONFIG_BMP085)		+= bmp085.o
obj-$(CONFIG_BMP085_I2C)	+= bmp085-i2c.o
obj-$(CONFIG_BMP085_SPI)	+= bmp085-spi.o
obj-$(CONFIG_DUMMY_IRQ)		+= dummy-irq.o
obj-$(CONFIG_ICS932S401)	+= ics932s401.o
obj-$(CONFIG_LKDTM)		+= lkdtm.o
obj-$(CONFIG_TIFM_CORE)       	+= tifm_core.o
obj-$(CONFIG_TIFM_7XX1)       	+= tifm_7xx1.o
obj-$(CONFIG_PHANTOM)		+= phantom.o
obj-$(CONFIG_SENSORS_BH1780)	+= bh1780gli.o
obj-$(CONFIG_SENSORS_BH1770)	+= bh1770glc.o
obj-$(CONFIG_SENSORS_APDS990X)	+= apds990x.o
obj-$(CONFIG_SGI_IOC4)		+= ioc4.o
obj-$(CONFIG_ENCLOSURE_SERVICES) += enclosure.o
obj-$(CONFIG_KGDB_TESTS)	+= kgdbts.o
obj-$(CONFIG_SGI_XP)		+= sgi-xp/
obj-$(CONFIG_SGI_GRU)		+= sgi-gru/
obj-$(CONFIG_CS5535_MFGPT)	+= cs5535-mfgpt.o
obj-$(CONFIG_HP_ILO)		+= hpilo.o
obj-$(CONFIG_APDS9802ALS)	+= apds9802als.o
obj-$(CONFIG_APDS9930)		+= apds993x.o
obj-$(CONFIG_ISL29003)		+= isl29003.o
obj-$(CONFIG_ISL29020)		+= isl29020.o
obj-$(CONFIG_USB_HSIC_SMSC_HUB) += smsc_hub.o
obj-$(CONFIG_SENSORS_TSL2550)	+= tsl2550.o
obj-$(CONFIG_EP93XX_PWM)	+= ep93xx_pwm.o
obj-$(CONFIG_DS1682)		+= ds1682.o
obj-$(CONFIG_TI_DAC7512)	+= ti_dac7512.o
obj-$(CONFIG_UID_STAT)		+= uid_stat.o
obj-$(CONFIG_C2PORT)		+= c2port/
obj-$(CONFIG_HMC6352)		+= hmc6352.o
obj-y				+= eeprom/
obj-y				+= cb710/
obj-$(CONFIG_SPEAR13XX_PCIE_GADGET)	+= spear13xx_pcie_gadget.o
obj-$(CONFIG_VMWARE_BALLOON)	+= vmw_balloon.o
obj-$(CONFIG_ARM_CHARLCD)	+= arm-charlcd.o
obj-$(CONFIG_PCH_PHUB)		+= pch_phub.o
obj-y				+= ti-st/
obj-y				+= lis3lv02d/
obj-y				+= carma/
obj-$(CONFIG_USB_SWITCH_FSA9480) += fsa9480.o
obj-$(CONFIG_ALTERA_STAPL)	+=altera-stapl/
obj-$(CONFIG_INTEL_MEI)		+= mei/
obj-$(CONFIG_VMWARE_VMCI)	+= vmw_vmci/
obj-$(CONFIG_LATTICE_ECP3_CONFIG)	+= lattice-ecp3-config.o
obj-$(CONFIG_SRAM)		+= sram.o
obj-$(CONFIG_TSIF) += msm_tsif.o
obj-$(CONFIG_HAPTIC_ISA1200)		+= isa1200.o
obj-$(CONFIG_QSEECOM) += qseecom.o
ifdef CONFIG_COMPAT
obj-$(CONFIG_QSEECOM) += compat_qseecom.o
endif
obj-$(CONFIG_QFP_FUSE) += qfp_fuse.o
obj-$(CONFIG_TI_DRV2667) += ti_drv2667.o
obj-$(CONFIG_QPNP_MISC) += qpnp-misc.o
obj-$(CONFIG_QCOM_LIQUID_DOCK)	+= qcom_liquid_dock.o
obj-y				+= qcom/
obj-$(CONFIG_USB_EXT_TYPE_C_PERICOM)	+= type-c-pericom.o
obj-$(CONFIG_USB_EXT_TYPE_C_TI)		+= type-c-ti.o
obj-$(CONFIG_UID_CPUTIME) 	+= uid_cputime.o
obj-y				+= fusb301.o
# Required flag to configure the core to operate with the Linux kernel
ccflags-y += -DFSC_PLATFORM_LINUX
# Optional flag to enable interrupt-driven operation
ccflags-y += -DFSC_INTERRUPT_TRIGGERED
# Optional flag to enable debug/hostcomm interface and functionality
ccflags-y += -DFSC_DEBUG+	
# The following flags are used to configure which features are compiled in,
# and the allowed combinations are:
#	FSC_HAVE_SRC - Source only
#	FSC_HAVE_SNK - Sink only
#	FSC_HAVE_SRC, FSC_HAVE_SNK - Source or sink configurable
#	FSC_HAVE_SRC, FSC_HAVE_SNK, FSC_HAVE_DRP - DRP capable source or sink
#	FSC_HAVE_ACCMODE - Accessory mode. Requires FSC_HAVE_SRC.
#	FSC_HAVE_VDM - Vendor Defined Message (VDM) support.
#	FSC_HAVE_DP - Display Port (DP) support. Requires FSC_HAVE_VDM.
ccflags-y += -DFSC_HAVE_SRC
ccflags-y += -DFSC_HAVE_SNK
ccflags-y += -DFSC_HAVE_DRP
ccflags-y += -DFSC_HAVE_ACCMODE
ccflags-y += -DFSC_HAVE_VDM
ccflags-y += -DFSC_HAVE_DP

obj-y     += fusb30x_whole.o
fusb30x_whole-objs         := Platform_Linux/fusb30x_driver.o \
Platform_Linux/fusb30x_global.o \
Platform_Linux/platform.o \
Platform_Linux/platform_helpers.o \
Platform_Linux/hostcomm.o \
core/AlternateModes.o \
core/core.o \
core/fusb30X.o \
core/Log.o \
core/PDPolicy.o \
core/PDProtocol.o \
core/TypeC.o \
core/vdm/bitfield_translators.o \
core/vdm/vdm.o \
core/vdm/vdm_callbacks.o \
core/vdm/vdm_config.o \
core/vdm/DisplayPort/configure.o \
core/vdm/DisplayPort/dp.o \
core/vdm/DisplayPort/dp_system_stubs.o