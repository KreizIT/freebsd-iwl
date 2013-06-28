/*	$FreeBSD$	*/

/*-
 * Copyright (c) 2011 Intel Corporation
 * Copyright (c) 2007, 2008
 *	Damien Bergamini <damien.bergamini@free.fr>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef __iwl_csr_h__
#define __iwl_csr_h__

/*
 * This file implements/defines Command Status Registers
 * of Section 7.3 of PRM
 */

/* Command Status Register Table */
#define IWL_HW_IF_CONFIG	0x000
#define IWL_INT_COALESCING	0x004
#define IWL_INT_PERIODIC	0x005	/* use IWL_WRITE_1 */
#define IWL_INT			0x008
#define IWL_INT_MASK		0x00c
#define IWL_FH_INT		0x010
#define IWL_GPIO_IN		0x018
#define IWL_RESET		0x020
#define IWL_GP_CNTRL		0x024
#define IWL_HW_REV		0x028
#define IWL_EEPROM		0x02c
#define IWL_EEPROM_GP		0x030
#define IWL_OTP_GP		0x034
#define IWL_GIO			0x03c
#define IWL_GP_UCODE_REG	0x048
#define IWL_GP_DRIVER		0x050
#define IWL_UCODE_GP1		0x054
#define IWL_UCODE_GP1_SET	0x058
#define IWL_UCODE_GP1_CLR	0x05c
#define IWL_UCODE_GP2		0x060
#define IWL_UCODE_GP2_SET	0x064
#define IWL_UCODE_GP2_CLR	0x068
#define IWL_LED			0x094
#define IWL_DRAM_INT_TBL	0x0a0
#define IWL_SHADOW_REG_CTRL	0x0a8
#define IWL_GIO_CHICKEN		0x100
#define IWL_ANA_PLL		0x20c
#define IWL_HW_REV_WA		0x22c
#define IWL_DBG_HPET_MEM	0x240
#define IWL_DBG_LINK_PWR_MGMT	0x250

/* HBUS CSR Register Space */
#define IWL_MEM_RADDR		0x40c
#define IWL_MEM_WADDR		0x410
#define IWL_MEM_WDATA		0x418
#define IWL_MEM_RDATA		0x41c
#define IWL_TARG_MBX_C		0x430
#define IWL_PRPH_WADDR		0x444
#define IWL_PRPH_RADDR		0x448
#define IWL_PRPH_WDATA		0x44c
#define IWL_PRPH_RDATA		0x450
#define IWL_HBUS_TARG_WRPTR	0x460

/* Possible flags for register IWL_HW_IF_CONFIG. */
/* HW Interface Configuration Register – CSR */
#define IWL_HW_IF_CONFIG_4965_R		(1 <<  4)
#define IWL_HW_IF_CONFIG_MAC_SI		(1 <<  8)
#define IWL_HW_IF_CONFIG_RADIO_SI	(1 <<  9)
#define IWL_HW_IF_CONFIG_EEPROM_LOCKED	(1 << 21)
#define IWL_HW_IF_CONFIG_NIC_READY	(1 << 22)
#define IWL_HW_IF_CONFIG_HAP_WAKE_L1A	(1 << 23)
#define IWL_HW_IF_CONFIG_PREPARE_DONE	(1 << 25)
#define IWL_HW_IF_CONFIG_PREPARE	(1 << 27)

/* Possible values for register IWL_INT_PERIODIC. */
#define IWL_INT_PERIODIC_DIS	0x00
#define IWL_INT_PERIODIC_ENA	0xff

/* Possible flags for registers IWL_PRPH_RADDR/IWL_PRPH_WADDR. */
#define IWL_PRPH_DWORD		((sizeof (uint32_t) - 1) << 24)

/* Possible values for IWL_BSM_WR_MEM_DST. */
#define IWL_FW_TEXT_BASE	0x00000000
#define IWL_FW_DATA_BASE	0x00800000

/* Possible flags for INTA Register – Interrupt Flags */
#define IWL_INT_ALIVE		(1 <<  0)
#define IWL_INT_WAKEUP		(1 <<  1)
#define IWL_INT_SW_RX		(1 <<  3)
#define IWL_INT_CT_REACHED	(1 <<  6)
#define IWL_INT_RF_TOGGLED	(1 <<  7)
#define IWL_INT_SW_ERR		(1 << 25)
#define IWL_INT_SCHED		(1 << 26)
#define IWL_INT_FH_TX		(1 << 27)
#define IWL_INT_RX_PERIODIC	(1 << 28)
#define IWL_INT_HW_ERR		(1 << 29)
#define IWL_INT_FH_RX		(1 << 31)

/* Shortcut for the above. */
#define IWL_INT_MASK_DEF						\
	(IWL_INT_SW_ERR | IWL_INT_HW_ERR | IWL_INT_FH_TX |		\
	    IWL_INT_FH_RX | IWL_INT_ALIVE | IWL_INT_WAKEUP |		\
	    IWL_INT_SW_RX | IWL_INT_CT_REACHED | IWL_INT_RF_TOGGLED)

/* Possible flags for register IWL_FH_INT. */
/* FH (DMA) Register – Interrupt Flags */
#define IWL_FH_INT_TX_CHNL(x)	(1 << (x))
#define IWL_FH_INT_RX_CHNL(x)	(1 << ((x) + 16))
#define IWL_FH_INT_HI_PRIOR	(1 << 30)
#define IWL_FH_INT_ERR		(1 << 31)
#define IWL_FH_INT_RX_CHNL1	(1 << 17)
#define IWL_FH_INT_RX_CHNL0	(1 << 16)
#define IWL_FH_INT_TX_CHNL1	(1 << 1)
#define IWL_FH_INT_TX_CHNL0	(1 << 0)

/* Shortcuts for the above. */
#define IWL_FH_INT_TX								\
	(IWL_FH_INT_TX_CHNL(0) | IWL_FH_INT_TX_CHNL(1))
#define IWL_FH_INT_RX								\
	(IWL_FH_INT_RX_CHNL(0) | IWL_FH_INT_RX_CHNL(1) | IWL_FH_INT_HI_PRIOR)

/* Possible flags for Reset Register IWL_RESET. */
#define IWL_RESET_NEVO			(1 << 0)
#define IWL_RESET_FORCE_NMI		(1 << 1)
#define IWL_RESET_SW			(1 << 7)
#define IWL_RESET_MASTER_DISABLED	(1 << 8)
#define IWL_RESET_STOP_MASTER		(1 << 9)
#define IWL_RESET_LINK_PWR_MGMT_DIS	(1 << 31)

/* Possible flags for register IWL_GP_CNTRL. */
/* General Purpose Control Register */
#define IWL_GP_CNTRL_MAC_ACCESS_ENA	(1 << 0)
#define IWL_GP_CNTRL_MAC_CLOCK_READY	(1 << 0)
#define IWL_GP_CNTRL_INIT_DONE		(1 << 2)
#define IWL_GP_CNTRL_MAC_ACCESS_REQ	(1 << 3)
#define IWL_GP_CNTRL_SLEEP		(1 << 4)
#define IWL_GP_CNTRL_POWER_SAVE_TYPE	(0x07000000)
#define IWL_GP_CNTRL_RFKILL		(1 << 27)

/* Possible flags for register IWL_HW_REV. */
/* HW Revision Type Register */
#define IWL_HW_REV_TYPE_SHIFT		4
#define IWL_HW_REV_TYPE_MASK		0x000000f0
#define IWL_HW_REV_TYPE_4965		0
#define IWL_HW_REV_TYPE_5300		2
#define IWL_HW_REV_TYPE_5350		3
#define IWL_HW_REV_TYPE_5150		4
#define IWL_HW_REV_TYPE_5100		5
#define IWL_HW_REV_TYPE_1000		6
#define IWL_HW_REV_TYPE_6000		7
#define IWL_HW_REV_TYPE_6050		8
#define IWL_HW_REV_TYPE_6005		11
#define IWL_HW_REV_TYPE_2000		16

/* Possible flags for register IWL_GIO_CHICKEN. */
/* PCI Express Link Power Management Register */
#define IWL_GIO_CHICKEN_L1A_NO_L0S_RX	(1 << 23)
#define IWL_GIO_CHICKEN_DIS_L0S_TIMER	(1 << 29)

/* Possible flags for register IWL_GIO. */
#define IWL_GIO_L0S_ENA			(1 << 1)

/* Possible flags for register IWL_UCODE_GP1. */
#define IWL_UCODE_GP1_MAC_SLEEP		(1 << 0)
#define IWL_UCODE_GP1_RFKILL		(1 << 1)
#define IWL_UCODE_GP1_CMD_BLOCKED	(1 << 2)
#define IWL_UCODE_GP1_CTEMP_STOP_RF	(1 << 3)
#define IWL_UCODE_GP1_CT_KILL_EXIT	(1 << 3)
#define IWL_UCODE_GP1_UCODE_DISABLE	(1 << 4)

/* Possible flags for register IWL_GP_DRIVER. */
#define IWL_GP_DRIVER_RADIO_3X3_HYB	(0 << 0)
#define IWL_GP_DRIVER_RADIO_2X2_HYB	(1 << 0)
#define IWL_GP_DRIVER_RADIO_2X2_IPA	(2 << 0)
#define IWL_GP_DRIVER_CALIB_VER6	(1 << 2)
#define IWL_GP_DRIVER_6050_1X2		(1 << 3)

/* Possible flags/values for register IWL_LED. */
#define IWL_LED_BSM_CTRL		(1 << 5)
#define IWL_LED_OFF			0x00000038
#define IWL_LED_ON			0x00000078

/* Possible flags for register IWL_EEPROM. */
#define IWL_EEPROM_READ_VALID		(1 << 0)
#define IWL_EEPROM_CMD			(1 << 1)
#define IWL_EEPROM_REG_MSK_ADDR		(0x0000FFFC)
#define IWL_EEPROM_REG_MSK_DATA		(0xFFFF0000)

/* Possible flags for register IWL_EEPROM_GP. */
#define IWL_EEPROM_GP_VALID_MSK				(0x00000007)
#define IWL_EEPROM_GP_BAD_SIGNATURE_BOTH_EEP_AND_OTP	(0x00000000)
#define IWL_EEPROM_GP_BAD_SIG_EEP_GOOD_SIG_OTP		(0x00000001)
#define IWL_EEPROM_GP_GOOD_SIG_EEP_LESS_THAN_4K		(0x00000002)
#define IWL_EEPROM_GP_GOOD_SIG_EEP_MORE_THAN_4K		(0x00000004)
#define IWL_EEPROM_GP_IF_OWNER				(0x00000180)

/* Possible flags for register IWL_OTP_GP. */
#define IWL_OTP_GP_DEV_SEL_OTP		(1 << 16)
#define IWL_OTP_GP_RELATIVE_ACCESS	(1 << 17)
#define IWL_OTP_GP_ECC_CORR_STTS	(1 << 20)
#define IWL_OTP_GP_ECC_UNCORR_STTS	(1 << 21)

/* Possible flags for register IWL_DRAM_INT_TBL. */
#define IWL_DRAM_INT_TBL_WRAP_CHECK	(1 << 27)
#define IWL_DRAM_INT_TBL_ENABLE		(1 << 31)

/* Possible values for register IWL_ANA_PLL. */
#define IWL_ANA_PLL_INIT		0x00880300

#endif /* __iwl_csr_h__ */
