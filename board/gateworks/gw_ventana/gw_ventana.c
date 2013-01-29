/*
 * Copyright (C) 2013 Gateworks Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6q_pins.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/sys_proto.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/imx_pwm.h>
#include <linux/list.h>
#include <linux/ctype.h>
#include <linux/fb.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <ipu_pixfmt.h>
#include <i2c.h>
#include <fdt_support.h>
#include <jffs2/load_kernel.h>
#include <mtd_node.h>
#include <spi_flash.h>
#include <hwconfig.h>
#include <fuse.h>

#include "ventana_eeprom.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |	       \
       PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |	       \
       PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |	       \
       PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |	       \
       PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED	  |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

/* UART1, RS485 */
iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT6__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT7__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* UART2, Console */
iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_SD4_DAT7__UART2_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD4_DAT4__UART2_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

#if 0 // depends on GW5400A vs GW5400B
/* UART3, GPS */
iomux_v3_cfg_t const uart3_pads[] = {
	MX6_PAD_SD4_CMD__UART3_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD4_CLK__UART3_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C1, GSC */
struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D21__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_EIM_D21__GPIO_3_21 | PC,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_EIM_D28__GPIO_3_28 | PC,
		.gp = IMX_GPIO_NR(3, 28)
	}
};

/* I2C2, PFUSE, PCIe Switch/Clock/Mezz */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO_4_12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO_4_13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

/* I2C3, Accel, Audio Codec, Video Decoder, Video Encoder, MIPI, LVDS, DIO */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_3__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO_3__GPIO_1_3 | PC,
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO_6__GPIO_1_6 | PC,
		.gp = IMX_GPIO_NR(1, 6)
	}
};

/* MMC */
iomux_v3_cfg_t const usdhc3_pads[] = {
       MX6_PAD_SD3_CLK__USDHC3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6_PAD_SD3_CMD__USDHC3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6_PAD_SD3_DAT0__USDHC3_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6_PAD_SD3_DAT1__USDHC3_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6_PAD_SD3_DAT2__USDHC3_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6_PAD_SD3_DAT3__USDHC3_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6_PAD_SD3_DAT5__GPIO_7_0    | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

/* ENET */
iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__ENET_RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__ENET_RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__ENET_RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__ENET_RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__ENET_RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__ENET_RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__ENET_RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__ENET_RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__ENET_RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__ENET_RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD0__GPIO_1_30 | MUX_PAD_CTRL(NO_PAD_CTRL), /* PHY nRST */
};

/* NAND */
iomux_v3_cfg_t const nfc_pads[] = {
	MX6_PAD_NANDF_CLE__RAWNAND_CLE     | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_ALE__RAWNAND_ALE     | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_WP_B__RAWNAND_RESETN | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_RB0__RAWNAND_READY0  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_CS0__RAWNAND_CE0N    | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD4_CMD__RAWNAND_RDN       | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD4_CLK__RAWNAND_WRN       | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D0__RAWNAND_D0       | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D1__RAWNAND_D1       | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D2__RAWNAND_D2       | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D3__RAWNAND_D3       | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D4__RAWNAND_D4       | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D5__RAWNAND_D5       | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D6__RAWNAND_D6       | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D7__RAWNAND_D7       | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#ifdef CONFIG_CMD_NAND
static void setup_gpmi_nand(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/* config gpmi nand iomux */
	imx_iomux_v3_setup_multiple_pads(nfc_pads,
		ARRAY_SIZE(nfc_pads));

	/* config gpmi and bch clock to 100 MHz */
	clrsetbits_le32(&mxc_ccm->cs2cdr,
		MXC_CCM_CS2CDR_ENFC_CLK_PODF_MASK |
		MXC_CCM_CS2CDR_ENFC_CLK_PRED_MASK |
		MXC_CCM_CS2CDR_ENFC_CLK_SEL_MASK,
		MXC_CCM_CS2CDR_ENFC_CLK_PODF(0) |
		MXC_CCM_CS2CDR_ENFC_CLK_PRED(3) |
		MXC_CCM_CS2CDR_ENFC_CLK_SEL(3));

	/* enable gpmi and bch clock gating */
	setbits_le32(&mxc_ccm->CCGR4,
		MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
		MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
		MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
		MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
		MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_OFFSET);

	/* enable apbh clock gating */
	setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
}
#endif

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* toggle PHY_RST# */
	gpio_direction_output(IMX_GPIO_NR(1, 30), 0);
	mdelay(2);
	gpio_set_value(IMX_GPIO_NR(1, 30), 1);
}

iomux_v3_cfg_t const usb_pads[] = {
	/* HUB reset */
	MX6_PAD_SD1_DAT0__GPIO_1_16 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* USBOTG_PWR enable */
	MX6_PAD_EIM_D22__USBOH3_USBOTG_PWR,
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
#if 0 // depends on GW5400A vs GW5400B
	imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
#endif
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));

	/* Reset USB hub */
	gpio_direction_output(IMX_GPIO_NR(1, 16), 0);
	mdelay(2);
	gpio_set_value(IMX_GPIO_NR(1, 16), 1);

	return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[1] = {
       {USDHC3_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	if (cfg->esdhc_base == USDHC3_BASE_ADDR) {
		/* Card Detect */
		gpio_direction_input(IMX_GPIO_NR(7, 0));
		ret = !gpio_get_value(IMX_GPIO_NR(7, 0));
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
			case 0:
				imx_iomux_v3_setup_multiple_pads(
					usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
				break;
			default:
				printf("Warning: you configured more USDHC controllers"
					"(%d) then supported by the board (%d)\n",
					index + 1, CONFIG_SYS_FSL_USDHC_NUM);
				return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
	}

	return status;
}
#endif

/* Gateworks System Controller I2C access may NAK when busy - use
 * retries.
 */
int gsc_i2c_read(uchar chip, uint addr, int alen, uchar *buf, int len)
{
	int retry = 3;
	int n = 0;
	int ret;

	while (n++ < retry) {
		ret = i2c_read(chip, addr, alen, buf, len);
		if (!ret)
			break;
		printf("%s: 0x%02x 0x%02x retry%d: %d\n", __func__, chip, addr, n, ret);
		if (ret != -ENODEV)
			break;
		mdelay(10);
	}
	return ret;
}

int gsc_i2c_write(uchar chip, uint addr, int alen, uchar *buf, int len)
{
	int retry = 3;
	int n = 0;
	int ret;

	while (n++ < retry) {
		ret = i2c_write(chip, addr, alen, buf, len);
		if (!ret)
			break;
		printf("%s: 0x%02x 0x%02x retry%d: %d\n", __func__, chip, addr, n, ret);
		if (ret != -ENODEV)
			break;
		mdelay(10);
	}
	mdelay(1);
	return ret;
}

/*
 * For SPL boot some boards need i2c before SDRAM is initialized so force
 * variables to live in SRAM
 */
static struct ventana_board_info __attribute__((section(".data"))) ventana_info;

/* read ventana EEPROM and return structure or NULL on error
 * should be called once, the first time eeprom data is needed
 */
static void
read_eeprom(void)
{
	int i;
	int chksum;
	struct ventana_board_info *info = &ventana_info;
	unsigned char *buf = (unsigned char *) &ventana_info;
	int n = 0;

	memset(info, 0, sizeof(ventana_info));

	// wait for bus and device exist - we will not boot w/o our EEPROM
	while(1) {
		if (0 == i2c_set_bus_num(0) && 0 == i2c_probe(0x51))
			break;
		mdelay(1);
		n++;
/*
		if (n > 2000) {
			printf("EEPROM: Failed to probe EEPROM\n");
			info->model[0] = 0;
			return;
		}
*/
	}

	// read eeprom config section
	if (gsc_i2c_read(0x51, 0x00, 1, buf, sizeof(ventana_info))) {
		printf("EEPROM: Failed to read EEPROM\n");
		info->model[0] = 0;
		return;
	}

	// sanity checks
	if (info->model[0] != 'G' || info->model[1] != 'W') {
		printf("EEPROM: Invalid Model in EEPROM\n");
		info->model[0] = 0;
		return;
	}

	// validate checksum
	for (chksum = 0, i = 0; i < sizeof(*info)-2; i++)
		chksum += buf[i];
	if ((info->chksum[0] != chksum>>8) || (info->chksum[1] != (chksum&0xff))) {
		printf("EEPROM: Failed EEPROM checksum\n");
		info->model[0] = 0;
		return;
	}
}

#ifdef CONFIG_CMD_GSC
int read_hwmon(const char *name, uint reg, uint size, uint low, uint high)
{
	unsigned char buf[3];
	uint ui;
	int ret;

	printf("%-8s:", name);
	memset(buf, 0, sizeof(buf));
	if (gsc_i2c_read(0x29, reg, 1, buf, size)) {
		printf("fRD\n");
		ret = -1;
	} else {
		ret = 0;
		ui = buf[0] | (buf[1]<<8) | (buf[2]<<16);
		if (ui == 0xffffff)
			printf("fVL");
		else if (ui < low) {
			printf("%d fLO", ui);
			ret = -2;
		}
		else if (ui > high) {
			printf("%d fHI", ui);
			ret = -3;
		}
		else
			printf("%d", ui);
	}
	printf("\n");

	return ret;
}

int do_gsc(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	i2c_set_bus_num(0);
	read_hwmon("Temp",     0x00, 2, 0, 9000);
	read_hwmon("VIN",      0x02, 3, 8000, 60000);
	read_hwmon("VDD_3P3",  0x05, 3, 3300*0.9, 3300*1.1);
	read_hwmon("VBATT",    0x08, 3, 2000*0.9, 3000*1.1);
	read_hwmon("VDD_CORE", 0x0e, 3, 1375*0.9, 1375*1.1);
	read_hwmon("VDD_SOC",  0x11, 3, 1375*0.9, 1375*1.1);
	read_hwmon("VDD_HIGH", 0x14, 3, 3000*0.9, 3000*1.1);
	read_hwmon("VDD_DDR",  0x17, 3, 1500*0.9, 1500*1.1);
	read_hwmon("VDD_5P0",  0x0b, 3, 5000*0.9, 5000*1.1);
	read_hwmon("VDD_2P5",  0x23, 3, 2500*0.9, 2500*1.1);
	read_hwmon("VDD_1P8",  0x1d, 3, 1800*0.9, 1800*1.1);
	read_hwmon("VDD_1P0",  0x20, 3, 1000*0.9, 1000*1.1);

	return 0;
}

U_BOOT_CMD(gsc, 1, 1, do_gsc,
  "GSC test",
  ""
);
#endif // CONFIG_CMD_GSC


/* get_mac from env string, with default
 */
static void get_mac(char *envvar, unsigned char *def)
{
	char str[20];
	char *env = getenv(envvar);

	if (!env) {
		sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
			def[0], def[1], def[2], def[3], def[4], def[5]);
		setenv(envvar, str);
	}
}


/* this is passed in as ATAG_REVISION which is reported from linux
 * /proc/cpuinfo 'Revision' field (see arch/arm/kernel/setup.c)
 *
 * the value 0x63000 is needed for imx-lib/vpu_lib.h cpu_is_mx6q function
 * to return true to setup proper firmware filename for loading binary firmware
 * 0x61xxx is imx6dl
 * 0x63xxx is imx6q
 */
u32 get_board_rev(void)
{
	return 0x63000;
}

#ifdef CONFIG_SERIAL_TAG
/* called when setting up ATAGS before booting kernel
 * populate serialnum from the following (in order of priority):
 *   serial# env var
 *   eeprom
 */
void get_board_serial(struct tag_serialnr *serialnr)
{
  char *serial = getenv("serial#");

  if (serial) {
    serialnr->high = 0;
    serialnr->low = simple_strtoul(serial, NULL, 10);
  } else if (ventana_info.model[0]) {
    serialnr->high = 0;
		serialnr->low = ventana_info.serial;
	} else {
    serialnr->high = 0;
    serialnr->low = 0;
  }
}
#endif

#ifdef CONFIG_MXC_SPI
iomux_v3_cfg_t const ecspi1_pads[] = {
	/* SS1 */
	MX6_PAD_EIM_D19__GPIO_3_19   | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

static void setup_spi(void)
{
	gpio_direction_output(CONFIG_SF_DEFAULT_CS, 1);
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));
}
#endif

int board_phy_config(struct phy_device *phydev)
{
	unsigned short val;

	/* Marvel 88E1510 */
	if (phydev->phy_id == 0x1410dd1) {
		/* Errata 3.1 - PHY initialization */
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 0x00ff);
		phy_write(phydev, MDIO_DEVAD_NONE, 17, 0x214b);
		phy_write(phydev, MDIO_DEVAD_NONE, 16, 0x2144);
		phy_write(phydev, MDIO_DEVAD_NONE, 17, 0x0c28);
		phy_write(phydev, MDIO_DEVAD_NONE, 16, 0x2146);
		phy_write(phydev, MDIO_DEVAD_NONE, 17, 0xb233);
		phy_write(phydev, MDIO_DEVAD_NONE, 16, 0x214d);
		phy_write(phydev, MDIO_DEVAD_NONE, 17, 0xcc0c);
		phy_write(phydev, MDIO_DEVAD_NONE, 16, 0x2159);
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 0x00fb);
		phy_write(phydev, MDIO_DEVAD_NONE,  7, 0xc00d);
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 0);

		/* LED configuration (See datasheet section 2.26.4)
		 * LED[0] (SPD:Amber) R16_3.3:0 to 0111: on-GbE link
		 * LED[1] (LNK:Green) R16_3.7:4 to 0001: on-link, blink-activity
		 */
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 3);
		val = phy_read(phydev, MDIO_DEVAD_NONE, 16);
		val &= 0xff00;
		val |= 0x0017;
		phy_write(phydev, MDIO_DEVAD_NONE, 16, val);
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 0);

#if 0 // untested
		/* Errata 4.4: improving link times with non IEEE compliant link partners */
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 0x00fc);
		val = phy_read(phydev, MDIO_DEVAD_NONE, 1);
		val |= 1<<15;
		phy_write(phydev, MDIO_DEVAD_NONE, 1, val);
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 0);
#endif

#if 0 // do not seem to need this on gw5400
		/* introduce tx/rx clock delay */
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 2);
		val = phy_read(phydev, MDIO_DEVAD_NONE, 21);
		val &= ~(1<<4); /* tx clock delay */
		val &= ~(1<<5); /* rx clock delay */
		phy_write(phydev, MDIO_DEVAD_NONE, 21, val);
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 0);
#endif

#if 0 // only needed if making changes above which require reset
		/* soft reset PHY */
		val = phy_read(phydev, MDIO_DEVAD_NONE, 0);
		val |= (1<<15);
		phy_write(phydev, MDIO_DEVAD_NONE, 0, val);
#endif
	}

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int board_eth_init(bd_t *bis)
{
	int ret;

	setup_iomux_enet();

	ret = cpu_eth_init(bis);
	if (ret)
		printf("FEC MXC: %s:failed\n", __func__);

	return 0;
}

static void setup_board_gpio(const char* model)
{
	if (strncasecmp(model, "GW54", 4) == 0) {
		if (strncasecmp(model, "GW5400-A", 8) == 0) {
			// PANLEDG#
			imx_iomux_v3_setup_pad(MX6_PAD_KEY_COL0__GPIO_4_6 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_output(IMX_GPIO_NR(4, 6), 1);  // grn off

			// PANLEDR#
			imx_iomux_v3_setup_pad(MX6_PAD_KEY_COL2__GPIO_4_10 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_output(IMX_GPIO_NR(4, 10), 1); // red off

			// MX6_LOCLED#
			imx_iomux_v3_setup_pad(MX6_PAD_KEY_ROW4__GPIO_4_15 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_output(IMX_GPIO_NR(4, 15), 1); // loc off

			// MIPI DIO
			imx_iomux_v3_setup_pad(MX6_PAD_SD1_DAT3__GPIO_1_21 | MUX_PAD_CTRL(NO_PAD_CTRL));

			// RS485 Transmit Enable
			imx_iomux_v3_setup_pad(MX6_PAD_EIM_D24__GPIO_3_24 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_output(IMX_GPIO_NR(3, 24), 0);

			// Expansion IO0 - PWREN#
			imx_iomux_v3_setup_pad(MX6_PAD_KEY_ROW0__GPIO_4_7 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_output(IMX_GPIO_NR(4, 7), 0);

			// Expansion IO1 - IRQ#
			imx_iomux_v3_setup_pad(MX6_PAD_KEY_ROW1__GPIO_4_9 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_input(IMX_GPIO_NR(4, 9));
		}

		else if ( (strncasecmp(model, "GW5400", 6) == 0)
		       || (strncasecmp(model, "GW5410", 6) == 0)
		) {
			// PANLEDG#
			imx_iomux_v3_setup_pad(MX6_PAD_KEY_COL0__GPIO_4_6 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_output(IMX_GPIO_NR(4, 6), 1);  // grn off

			// PANLEDR#
			imx_iomux_v3_setup_pad(MX6_PAD_KEY_ROW0__GPIO_4_7 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_output(IMX_GPIO_NR(4, 7), 1);  // red off

			// MX6_LOCLED#
			imx_iomux_v3_setup_pad(MX6_PAD_KEY_ROW4__GPIO_4_15 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_output(IMX_GPIO_NR(4, 15), 1);  // grn off

			// RS485 Transmit Enable
			imx_iomux_v3_setup_pad(MX6_PAD_SD3_DAT4__GPIO_7_1 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_output(IMX_GPIO_NR(7, 1), 0);

			// Expansion IO0 - PWREN#
			imx_iomux_v3_setup_pad(MX6_PAD_EIM_A19__GPIO_2_19 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_output(IMX_GPIO_NR(2, 19), 0);

			// Expansion IO1 - IRQ#
			imx_iomux_v3_setup_pad(MX6_PAD_EIM_A20__GPIO_2_18 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_input(IMX_GPIO_NR(2, 18));

			// MSATA Enable
			imx_iomux_v3_setup_pad(MX6_PAD_SD4_DAT0__GPIO_2_8 | MUX_PAD_CTRL(NO_PAD_CTRL));
			gpio_direction_output(IMX_GPIO_NR(2, 8), (hwconfig("msata"))?1:0);
			printf("MSATA: %s\n", (hwconfig("msata")?"enabled":"disabled"));
		}

		// UART2_EN#
		imx_iomux_v3_setup_pad(MX6_PAD_SD4_DAT3__GPIO_2_11 | MUX_PAD_CTRL(NO_PAD_CTRL));
		printf("RS232: %s\n", (hwconfig("rs232"))?"enabled":"disabled");
		gpio_direction_output(IMX_GPIO_NR(2, 11), (hwconfig("rs232"))?0:1);
		// TODO: flush UART RX FIFO after disable

		// DIOI2C_DIS#
		imx_iomux_v3_setup_pad(MX6_PAD_GPIO_19__GPIO_4_5 | MUX_PAD_CTRL(NO_PAD_CTRL));
		gpio_direction_output(IMX_GPIO_NR(4,  5), 0);

		/* configure board general purpose IO's based on hwconfig
		 */
		// MX6_DIO0
		if (hwconfig("dio0")) {
			if (hwconfig_subarg_cmp("dio0", "mode", "gpio")) {
				printf("DIO0:  gpio\n");
				imx_iomux_v3_setup_pad(MX6_PAD_GPIO_9__GPIO_1_9 | MUX_PAD_CTRL(NO_PAD_CTRL));
				gpio_direction_input(IMX_GPIO_NR(1, 9));
			} else if (hwconfig_subarg_cmp("dio0", "mode", "pwm")) {
				printf("DIO0:  pwm\n");
				imx_iomux_v3_setup_pad(MX6_PAD_GPIO_9__PWM1_PWMO | MUX_PAD_CTRL(NO_PAD_CTRL));
			}
		}
		// MX6_DIO1
		if (hwconfig("dio1")) {
			if (hwconfig_subarg_cmp("dio1", "mode", "gpio")) {
				printf("DIO1:  gpio\n");
				imx_iomux_v3_setup_pad(MX6_PAD_SD1_DAT2__GPIO_1_19 | MUX_PAD_CTRL(NO_PAD_CTRL));
				gpio_direction_input(IMX_GPIO_NR(1, 19));
			} else if (hwconfig_subarg_cmp("dio1", "mode", "pwm")) {
				printf("DIO1:  pwm\n");
				imx_iomux_v3_setup_pad(MX6_PAD_SD1_DAT2__PWM2_PWMO | MUX_PAD_CTRL(NO_PAD_CTRL));
			}
		}
		// MX6_DIO2
		if (hwconfig("dio2")) {
			if (hwconfig_subarg_cmp("dio2", "mode", "gpio")) {
				printf("DIO2:  gpio\n");
				imx_iomux_v3_setup_pad(MX6_PAD_SD4_DAT1__GPIO_2_9 | MUX_PAD_CTRL(NO_PAD_CTRL));
				gpio_direction_input(IMX_GPIO_NR(2, 9));
			} else if (hwconfig_subarg_cmp("dio2", "mode", "pwm")) {
				printf("DIO2:  pwm\n");
				imx_iomux_v3_setup_pad(MX6_PAD_SD4_DAT1__PWM3_PWMO | MUX_PAD_CTRL(NO_PAD_CTRL));
			}
		}
		// MX6_DIO3
		if (hwconfig("dio3")) {
			if (hwconfig_subarg_cmp("dio3", "mode", "gpio")) {
				printf("DIO3:  gpio\n");
				imx_iomux_v3_setup_pad(MX6_PAD_SD4_DAT2__GPIO_2_10 | MUX_PAD_CTRL(NO_PAD_CTRL));
				gpio_direction_input(IMX_GPIO_NR(2, 10));
			} else if (hwconfig_subarg_cmp("dio3", "mode", "pwm")) {
				printf("DIO3:  pwm\n");
				imx_iomux_v3_setup_pad(MX6_PAD_SD4_DAT2__PWM4_PWMO | MUX_PAD_CTRL(NO_PAD_CTRL));
			}
		}
	}
}

static int setup_pcie(void)
{
	/* disable spread-spectrum clock: kernel hang when enabled - not clear why */
	imx_iomux_v3_setup_pad(MX6_PAD_SD1_CLK__GPIO_1_20 | MUX_PAD_CTRL(NO_PAD_CTRL));
	gpio_direction_output(IMX_GPIO_NR(1, 20), 0); // PCIECK_SSON

	/* toggle PCI_RST# */
	imx_iomux_v3_setup_pad(MX6_PAD_ENET_TXD1__GPIO_1_29 | MUX_PAD_CTRL(NO_PAD_CTRL));
	gpio_direction_output(IMX_GPIO_NR(1, 29), 0); // PCIESWT_RST#
	mdelay(1);
	gpio_direction_output(IMX_GPIO_NR(1, 29), 1); // PCIESWT_RST#

	return 0;
}

#ifdef CONFIG_CMD_SATA
static int setup_sata(void)
{
	struct iomuxc_base_regs *const iomuxc_regs
		= (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	int ret = enable_sata_clock();
	if (ret)
		return ret;

	clrsetbits_le32(&iomuxc_regs->gpr[13],
			IOMUXC_GPR13_SATA_MASK,
			IOMUXC_GPR13_SATA_PHY_8_RXEQ_3P0DB
			|IOMUXC_GPR13_SATA_PHY_7_SATA2M
			|IOMUXC_GPR13_SATA_SPEED_3G
			|(3<<IOMUXC_GPR13_SATA_PHY_6_SHIFT)
			|IOMUXC_GPR13_SATA_SATA_PHY_5_SS_DISABLED
			|IOMUXC_GPR13_SATA_SATA_PHY_4_ATTEN_9_16
			|IOMUXC_GPR13_SATA_PHY_3_TXBOOST_0P00_DB
			|IOMUXC_GPR13_SATA_PHY_2_TX_1P104V
			|IOMUXC_GPR13_SATA_PHY_1_SLOW);

	return 0;
}
#endif

#if defined(CONFIG_VIDEO_IPUV3)

static iomux_v3_cfg_t const backlight_pads[] = {
	/* Backlight on MIPI connector: J16 */
	MX6_PAD_SD2_CMD__GPIO_1_11 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define MIPI_BACKLIGHT_GP IMX_GPIO_NR(1, 11)

	/* Backlight CABEN on LVDS connector: J6 */
	MX6_PAD_SD2_CLK__GPIO_1_10 | MUX_PAD_CTRL(NO_PAD_CTRL),

#define LVDS_BACKLIGHT_GP IMX_GPIO_NR(1, 10)
};

static struct pwm_device pwm3 = {
	.pwm_id = 3,
	.pwmo_invert = 0,
};

/* Parallel RGB Input (Analog Video In) */
static iomux_v3_cfg_t const vidin_pads[] = {
	MX6_PAD_EIM_EB2__IPU2_CSI1_D_19,
	MX6_PAD_EIM_D16__IPU2_CSI1_D_18,
	MX6_PAD_EIM_D18__IPU2_CSI1_D_17,
	MX6_PAD_EIM_D19__IPU2_CSI1_D_16,
	MX6_PAD_EIM_D20__IPU2_CSI1_D_15,
	MX6_PAD_EIM_D26__IPU2_CSI1_D_14,
	MX6_PAD_EIM_D27__IPU2_CSI1_D_13,
	MX6_PAD_EIM_A17__IPU2_CSI1_D_12,
	MX6_PAD_EIM_D29__IPU2_CSI1_VSYNC,
	MX6_PAD_EIM_EB3__IPU2_CSI1_HSYNC,
	MX6_PAD_EIM_A16__IPU2_CSI1_PIXCLK,
};

/* Parallel RGB Output (Analog Video Out) */
static iomux_v3_cfg_t const vidout_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN2,
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN3,
	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
};

struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};


static int detect_hdmi(struct display_info_t const *dev)
{
	struct hdmi_regs *hdmi  = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	return readb(&hdmi->phy_stat0) & HDMI_PHY_HPD;
}

static void enable_hdmi(struct display_info_t const *dev)
{
	struct hdmi_regs *hdmi  = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	u8 reg;
	printf("%s: setup HDMI monitor\n", __func__);
	reg = readb(&hdmi->phy_conf0);
	reg |= HDMI_PHY_CONF0_PDZ_MASK;
	writeb(reg, &hdmi->phy_conf0);

	udelay(3000);
	reg |= HDMI_PHY_CONF0_ENTMDS_MASK;
	writeb(reg, &hdmi->phy_conf0);
	udelay(3000);
	reg |= HDMI_PHY_CONF0_GEN2_TXPWRON_MASK;
	writeb(reg, &hdmi->phy_conf0);
	writeb(HDMI_MC_PHYRSTZ_ASSERT, &hdmi->mc_phyrstz);
}

static int detect_i2c(struct display_info_t const *dev)
{
	return ((0 == i2c_set_bus_num(dev->bus))
		&&
		(0 == i2c_probe(dev->addr)));
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;

	/* set CH0 data width to 24bit (IOMUXC_GPR2:5 0=18bit, 1=24bit) */
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
	writel(reg, &iomux->gpr[2]);

	/* Disable CABC:
	 * when enabled this feature sets backlight automatically according to content
	 * which may cause annoying unstable backlight issue
	 */
	gpio_direction_output(LVDS_BACKLIGHT_GP, 0);

	/* pwm */
	imx_iomux_v3_setup_pad(MX6_PAD_SD1_CMD__PWM4_PWMO | MUX_PAD_CTRL(PAD_CTL_DSE_240ohm));
	imx_pwm_config(pwm3, 25000, 50000);
	imx_pwm_enable(pwm3);
}

#if 0
static void enable_vidout(struct display_info_t const *dev)
{
	imx_iomux_v3_setup_multiple_pads(
		vidout_pads,
		 ARRAY_SIZE(vidout_pads));
}
#endif

static struct display_info_t const displays[] = {{
	/* HDMI Output */
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	/* HannStar HSD100PXN1-A00 with egalx_ts controller
	 * (aka Freescale MXC-LVDS1 10" 1024x768 60Hz LCD touchscreen)
	 */
	.bus	= 2,
	.addr	= 0x4,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} } };

int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel = getenv("panel");
	if (!panel) {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			struct display_info_t const *dev = displays+i;
			if (dev->detect(dev)) {
				panel = dev->mode.name;
				printf("auto-detected panel %s\n", panel);
				break;
			}
		}
		if (!panel) {
			panel = displays[0].mode.name;
			i = 0;
			printf("No panel detected: default to %s\n", panel);
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0,
				    displays[i].pixfmt);
		if (!ret) {
			displays[i].enable(displays+i);
			printf("DISP:  %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		ret = -EINVAL;
	}
	return (0 != ret);
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	struct hdmi_regs *hdmi = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	int reg;

	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=   MXC_CCM_CCGR3_IPU1_IPU_DI0_OFFSET
		|MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* Turn on HDMI PHY clock */
	reg = __raw_readl(&mxc_ccm->CCGR2);
	reg |=  MXC_CCM_CCGR2_HDMI_TX_IAHBCLK_MASK
	       |MXC_CCM_CCGR2_HDMI_TX_ISFRCLK_MASK;
	writel(reg, &mxc_ccm->CCGR2);

	/* clear HDMI PHY reset */
	writeb(HDMI_MC_PHYRSTZ_DEASSERT, &hdmi->mc_phyrstz);

	/* set PFD1_FRAC to 0x13 == 455 MHz (480*18)/0x13 */
	writel(ANATOP_PFD_480_PFD1_FRAC_MASK, &anatop->pfd_480_clr);
	writel(0x13<<ANATOP_PFD_480_PFD1_FRAC_SHIFT, &anatop->pfd_480_set);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg &= ~(MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK
		|MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK
		|MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET)
	      |(CHSCCDR_PODF_DIVIDE_BY_3
		<<MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET)
	      |(CHSCCDR_IPU_PRE_CLK_540M_PFD
		<<MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	     |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~IOMUXC_GPR3_LVDS0_MUX_CTL_MASK)
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);

	/* backlights off until needed */
	imx_iomux_v3_setup_multiple_pads(backlight_pads,
					 ARRAY_SIZE(backlight_pads));
	gpio_direction_input(LVDS_BACKLIGHT_GP);
	gpio_direction_input(MIPI_BACKLIGHT_GP);
}
#endif

static int setup_pmic_voltages(void)
{
	int ret;
	unsigned char value, rev_id = 0 ;

	ret = i2c_set_bus_num(1);
	if (ret)
		return ret; 
	if (!i2c_probe(0x8)) {
		if (i2c_read(0x8, 0, 1, &value, 1)) {
			printf("Read device ID error!\n");
			return -1;
		}
		if (i2c_read(0x8, 3, 1, &rev_id, 1)) {
			printf("Read Rev ID error!\n");
			return -1;
		}
		printf("PMIC:  deviceid=%x, revid=%x\n", value, rev_id);
		/*set VGEN1 to 1.5V and enable*/
		if (i2c_read(0x8, 0x6c, 1, &value, 1)) {
			printf("Read VGEN1 error!\n");
			return -1;
		}
		value &= ~0x1f;
		value |= 0x1e;
		if (i2c_write(0x8, 0x6c, 1, &value, 1)) {
			printf("Set VGEN1 error!\n");
			return -1;
		}
		/*set SWBST to 5.0V and enable */
		if (i2c_read(0x8, 0x66, 1, &value, 1)) {
			printf("Read SWBST error!\n");
			return -1;
		}
		value &= ~0xf;
		value |= 0x8;
		if (i2c_write(0x8, 0x66, 1, &value, 1)) {
			printf("Set SWBST error!\n");
			return -1;
		}
	}
	return 0;
}

#ifdef CONFIG_CMD_BMODE
/* BOOT_CFG1, BOOT_CFG2, BOOT_CFG3, BOOT_CFG4
 * see Table 8-11 and Table 5-9
 *  BOOT_CFG1[7] = 1 (boot from NAND)
 *  BOOT_CFG1[5] = 0 - raw NAND
 *  BOOT_CFG1[4] = 0 - default pad settings
 *  BOOT_CFG1[3:2] = 00 - devices = 1
 *  BOOT_CFG1[1:0] = 00 - Row Address Cycles = 3
 *  BOOT_CFG2[4:3] = 00 - Boot Search Count = 2
 *  BOOT_CFG2[2:1] = 01 - Pages In Block = 64
 *  BOOT_CFG2[0] = 0 - Reset time 12ms
 */
static const struct boot_mode board_boot_modes[] = {
	/* NAND: raw, 64pages per block, 3 row addr cycles, 2 copies of FCB/DBBT */
	{ "nand", MAKE_CFGVAL(0x80, 0x02, 0x00, 0x00) },
	{ NULL,	0 },
};
#endif

/*
 * Board Support
 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}


/*
 * very early in the call chain - setup SoC peripherals
 * (NB: Can not printf from here)
 */
int board_early_init_f(void)
{
	setup_iomux_uart();
#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

	return 0;
}

#if defined(CONFIG_DISPLAY_BOARDINFO)
/* Identify board and display banner/info
 */
#define SRC_SBMR1 0x020d8004  // holds BOOT_CFG1-BOOT_CFG4 from eFuse/pins
#define SRC_SBMR2 0x020d801c
#define SRC_GPR9  0x020d8040  // holds copy of BOOT_CFG1-BOOT_CFG4 acted on
#define SRC_GPR10 0x020d8044

void show_boot_mode(uint boot_cfg)
{
// this reads boot_cfg efuse/pins - does not reflect what actually booted
// from if you used bmode
//printf("BOOT_CFG1=0x%02x\n", src_sbmr1 & 0xff);
//printf("BOOT_CFG2=0x%02x\n", (src_sbmr1 >> 8) & 0xff);
//printf("BOOT_CFG3=0x%02x\n", (src_sbmr1 >> 16) & 0xff);
//printf("BOOT_CFG4=0x%02x\n", (src_sbmr1 >> 24) & 0xff);
	switch ((boot_cfg & 0x000000ff) >> 4) {
	case 0x2:
		printf("SATA");
		break;
	case 0x3:
		printf("SPI NOR");
		break;
	case 0x4:
	case 0x5:
		// BOOT_CFG2[3:4] is devno
		printf(" SD%d", (boot_cfg & 0x00001800) >> 11);
    break;
  case 0x6:
  case 0x7:
		// BOOT_CFG2[3:4] is devno
		printf(" MMC%d", (boot_cfg & 0x00001800) >> 11);
    break;
  case 0x8 ... 0xf:
		printf("NAND");
    break;
  default:
		printf("Unknown");
    break;
  }
	printf(" 0x%08x\n", boot_cfg);
}


int checkboard(void)
{
	struct ventana_board_info *info = &ventana_info;
	uint src_sbmr2 = readl(SRC_SBMR2);
	uint src_gpr10 = readl(SRC_GPR10);

	read_eeprom();
	if (!(src_sbmr2 & 1<<4)) {
		// can consider this 'manufacturing mode' if needed
		printf("First boot - eFUSE not blown\n");
	}

	//if (src_gpr10 & 1<<30)
	//	printf("APP_IMAGE: Secondary\n");
	printf("APP_IMAGE: %s\n", (src_gpr10 & 1<<30)?"Secondary":"Primary");
	if (src_gpr10 & 1<<29)
		printf("NAND: bad blocks in application image\n");

#if defined(CONFIG_ENV_IS_IN_SPI_FLASH)
	printf("Env: SPI FLASH\n");
#elif defined(CONFIG_ENV_IS_IN_MMC)
	printf("Env: MMC\n");
#elif defined(CONFIG_ENV_IS_IN_NAND)
	printf("Env: NAND FLASH\n");
#endif

	// SRC_SBMR1 reflects eFUSE/pin
	printf("BOOT_CFG: ");
	show_boot_mode(readl(SRC_SBMR1));
	// SRC_GPR9 reflects what was actually booted off of if not 0
	// (ie if bmode was used)
	if (readl(SRC_GPR9)) {
		printf("BMODE: ");
		show_boot_mode(readl(SRC_GPR9));
	}
	printf("\n");

	printf("Gateworks Corporation Copyright 2013\n");
	if (info->model[0]) {
		printf("Model: %s\n", info->model);
		printf("MFGDate: %02x-%02x-%02x%02x\n",
			info->mfgdate[0], info->mfgdate[1],
			info->mfgdate[2], info->mfgdate[3]);
		printf("Serial:%d\n", info->serial);
	} else {
		printf("Invalid EEPROM - board will not function fully\n");
	}

	return 0;
}
#endif

/* Set gd->ram_size
 */
int dram_init(void)
{
	struct ventana_board_info *info = &ventana_info;

	if (info->model[0] && info->sdram_size > 0 && info->sdram_size < 9) {
		int i = info->sdram_size;
		gd->ram_size = 32*1024*1024;
		while(--i)
			gd->ram_size *=2;
	} else {
		// let get_ram_size do its work against 1GB
		gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);
	}

	return 0;
}

/* initialize periperhals
 */
int board_init(void)
{
	int ret = 0;

#ifdef CONFIG_CMD_NAND
	setup_gpmi_nand();
#endif

	/* address of linux boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_I2C_MXC
	ret = setup_pmic_voltages();
	if (ret)
		return -1;
#endif

	setup_pcie();

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

	if (!i2c_set_bus_num(0)) {
		unsigned char buf[4];
		if (!gsc_i2c_read(0x20, 14, 1, buf, 1)) {
			printf("GSC:   v%d", buf[0]);
			if (!gsc_i2c_read(0x20, 10, 1, buf, 4)) {
				/* show firmware revision and CRC */
				printf(" 0x%04x", buf[2] | buf[3]<<8);
				/* show status register */
				printf(" 0x%02x", buf[0]);
				/* display if encountered a GSC watchdog timeout */
				if (buf[0] & 0x40) {
					printf(" WD_TIMEOUT");
					/* clear flag */
					buf[0] &= ~0x40;
					gsc_i2c_write(0x20, 1, 1, &buf[0], 1);
				}
			}
			printf("\n");
		}
		if (!gsc_i2c_read(0x68, 0x00, 1, buf, 4))
			printf("RTC:   %d\n", buf[0] | buf[1]<<8 | buf[2]<<16 | buf[3]<<24);
	}

#ifdef CONFIG_CMD_GSC
	// if eFUSE not blown show GSC HWMON info
	if (!(readl(SRC_SBMR2) & 1<<4)) {
		mdelay(1500);
		do_gsc(NULL, 0, 0, NULL);
	}
#endif

	return 0;
}

/* late init
 */
int misc_init_r(void)
{
	/* set env vars based on board model from EEPROM */
	if (ventana_info.model[0]) {
		char str[20];
		char fdt[30];
		char *p;
		int i;

		memset(str, 0, sizeof(str));
		for (i = 0; i < sizeof(ventana_info.model) && ventana_info.model[i]; i++)
			str[i] = tolower(ventana_info.model[i]);
		if (!getenv("model"))
			setenv("model", str);
		if (!getenv("fdt_file")) {
			sprintf(fdt, "imx6q-%s.dtb", str);
			setenv("fdt_file", fdt);
		}
		if ( (p = strchr(str, '-')) ) {
			*p++ = 0;

			setenv("model_base", str);
			if (!getenv("fdt_file1")) {
				sprintf(fdt, "imx6q-%s.dtb", str);
				setenv("fdt_file1", fdt);
			}
			str[4] = 'x';
			str[5] = 'x';
			str[6] = 0;
			if (!getenv("fdt_file2")) {
				sprintf(fdt, "imx6q-%s.dtb", str);
				setenv("fdt_file2", fdt);
			}
		}
		get_mac("ethaddr", ventana_info.mac0);
		get_mac("eth1addr", ventana_info.mac1);
		sprintf(str, "%6d", ventana_info.serial);
		setenv("serial#", str);
		setup_board_gpio(getenv("model"));
	}

	/* generate a random eth mac if no EEPROM (1st boot - mfg mode) */
	else {
		u32 ethaddr_low, ethaddr_high;
		char str[20];

		/* use Device Unique ID bits 0-64 from eFUSE (OCOTP_CFG1/OCOTP_CFG2) */
		fuse_read(0, 1, &ethaddr_low);
		fuse_read(0, 2, &ethaddr_high);

		/*
		 * setting the 2nd LSB in the most significant byte of
		 * the address makes it a locally administered ethernet
		 * address
		 */
    ethaddr_high &= 0xfeff;
		ethaddr_high |= 0x0200;
		sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
			ethaddr_high >> 8, ethaddr_high & 0xff,
			ethaddr_low >> 24, (ethaddr_low >> 16) & 0xff,
			(ethaddr_low >> 8) & 0xff, ethaddr_low & 0xff);
		printf("### Setting random MAC address = \"%s\"\n", str);
		setenv("ethaddr", str);
	}

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	/* disable GSC boot watchdog
	 *
	 *  The Gateworks System Controller implements a boot
	 *  watchdog (always enabled) to cover things like ERR006282 which can
	 *  lead to random boot failures.
	 */
	if (!i2c_set_bus_num(0)) {
		unsigned char val;
		if (!gsc_i2c_read(0x20, 1, 1, &val, 1)) {
			val |= 0x80;
			if (gsc_i2c_write(0x20, 1, 1, &val, 1))
				printf("Error: could not disable GSC Watchdog\n");
		} else {
			printf("Error: could not disable GSC Watchdog\n");
		}
	}

	return 0;
}

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
void ft_board_setup(void *blob, bd_t * bd)
{
	struct ventana_board_info *info = &ventana_info;
	struct node_info nodes[] = {
		{ "sst,w25q256",          MTD_DEV_TYPE_NOR, },  // SPI flash
		{ "fsl,imx6q-gpmi-nand",  MTD_DEV_TYPE_NAND, }, // NAND flash
	};
  const char *model = getenv("model");

	if (getenv("fdt_noauto")) {
		printf("   Skiping ft_board_setup (fdt_noauto defined)\n");
		return;
	}

  if (!model) {
		printf("invalid board info: Leaving FDT fully enabled\n");
		return;
	}

	/* MTD partitions
	 * Update partition nodes using info from mtdparts env var
	 */
	printf("   Updating MTD partitions...\n");
	fdt_fixup_mtdparts(blob, nodes, ARRAY_SIZE(nodes));

	printf("   Adjusting FDT per EEPROM for %s...\n", model);
	/* Note that fdt_fixup_ethernet is called in arm/lib/bootm before this
	 * which sets mac-address and local-mac-address properties of
	 * ethernet<n> aliases to ethaddr...eth<n>addr env
	 */

	/* board serial number */
	fdt_setprop(blob, 0, "system-serial", getenv("serial#"), strlen(getenv("serial#") + 1));

	/* Peripheral Config */
	if (!info->config_eth0)
		fdt_del_node_and_alias(blob, "ethernet0");
	if (!info->config_eth1)
		fdt_del_node_and_alias(blob, "ethernet1");
	if (!info->config_hdmi_out)
		fdt_del_node_and_alias(blob, "hdmi_out");
	if (!info->config_sata)
		fdt_del_node_and_alias(blob, "ahci0");
	if (!info->config_pcie)
		fdt_del_node_and_alias(blob, "pcie");
	if (!info->config_ssi0)
		fdt_del_node_and_alias(blob, "ssi0");
	if (!info->config_ssi1)
		fdt_del_node_and_alias(blob, "ssi1");
	if (!info->config_lcd)
		fdt_del_node_and_alias(blob, "lcd0");
	if (!info->config_lvds0)
		fdt_del_node_and_alias(blob, "lvds0");
	if (!info->config_lvds1)
		fdt_del_node_and_alias(blob, "lvds1");
	if (!info->config_usb0)
		fdt_del_node_and_alias(blob, "usb0");
	if (!info->config_usb1)
		fdt_del_node_and_alias(blob, "usb1");
	if (!info->config_sd0)
		fdt_del_node_and_alias(blob, "usdhc0");
	if (!info->config_sd1)
		fdt_del_node_and_alias(blob, "usdhc1");
	if (!info->config_sd2)
		fdt_del_node_and_alias(blob, "usdhc2");
	if (!info->config_sd3)
		fdt_del_node_and_alias(blob, "usdhc3");
	if (!info->config_uart0)
		fdt_del_node_and_alias(blob, "serial0");
	if (!info->config_uart1)
		fdt_del_node_and_alias(blob, "serial1");
	if (!info->config_uart2)
		fdt_del_node_and_alias(blob, "serial2");
	if (!info->config_uart3)
		fdt_del_node_and_alias(blob, "serial3");
	if (!info->config_uart4)
		fdt_del_node_and_alias(blob, "serial4");
	if (!info->config_ipu0)
		fdt_del_node_and_alias(blob, "ipu0");
	if (!info->config_ipu1)
		fdt_del_node_and_alias(blob, "ipu1");
	if (!info->config_flexcan)
		fdt_del_node_and_alias(blob, "can0");
	if (!info->config_mipi_dsi)
		fdt_del_node_and_alias(blob, "mipi_dsi");
	if (!info->config_mipi_csi)
		fdt_del_node_and_alias(blob, "mipi_csi");
	if (!info->config_tzasc0)
		fdt_del_node_and_alias(blob, "tzasc0");
	if (!info->config_tzasc1)
		fdt_del_node_and_alias(blob, "tzasc1");
	if (!info->config_i2c0)
		fdt_del_node_and_alias(blob, "i2c0");
	if (!info->config_i2c1)
		fdt_del_node_and_alias(blob, "i2c1");
	if (!info->config_i2c2)
		fdt_del_node_and_alias(blob, "i2c2");
	if (!info->config_vpu)
		fdt_del_node_and_alias(blob, "vpu");
	if (!info->config_csi0)
		fdt_del_node_and_alias(blob, "csi0");
	if (!info->config_csi1)
		fdt_del_node_and_alias(blob, "csi1");
	if (!info->config_caam)
		fdt_del_node_and_alias(blob, "caam");
	if (!info->config_espci0)
		fdt_del_node_and_alias(blob, "spi0");
	if (!info->config_espci1)
		fdt_del_node_and_alias(blob, "spi1");
	if (!info->config_espci2)
		fdt_del_node_and_alias(blob, "spi2");
	if (!info->config_espci3)
		fdt_del_node_and_alias(blob, "spi3");
	if (!info->config_espci4)
		fdt_del_node_and_alias(blob, "spi4");
	if (!info->config_espci5)
		fdt_del_node_and_alias(blob, "spi5");
	if (!info->config_hdmi_in)
		fdt_del_node_and_alias(blob, "hdmi_in");
	if (!info->config_vid_out)
		fdt_del_node_and_alias(blob, "cvbs_out");
	if (!info->config_vid_in)
		fdt_del_node_and_alias(blob, "cvbs_in");
	if (!info->config_nand)
		fdt_del_node_and_alias(blob, "nand");
}
#endif /* defined(CONFIG_OF_FLAT_TREE) && defined(CONFIG_OF_BOARD_SETUP) */

