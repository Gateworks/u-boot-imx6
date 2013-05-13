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
#include <asm/arch/mx6x_pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/imx_pwm.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <fdt_support.h>

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
	MX6Q_PAD_SD3_DAT6__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6Q_PAD_SD3_DAT7__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6Q_PAD_EIM_D24__UART1_DTR | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* UART2, Console */
iomux_v3_cfg_t const uart2_pads[] = {
       MX6Q_PAD_SD4_DAT7__UART2_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
       MX6Q_PAD_SD4_DAT4__UART2_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* UART3, GPS */
iomux_v3_cfg_t const uart3_pads[] = {
       MX6Q_PAD_SD4_CMD__UART3_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
       MX6Q_PAD_SD4_CLK__UART3_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C1, GSC */
struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_EIM_D21__I2C1_SCL | PC,
		.gpio_mode = MX6Q_PAD_EIM_D21__GPIO_3_21 | PC,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_EIM_D28__I2C1_SDA | PC,
		.gpio_mode = MX6Q_PAD_EIM_D28__GPIO_3_28 | PC,
		.gp = IMX_GPIO_NR(3, 28)
	}
};

/* I2C2, PFUSE, PCIe Switch/Clock/Mezz */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6Q_PAD_KEY_COL3__GPIO_4_12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6Q_PAD_KEY_ROW3__GPIO_4_13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

/* I2C3, Accel, Audio Codec, Video Decoder, Video Encoder, MIPI, LVDS, DIO */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_GPIO_3__I2C3_SCL | PC,
		.gpio_mode = MX6Q_PAD_GPIO_3__GPIO_1_3 | PC,
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_GPIO_6__I2C3_SDA | PC,
		.gpio_mode = MX6Q_PAD_GPIO_6__GPIO_1_6 | PC,
		.gp = IMX_GPIO_NR(1, 6)
	}
};

/* MMC */
iomux_v3_cfg_t const usdhc3_pads[] = {
       MX6Q_PAD_SD3_CLK__USDHC3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD3_CMD__USDHC3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD3_DAT0__USDHC3_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD3_DAT1__USDHC3_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD3_DAT2__USDHC3_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD3_DAT3__USDHC3_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD3_DAT5__GPIO_7_0    | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

/* ENET */
iomux_v3_cfg_t const enet_pads[] = {
	MX6Q_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_ENET_TXD0__GPIO_1_30 | MUX_PAD_CTRL(NO_PAD_CTRL), /* PHY nRST */
};


/* GPIO assignments */
static iomux_v3_cfg_t const gpio_pads[] = {
	// MX6_DIO0
	//MX6Q_PAD_GPIO_9__GPIO_1_9 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6Q_PAD_GPIO_9__PWM1_PWMO | MUX_PAD_CTRL(NO_PAD_CTRL),

	// MX6_DIO1
	//MX6Q_PAD_SD1_DAT2__GPIO_1_19 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6Q_PAD_SD1_DAT2__PWM2_PWMO | MUX_PAD_CTRL(NO_PAD_CTRL),

	// MX6_DIO2
	MX6Q_PAD_SD4_DAT1__GPIO_2_9 | MUX_PAD_CTRL(NO_PAD_CTRL),
	//MX6Q_PAD_SD4_DAT1__PWM3_PWMO | MUX_PAD_CTRL(NO_PAD_CTRL),

	// MX6_DIO3
	MX6Q_PAD_SD4_DAT2__GPIO_2_10 | MUX_PAD_CTRL(NO_PAD_CTRL),
	//MX6Q_PAD_SD4_DAT2__PWM4_PWMO | MUX_PAD_CTRL(NO_PAD_CTRL),

	// MIPI_DIO
	MX6Q_PAD_SD1_DAT3__GPIO_1_21 | MUX_PAD_CTRL(NO_PAD_CTRL),
	// UART2_EN#
	MX6Q_PAD_SD4_DAT3__GPIO_2_11 | MUX_PAD_CTRL(NO_PAD_CTRL),
	// DIOI2C_DIS#
	MX6Q_PAD_GPIO_19__GPIO_4_5 | MUX_PAD_CTRL(NO_PAD_CTRL),

	// PCI6EXP_IO0 - PWREN#
	MX6Q_PAD_KEY_ROW0__GPIO_4_7 | MUX_PAD_CTRL(NO_PAD_CTRL),
	// PCI6EXP_IO1 - IRQ# 
	MX6Q_PAD_KEY_ROW1__GPIO_4_9 | MUX_PAD_CTRL(NO_PAD_CTRL),

	// PCIECK_SSON
	MX6Q_PAD_SD1_CLK__GPIO_1_20 | MUX_PAD_CTRL(NO_PAD_CTRL),
	// PCIESWT_RST#
	MX6Q_PAD_ENET_TXD1__GPIO_1_29 | MUX_PAD_CTRL(NO_PAD_CTRL),

	// MX6_PANLEDG#
	MX6Q_PAD_KEY_COL0__GPIO_4_6 | MUX_PAD_CTRL(NO_PAD_CTRL),
	// MX6_PANLEDR#
	MX6Q_PAD_KEY_COL2__GPIO_4_10 | MUX_PAD_CTRL(NO_PAD_CTRL),
	// MX6_LOCLED#
	MX6Q_PAD_KEY_ROW4__GPIO_4_15 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

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
	MX6Q_PAD_SD1_DAT0__GPIO_1_16 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
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

/* read ventana EEPROM and return structure or NULL on error
 */
static struct ventana_board_info *read_eeprom(int display)
{
#if 1
	int i;
	int chksum;
	static struct ventana_board_info board_info;
	unsigned char *buf = (unsigned char *) &board_info;
	struct ventana_board_info *info = &board_info;

//printf("%s\n", __func__);
	memset(info, 0, sizeof(board_info));

	// ensure bus and device exist
	//mdelay(1000); // I believe this is needed to wait for GSC to powerup w/o battery - need to investigate
	if (i2c_set_bus_num(0) || i2c_probe(0x51)) {
		if (display)
			printf("**** Failed to detect GSC EEPROM\n");
		return NULL;
	}

	// read eeprom config section
	if (i2c_read(0x51, 0x00, 1, buf, sizeof(board_info))) {
		if (display)
			printf("**** Failed to read GSC EEPROM\n");
		return NULL;
	}

	// sanity checks
	if (info->model[0] != 'G' || info->model[1] != 'W') {
		if (display)
			printf("**** Invalid Model\n");
		return NULL;
	}

	// validate checksum
//printf("calculating checksum\n");
	for (chksum = 0, i = 0; i < sizeof(*info)-2; i++)
		chksum += buf[i];
//printf("chksum:0x%04x:0x%02x%02x\n", chksum, info->chksum[1], info->chksum[0]);
	if ((info->chksum[0] != chksum>>8) || (info->chksum[1] != (chksum&0xff))) {
		if (display)
			printf("**** Failed EEPROM checksum\n");
		return NULL;
	}

//printf("board info valid\n");
	return info;
#else
	return NULL;
#endif
}


/* get_mac from env string, with default
 */
static void get_mac(char *envvar, unsigned char *def)
{
	char str[20];
	char *env = getenv(envvar);

	if (!env) {
		sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
			def[0], def[1], def[2], def[3], def[4], def[5]);
		printf("### Setting environment from ROM MAC address = \"%s\"\n", str);
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
	struct ventana_board_info *info = read_eeprom(0);
  char *serial = getenv("serial#");

//printf("%s\n", __func__);
  if (serial) {
    serialnr->high = 0;
    serialnr->low = simple_strtoul(serial, NULL, 10);
  } else if (info) {
    serialnr->high = 0;
		serialnr->low = info->serial;
	} else {
    serialnr->high = 0;
    serialnr->low = 0;
  }
}
#endif

#ifdef CONFIG_MXC_SPI
iomux_v3_cfg_t const ecspi1_pads[] = {
	/* SS1 */
	MX6Q_PAD_EIM_D19__GPIO_3_19   | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6Q_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
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
//	unsigned short val;
//printf("\n%s: port%d link=%d addr=%d phy_id=0x%08x\n", __func__, phydev->port, phydev->link, phydev->addr, phydev->phy_id);

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

static void setup_gpio(void)
{
	imx_iomux_v3_setup_multiple_pads(gpio_pads,
					 ARRAY_SIZE(gpio_pads));

	/* configure outputs */
	gpio_direction_output(IMX_GPIO_NR(2, 11), 0); // UART2_EN#
	gpio_direction_output(IMX_GPIO_NR(4,  5), 0); // DIOI2C_DIS#

	/* default LED's off */
	gpio_direction_output(IMX_GPIO_NR(4, 10), 1); // red
	gpio_direction_output(IMX_GPIO_NR(4, 15), 1); // loc
	gpio_direction_output(IMX_GPIO_NR(4, 6), 1);  // grn

	/* default DIO's */
	gpio_direction_input(IMX_GPIO_NR(1, 9));
	gpio_direction_input(IMX_GPIO_NR(1, 19));
	gpio_direction_input(IMX_GPIO_NR(2, 9));
	gpio_direction_input(IMX_GPIO_NR(2, 10));

	/* default Expansion board DIO's */
	gpio_direction_output(IMX_GPIO_NR(4, 7), 0); // PWREN#
	gpio_direction_input(IMX_GPIO_NR(4, 9));     // IRQ#
}

static int setup_pcie(void)
{
	/* enable clock and toggle PCI_RST# */
	gpio_direction_output(IMX_GPIO_NR(1, 29), 0); // PCIESWT_RST#
	gpio_direction_output(IMX_GPIO_NR(1, 20), 1); // PCIECK_SSON

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
	MX6Q_PAD_SD2_CMD__GPIO_1_11 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define MIPI_BACKLIGHT_GP IMX_GPIO_NR(1, 11)

	/* Backlight CABEN on LVDS connector: J6 */
	MX6Q_PAD_SD2_CLK__GPIO_1_10 | MUX_PAD_CTRL(NO_PAD_CTRL),

#define LVDS_BACKLIGHT_GP IMX_GPIO_NR(1, 10)
};

static struct pwm_device pwm3 = {
	.pwm_id = 3,
	.pwmo_invert = 0,
};

/* Parallel RGB Input (Analog Video In) */
static iomux_v3_cfg_t const vidin_pads[] = {
	MX6Q_PAD_EIM_DA2__IPU2_CSI1_D_7,
	MX6Q_PAD_EIM_DA3__IPU2_CSI1_D_6,
	MX6Q_PAD_EIM_DA4__IPU2_CSI1_D_5,
	MX6Q_PAD_EIM_DA5__IPU2_CSI1_D_4,
	MX6Q_PAD_EIM_DA6__IPU2_CSI1_D_3,
	MX6Q_PAD_EIM_DA7__IPU2_CSI1_D_2,
	MX6Q_PAD_EIM_DA8__IPU2_CSI1_D_1,
	MX6Q_PAD_EIM_DA9__IPU2_CSI1_D_0,
};

/* Parallel RGB Output (Analog Video Out) */
static iomux_v3_cfg_t const vidout_pads[] = {
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,
	MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,
	MX6Q_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6Q_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6Q_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6Q_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6Q_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6Q_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6Q_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6Q_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6Q_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6Q_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6Q_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6Q_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6Q_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6Q_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6Q_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6Q_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
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
//printf("%s: %d\n", __func__, __raw_readb(HDMI_ARB_BASE_ADDR+HDMI_PHY_STAT0) & HDMI_PHY_HPD);
	return __raw_readb(HDMI_ARB_BASE_ADDR+HDMI_PHY_STAT0) & HDMI_PHY_HPD;
}

static void enable_hdmi(struct display_info_t const *dev)
{
	u8 reg;
	printf("%s: setup HDMI monitor\n", __func__);
	reg = __raw_readb(
			HDMI_ARB_BASE_ADDR
			+HDMI_PHY_CONF0);
	reg |= HDMI_PHY_CONF0_PDZ_MASK;
	__raw_writeb(reg,
		     HDMI_ARB_BASE_ADDR
			+HDMI_PHY_CONF0);
	udelay(3000);
	reg |= HDMI_PHY_CONF0_ENTMDS_MASK;
	__raw_writeb(reg,
		     HDMI_ARB_BASE_ADDR
			+HDMI_PHY_CONF0);
	udelay(3000);
	reg |= HDMI_PHY_CONF0_GEN2_TXPWRON_MASK;
	__raw_writeb(reg,
		     HDMI_ARB_BASE_ADDR
			+HDMI_PHY_CONF0);
	__raw_writeb(HDMI_MC_PHYRSTZ_ASSERT,
		     HDMI_ARB_BASE_ADDR+HDMI_MC_PHYRSTZ);
}

static int detect_i2c(struct display_info_t const *dev)
{
//printf("%s bus=%d addr=0x%02x %d\n", __func__, dev->bus, dev->addr, ((0 == i2c_set_bus_num(dev->bus)) && (0 == i2c_probe(dev->addr))));
	return ((0 == i2c_set_bus_num(dev->bus))
		&&
		(0 == i2c_probe(dev->addr)));
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;

	/* set CH0 data width to 24bit (IOMUXC_GPR2:5 0=18bit, 1=24bit) */
//printf("%s\n", __func__);
	u32 reg = readl(&iomux->gpr[2]);
//printf("%p=%x\n", &iomux->gpr[2], reg);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
	writel(reg, &iomux->gpr[2]);

	/* Disable CABC:
	 * when enabled this feature sets backlight automatically according to content
	 * which may cause annoying unstable backlight issue
	 */
	gpio_direction_output(LVDS_BACKLIGHT_GP, 0);

	/* pwm */
	imx_iomux_v3_setup_pad(MX6Q_PAD_SD1_CMD__PWM4_PWMO | MUX_PAD_CTRL(PAD_CTL_DSE_240ohm));
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
	/* HannStar HSD100PXN1-A00 with egalx_ts controller */
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
#if 0
} }, {
	.bus	= 2,
	.addr	= 0x38,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "wsvga-lvds",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 600,
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
	.bus	= 2,
	.addr	= 0x2a,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= detect_i2c,
	.enable	= enable_vidout,
	.mode	= {
		.name           = "wvga-rgb",
		.refresh        = 57,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 37037,
		.left_margin    = 40,
		.right_margin   = 60,
		.upper_margin   = 10,
		.lower_margin   = 10,
		.hsync_len      = 20,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
#endif
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
			printf("Display: %s (%ux%u)\n",
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
	__raw_writeb(HDMI_MC_PHYRSTZ_DEASSERT,
		     HDMI_ARB_BASE_ADDR+HDMI_MC_PHYRSTZ);

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
//printf("Found PFUZE100! deviceid=%x,revid=%x\n", value, rev_id);
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

#ifdef CONFIG_PREBOOT
struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
	{"back",	IMX_GPIO_NR(2, 2),	'B'},
	{"home",	IMX_GPIO_NR(2, 4),	'H'},
	{"menu",	IMX_GPIO_NR(2, 1),	'M'},
	{"search",	IMX_GPIO_NR(2, 3),	'S'},
	{"volup",	IMX_GPIO_NR(7, 13),	'V'},
	{"voldown",	IMX_GPIO_NR(4, 5),	'v'},
};

/*
 * generate a null-terminated string containing the buttons pressed
 * returns number of keys pressed
 */
static int read_keys(char *buf)
{
	int i, numpressed = 0;
	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (!gpio_get_value(buttons[i].gpnum))
			buf[numpressed++] = buttons[i].ident;
	}
	buf[numpressed] = '\0';
	return numpressed;
}

static int do_kbd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char envvalue[ARRAY_SIZE(buttons)+1];
	int numpressed = read_keys(envvalue);
	setenv("keybd", envvalue);
	return numpressed == 0;
}

U_BOOT_CMD(
	kbd, 1, 1, do_kbd,
	"Tests for keypresses, sets 'keybd' environment variable",
	"Returns 0 (true) to shell if key is pressed."
);

static char const kbd_magic_prefix[] = "key_magic";
static char const kbd_command_prefix[] = "key_cmd";

static void preboot_keys(void)
{
	int numpressed;
	char keypress[ARRAY_SIZE(buttons)+1];
	numpressed = read_keys(keypress);
	if (numpressed) {
		char *kbd_magic_keys = getenv("magic_keys");
		char *suffix;
		/*
		 * loop over all magic keys
		 */
		for (suffix = kbd_magic_keys; *suffix; ++suffix) {
			char *keys;
			char magic[sizeof(kbd_magic_prefix) + 1];
			sprintf(magic, "%s%c", kbd_magic_prefix, *suffix);
			keys = getenv(magic);
			if (keys) {
				if (!strcmp(keys, keypress))
					break;
			}
		}
		if (*suffix) {
			char cmd_name[sizeof(kbd_command_prefix) + 1];
			char *cmd;
			sprintf(cmd_name, "%s%c", kbd_command_prefix, *suffix);
			cmd = getenv(cmd_name);
			if (cmd) {
				setenv("preboot", cmd);
				return;
			}
		}
	}
}
#endif // #ifdef CONFIG_PREBOOT

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
//	{"mmc1",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
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
	setup_gpio();
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
int checkboard(void)
{
	struct ventana_board_info *info;
//printf("%s\n", __func__);

	printf("Gateworks Corporation Copyright 2013\n");

	mdelay(1000); // I believe this is needed to wait for GSC to powerup w/o battery - need to investigate
 	info = read_eeprom(1);
	if (info) {
		printf("Model Number: %s\n", info->model);
		printf("Manufacturer Date: %02x-%02x-%02x%02x\n",
			info->mfgdate[0], info->mfgdate[1],
			info->mfgdate[2], info->mfgdate[3]);
		printf("Serial #: %d\n", info->serial);
	} else {
		printf("Invalid EEPROM - board will not function fully\n");
	}
	//return 1; //hang
	return 0;
}
#endif

/* Set gd->ram_size
 */
int dram_init(void)
{
	struct ventana_board_info *info = read_eeprom(0);

//printf("%s\n", __func__);
	if (info && info->sdram_size > 0 && info->sdram_size < 9) {
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
 
//printf("%s\n", __func__);
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

	return 0;
}

/* late init
 */
int misc_init_r(void)
{
	struct ventana_board_info *info = read_eeprom(0);

//printf("%s\n", __func__);
/*
	char *env;
	char str[20];
 	env = getenv("ethaddr");
	if (!env && info) {
		sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
			info->mac0[0], info->mac0[1],
			info->mac0[2], info->mac0[3],
			info->mac0[4], info->mac0[5]) ;
		printf("### Setting environment from ROM MAC address = \"%s\"\n", str);
		setenv("ethaddr", str);
		setenv("eth0addr", str);
	}
*/

	if (info) {
		get_mac("ethaddr", info->mac0);
		get_mac("eth1addr", info->mac1);
	}

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
static int disable_node(void *blob, const char *name, const char *path)
{
	int rc = fdt_find_and_setprop(blob, path, "status",
				"disabled", sizeof("disabled"), 1);
	if (rc) {
		printf("Unable to update status property in %s node: err=%s\n",
			name, fdt_strerror(rc));
	}
	return rc;
}

void ft_board_setup(void *blob, bd_t * bd)
{
	struct ventana_board_info *info = read_eeprom(0);
#if 0
	struct node_info nodes[] = {
		{ "sst,w25q256",    MTD_DEV_TYPE_NAND, },
	};
#endif

	if (getenv("fdt_noauto")) {
		printf("skiping ft_board_setup\n");
		return;
	}

	if (!info) {
		printf("invalid board info: Leaving DTB fully enabled\n");
		return;
	}

	printf("Adjusting DTB per EEPROM configuraiton...\n");

	/* Note that fdt_fixup_ethernet is called in arm/lib/bootm before this
	 * which sets mac-address and local-mac-address properties of
	 * ethernet<n> aliases to ethaddr...eth<n>addr env
	 */

#if 0
	/* MTD partitions */
	fdt_fixup_mtdparts(blob, nodes, ARRAY_SIZE(nodes));
#endif

	/* GPIO config: dio{0-3}
	 * TODO: setup pinmux for GPIO vs PWM depending on info->config_dio<n> and/or env
	 */

	/* SDRAM config: sdram_{size,speed,width} */

	/* CPU config: cpu_{speed,type} */

	/* FLASH config: nor_flash_size, spi_flash_size */

	/* Peripheral Config */
	if (!info->config_eth0) {
		disable_node(blob, "fec", "/soc/aips-bus@02100000/ethernet@02188000");
	}
	if (!info->config_eth1) { }
	if (!info->config_sata) { }
	if (!info->config_ssi0) {
		disable_node(blob, "ssi1",
			"/soc/aips-bus@02000000/spba-bus@02000000/ssi@02028000");
	}
	if (!info->config_ssi1) {
		disable_node(blob, "ssi2",
			"/soc/aips-bus@02000000/spba-bus@02000000/ssi@0202c000");
	}
	if (!info->config_ipu0) {
		disable_node(blob, "ipu1", "/soc/ipu@02400000");
	}
	if (!info->config_ipu1) {
		disable_node(blob, "ipu2", "/soc/ipu@02800000");
	}
	if (!info->config_mipi_csi) {
		disable_node(blob, "mipi_csi", "/soc/aips-bus@02100000/mipi@021dc000");
	}
	if (!info->config_mipi_dsi) {
		disable_node(blob, "mipi_dsi", "/soc/aips-bus@02100000/mipi@021e0000");
	}
	if (!info->config_tzasc0) {
		disable_node(blob, "tzasc1", "/soc/aips-bus@02100000/tzasc@021d0000");
	}
	if (!info->config_tzasc1) {
		disable_node(blob, "tzasc2", "/soc/aips-bus@02100000/tzasc@021d4000");
	}
	if (!info->config_caam) {
		disable_node(blob, "caam", "/soc/aips-bus@02100000/caam@02100000");
	}
	if (!info->config_flexcan) {
		disable_node(blob, "flexcan", "/soc/aips-bus@02000000/flexcan@02094000");
	}
	if (!info->config_vpu) {
		disable_node(blob, "vpu", "/soc/aips-bus@02000000/vpu@02040000");
	}
	if (!info->config_i2c0) {
		disable_node(blob, "i2c1", "/soc/aips-bus@02100000/i2c@021a0000");
	}
	if (!info->config_i2c1) {
		disable_node(blob, "i2c2", "/soc/aips-bus@02100000/i2c@021a4000");
	}
	if (!info->config_i2c2) {
		disable_node(blob, "i2c3", "/soc/aips-bus@02100000/i2c@021a8000");
	}
	if (!info->config_pcie) {
		disable_node(blob, "pcie", "/soc/aips-bus@02100000/pcie@01ffc000");
	}
	if (!info->config_usb0) {
		disable_node(blob, "usbh3", "/soc/aips-bus@02100000/usb@02184600");
	}
	if (!info->config_usb1) {
		disable_node(blob, "usbotg",
			"/soc/aips-bus@02000000/iomuxc@020e0000/usbotg");
	}
	if (!info->config_sd0) {
		disable_node(blob, "sd1", "/soc/aips-bus@02100000/usdhc@02190000");
	}
	if (!info->config_sd1) {
		disable_node(blob, "sd2", "/soc/aips-bus@02100000/usdhc@02194000");
	}
	if (!info->config_sd2) {
		disable_node(blob, "sd3", "/soc/aips-bus@02100000/usdhc@02198000");
	}
	if (!info->config_sd3) {
		disable_node(blob, "sd4", "/soc/aips-bus@02100000/usdhc@0219c000");
	}
	if (!info->config_uart0) {
		disable_node(blob, "uart1",
			"/soc/aips-bus@02000000/spba-bus@02000000/serial@02020000");
	}
	if (!info->config_uart1) {
printf("leaving uart2 enabled - must be invalid eeprom\n");
/*
		disable_node(blob, "uart2",
			"/soc/aips-bus@02100000/serial@021e8000");
*/
	}
	if (!info->config_uart2) {
		disable_node(blob, "uart3",
			"/soc/aips-bus@02100000/serial@021ec000");
	}
	if (!info->config_uart3) {
		disable_node(blob, "uart4",
			"/soc/aips-bus@02100000/serial@021f0000");
	}
	if (!info->config_uart4) {
		disable_node(blob, "uart5",
			"/soc/aips-bus@02100000/serial@021f4000");
	}
	if (!info->config_espci0) {
printf("leaving spi1 enabled - must be invalid eeprom\n");
/*
		disable_node(blob, "spi1",
			"/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02008000");
*/
	}
	if (!info->config_espci1) {
		disable_node(blob, "spi2",
			"/soc/aips-bus@02000000/spba-bus@02000000/ecspi@0200c000");
	}
	if (!info->config_espci2) {
		disable_node(blob, "spi3",
			"/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02010000");
	}
	if (!info->config_espci3) {
		disable_node(blob, "spi4",
			"/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02014000");
	}
	if (!info->config_espci4) {
		disable_node(blob, "spi5",
			"/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02018000");
	}
}
#endif /* defined(CONFIG_OF_FLAT_TREE) && defined(CONFIG_OF_BOARD_SETUP) */
