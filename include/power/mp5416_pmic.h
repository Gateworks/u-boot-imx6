// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2019 Gateworks Corporation
 *  Tim Harvey <tharvey@gateworks.com>
 */

#ifndef __MP5416_PMIC_H_
#define __MP5416_PMIC_H_

/* MP5416 registers */
enum {
	MP5416_CTL0		= 0x00,
	MP5416_CTL1		= 0x01,
	MP5416_CTL2		= 0x02,
	MP5416_ILIMIT		= 0x03,
	MP5416_VSET_SW1		= 0x04,
	MP5416_VSET_SW2		= 0x05,
	MP5416_VSET_SW3		= 0x06,
	MP5416_VSET_SW4		= 0x07,
	MP5416_VSET_LDO2	= 0x08,
	MP5416_VSET_LDO3	= 0x09,
	MP5416_VSET_LDO4	= 0x0a,
	MP5416_VSET_LDO5	= 0x0b,
	MP5416_STATUS1		= 0x0d,
	MP5416_STATUS2		= 0x0e,
	MP5416_STATUS3		= 0x0f,
	MP5416_ID2		= 0x11,
	MP5416_NUM_OF_REGS	= 0x12,
};

/*
 * SW Configuration
 */
#define MP5416_VSET_EN          BIT(7)

int power_mp5416_init(unsigned char bus);
#endif
