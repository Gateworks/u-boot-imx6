/*
 *  Copyright (C) 2014 Gateworks Corporation
 *  Tim Harvey <tharvey@gateworks.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#ifndef __LTC3676_PMIC_H_
#define __LTC3676_PMIC_H_

/* LTC3676 registers */
enum {
	LTC3676_BUCK1	= 0x01,
	LTC3676_BUCK2	= 0x02,
	LTC3676_BUCK3	= 0x03,
	LTC3676_BUCK4	= 0x04,
	LTC3676_LDOA	= 0x05,
	LTC3676_LDOB	= 0x06,
	LTC3676_SQD1	= 0x07,
	LTC3676_SQD2	= 0x08,
	LTC3676_CNTRL	= 0x09,
	LTC3676_DVB1A	= 0x0A,
	LTC3676_DVB1B	= 0x0B,
	LTC3676_DVB2A	= 0x0C,
	LTC3676_DVB2B	= 0x0D,
	LTC3676_DVB3A	= 0x0E,
	LTC3676_DVB3B	= 0x0F,
	LTC3676_DVB4A	= 0x10,
	LTC3676_DVB4B	= 0x11,
	LTC3676_MSKIRQ	= 0x12,
	LTC3676_MSKPG	= 0x13,
	LTC3676_USER	= 0x14,
	LTC3676_HRST	= 0x1E,
	LTC3676_CLIRQ	= 0x1F,
	LTC3676_IRQSTAT	= 0x15,
	LTC3676_PGSTATL	= 0x16,
	LTC3676_PGSTATR	= 0x17,
	LTC3676_NUM_OF_REGS = 0x20,
};

/*
 * SW Configuration
 */

/* DVB<x> registers */
#define LTC3676_DVB_MASK	0x1f
#define LTC3676_PGOOD_MASK	(1<<5)
#define LTC3676_REF_SELA	(0<<5)
#define LTC3676_REF_SELB	(1<<5)

/* BUCK<x> registers */
#define LTC3676_MODE_MASK	0x60
#define LTC3676_MODE		5
#define LTC3676_STARTUP		4
#define LTC3676_PHASE		3
#define LTC3676_FREQ		2
#define LTC3676_KEEPALIVE	1
#define LTC3676_RATE		0

enum ltc3676_sw {
	SW1 = 0,
	SW2 = 1,
	SW3 = 2,
	SW4 = 3
};

enum ltc3676_mode {
	PFM = 0, /* pulse-skipping */
	APF = 1, /* burst - auto switches */
	PWM = 2, /* forced continuous */
};

enum ltc3676_phase {
	PHASE1 = 0,
	PHASE2 = 1,
};

enum ltc3676_freq {
	F2250KHZ = 0,
	F1125KHZ = 1,
};

enum ltc3676_rate {
	SLOW = 0,
	FAST = 1,
};

int power_ltc3676_init(unsigned char bus);
void power_ltc3676_swconfig(struct pmic *, enum ltc3676_sw, enum ltc3676_mode,
			    enum ltc3676_phase, enum ltc3676_freq);
#endif
