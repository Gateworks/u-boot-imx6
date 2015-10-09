/*
 * Copyright (C) 2014 Gateworks Corporation
 * Tim Harvey <tharvey@gateworks.com>
 *
 * SPDX-License-Identifier:      GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/ltc3676_pmic.h>

int power_ltc3676_init(unsigned char bus)
{
	static const char name[] = "LTC3676_PMIC";
	struct pmic *p = pmic_alloc();

	if (!p) {
		printf("%s: POWER allocation error!\n", __func__);
		return -ENOMEM;
	}

	p->name = name;
	p->interface = PMIC_I2C;
	p->number_of_regs = LTC3676_NUM_OF_REGS;
	p->hw.i2c.addr = CONFIG_POWER_LTC3676_I2C_ADDR;
	p->hw.i2c.tx_num = 1;
	p->bus = bus;

	return 0;
}

void power_ltc3676_swconfig(struct pmic *p, enum ltc3676_sw sw,
			    enum ltc3676_mode mode, enum ltc3676_phase phase,
			    enum ltc3676_freq freq)
{
	u32 reg;

	pmic_reg_read(p, LTC3676_BUCK1 + sw, &reg);
	reg &= ~LTC3676_MODE_MASK;
	reg |= (mode << LTC3676_MODE);
	reg &= ~(1 << LTC3676_PHASE);
	reg |= (phase << LTC3676_PHASE);
	reg &= ~(1 << LTC3676_FREQ);
	reg |= (freq << LTC3676_FREQ);
	pmic_reg_write(p, LTC3676_BUCK1 + sw, reg);
}
