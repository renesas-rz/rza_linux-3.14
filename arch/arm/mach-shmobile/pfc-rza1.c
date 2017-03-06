/*
 * Copyright (C) 2013-2014  Renesas Solutions Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <asm/io.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include "r7s72100.h"

#define GPIO_CHIP_NAME "RZA1_INTERNAL_PFC"

#define RZA1_BASE	IOMEM(0xfcfe3000)
#define PORT_OFFSET	0x4
#define PORT(p)		(0x0000 + (p) * PORT_OFFSET)
#define PPR(p)		(0x0200 + (p) * PORT_OFFSET)
#define PM(p)		(0x0300 + (p) * PORT_OFFSET)
#define PMC(p)		(0x0400 + (p) * PORT_OFFSET)
#define PFC(p)		(0x0500 + (p) * PORT_OFFSET)
#define PFCE(p)		(0x0600 + (p) * PORT_OFFSET)
#define PFCAE(p)	(0x0a00 + (p) * PORT_OFFSET)
#define PIBC(p)		(0x4000 + (p) * PORT_OFFSET)
#define PBDC(p)		(0x4100 + (p) * PORT_OFFSET)
#define PIPC(p)		(0x4200 + (p) * PORT_OFFSET)

static struct mutex	mutex;

enum {
	REG_PFC = 0,
	REG_PFCE,
	REG_PFCAE,
	REG_NUM,
};

static bool mode_regset[][REG_NUM] = {
	/* PFC,	PFCE,	PFCAE */
	{false,	false,	false	}, /* port mode */
	{false,	false,	false	}, /* alt true */
	{true,	false,	false	}, /* alt 2 */
	{false,	true,	false	}, /* alt 3 */
	{true,	true,	false	}, /* alt 4 */
	{false,	false,	true	}, /* alt 5 */
	{true,	false,	true	}, /* alt 6 */
	{false,	true,	true	}, /* alt 7 */
	{true,	true,	true	}, /* alt 8 */
};

static unsigned int regs_addr[][REG_NUM] = {
	{PFC(0), PFCE(0), PFCAE(0)},
	{PFC(1), PFCE(1), PFCAE(1)},
	{PFC(2), PFCE(2), PFCAE(2)},
	{PFC(3), PFCE(3), PFCAE(3)},
	{PFC(4), PFCE(4), PFCAE(4)},
	{PFC(5), PFCE(5), PFCAE(5)},
	{PFC(6), PFCE(6), PFCAE(6)},
	{PFC(7), PFCE(7), PFCAE(7)},
	{PFC(8), PFCE(8), PFCAE(8)},
	{PFC(9), PFCE(9), PFCAE(9)},
	{PFC(10), PFCE(10), PFCAE(10)},
	{PFC(11), PFCE(11), PFCAE(11)},
};

#if 0 /* Removed because we are using dummy port bits now */
static unsigned int port_nbit[] = {
	16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
};
#endif

static inline int _bit_modify(void __iomem *addr, int bit, bool data)
{
	__raw_writel((__raw_readl(addr) & ~(0x1 << bit)) | (data << bit), addr);
	return 0;
}

static inline int bit_modify(unsigned int addr, int bit, bool data)
{
	return _bit_modify(RZA1_BASE + addr, bit, data);
}

static int set_direction(unsigned int port, int bit, enum pfc_direction dir)
{
	if ((port == 0) && (dir != DIR_IN))	/* p0 is input only */
		return -1;

	if (dir == DIR_IN) {
		bit_modify(PM(port), bit, true);
		bit_modify(PIBC(port), bit, true);
	} else {
		bit_modify(PM(port), bit, false);
		bit_modify(PIBC(port), bit, false);
	}

	return 0;
}

/* return the port number and modify the passed offset to be the bit */
static int get_port_bitshift(int *offset)
{
	unsigned int i;

#if 0 /* Removed because we are using dummy port bits now */
	for (i = 0; *offset >= port_nbit[i]; *offset -= port_nbit[i++])
		if (i > ARRAY_SIZE(port_nbit))
			return -1;
#else
	if (*offset >= GPIO_NR)
		return -1;
	i = *offset / 16;
	*offset = *offset % 16;
#endif
	return i;
}

static int chip_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	int port;
	unsigned int d;

	port = get_port_bitshift(&offset);


	d = __raw_readl(RZA1_BASE + PPR(port));

	return (d &= (0x1 << offset)) ? 1 : 0;
}

static void chip_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	int port;

	port = get_port_bitshift(&offset);
	if (port <= 0)	/* p0 is input only */
		return;

	bit_modify(PORT(port), offset, val);
	return;
}

static int chip_direction_input(struct gpio_chip *chip, unsigned offset)
{
	int port;

	port = get_port_bitshift(&offset);

	mutex_lock(&mutex);
	set_direction(port, offset, DIR_IN);
	mutex_unlock(&mutex);

	return 0;
}

static int chip_direction_output(struct gpio_chip *chip, unsigned offset,
				int val)
{
	int port;

	port = get_port_bitshift(&offset);
	if (port <= 0)	/* case : p0 is input only && negative value*/
		return -1;

	mutex_lock(&mutex);
	bit_modify(PORT(port), offset, val);
	set_direction(port, offset, DIR_OUT);
	mutex_unlock(&mutex);

	return 0;
}

static const char * const gpio_names[] = {
	"P0_0", "P0_1", "P0_2", "P0_3", "P0_4", "P0_5",

	/* dummy entries */
	"P0_6_DMY", "P0_7_DMY", "P0_8_DMY","P0_9_DMY","P0_10_DMY","P0_11_DMY",
	"P0_12_DMY","P0_13_DMY","P0_14_DMY","P0_15_DMY",

	"P1_0", "P1_1", "P1_2", "P1_3", "P1_4", "P1_5", "P1_6", "P1_7", "P1_8",
	"P1_9", "P1_10", "P1_11", "P1_12", "P1_13", "P1_14", "P1_15",
	"P2_0", "P2_1", "P2_2", "P2_3", "P2_4", "P2_5", "P2_6", "P2_7", "P2_8",
	"P2_9", "P2_10", "P2_11", "P2_12", "P2_13", "P2_14", "P2_15",
	"P3_0", "P3_1", "P3_2", "P3_3", "P3_4", "P3_5", "P3_6", "P3_7", "P3_8",
	"P3_9", "P3_10", "P3_11", "P3_12", "P3_13", "P3_14", "P3_15",
	"P4_0", "P4_1", "P4_2", "P4_3", "P4_4", "P4_5", "P4_6", "P4_7", "P4_8",
	"P4_9", "P4_10", "P4_11", "P4_12", "P4_13", "P4_14", "P4_15",
	"P5_0", "P5_1", "P5_2", "P5_3", "P5_4", "P5_5", "P5_6", "P5_7", "P5_8",
	"P5_9", "P5_10", "P5_11", "P5_12", "P5_13", "P5_14", "P5_15",
	"P6_0", "P6_1", "P6_2", "P6_3", "P6_4", "P6_5", "P6_6", "P6_7", "P6_8",
	"P6_9", "P6_10", "P6_11", "P6_12", "P6_13", "P6_14", "P6_15",
	"P7_0", "P7_1", "P7_2", "P7_3", "P7_4", "P7_5", "P7_6", "P7_7", "P7_8",
	"P7_9", "P7_10", "P7_11", "P7_12", "P7_13", "P7_14", "P7_15",
	"P8_0", "P8_1", "P8_2", "P8_3", "P8_4", "P8_5", "P8_6", "P8_7", "P8_8",
	"P8_9", "P8_10", "P8_11", "P8_12", "P8_13", "P8_14", "P8_15",
	"P9_0", "P9_1", "P9_2", "P9_3", "P9_4", "P9_5", "P9_6", "P9_7",

	/* dummy entires */
	"P9_8_DMY", "P9_9_DMY", "P9_10_DMY", "P9_11_DMY", "P9_12_DMY", "P9_13_DMY",
	"P9_14_DMY", "P9_15_DMY",

	"P10_0", "P10_1", "P10_2", "P10_3", "P10_4", "P10_5", "P10_6", "P10_7",
	"P10_8", "P10_9", "P10_10", "P10_11", "P10_12", "P10_13", "P10_14",
	"P10_15",
	"P11_0", "P11_1", "P11_2", "P11_3", "P11_4", "P11_5", "P11_6", "P11_7",
	"P11_8", "P11_9", "P11_10", "P11_11", "P11_12", "P11_13", "P11_14",
	"P11_15",
};

static struct gpio_chip chip = {
	.label = GPIO_CHIP_NAME,
	.names = gpio_names,
	.base = 0,
	.ngpio = GPIO_NR,

	.get = chip_gpio_get,
	.set = chip_gpio_set,

	.direction_input = chip_direction_input,
	.direction_output = chip_direction_output,
};

int r7s72100_pinmux_setup(void)
{
	int retval;
	mutex_init(&mutex);
	retval = gpiochip_add(&chip);
	return retval;
}

static int set_mode(unsigned int port, int bit, int mode)
{
	unsigned int reg;

	for (reg = REG_PFC; reg < REG_NUM; reg++)
		bit_modify(regs_addr[port][reg], bit, mode_regset[mode][reg]);

	return 0;
}

/*
 * @pinnum: a pin number.
 * @mode:   port mode or alternative N mode.
 * @dir:    Kind of I/O mode and data direction and PBDC and Output Level.
 *          PIPC enable SoC IP to control a direction.
 */
int r7s72100_pfc_pin_assign(enum pfc_pin_number pinnum, enum pfc_mode mode,
			enum pfc_direction dir)
{
	int port, bit = (int)pinnum;

	port = get_port_bitshift(&bit);

	/* Error is less than 0 port */
	if (port < 0)
		return -1;

	/* Port 0 there is only PMC and PIBC control register */
	if (port == 0) {
		/* Port initialization */
		bit_modify(PIBC(port), bit, false);	/* Input buffer block */
		bit_modify(PMC(port), bit, false);	/* Port mode */

		/* Port Mode */
		if (mode == PMODE) {
			if (dir == DIR_IN) {
				/* PIBC Setting : Input buffer allowed */
				bit_modify(PIBC(port), bit, true);
			} else {
				return -1;	/* P0 Portmode is input only */
			}
		/* Alternative Mode */
		} else {
			if ((bit == P0_4) || (bit == P0_5)) {
				/* PMC Setting : Alternative mode */
				bit_modify(PMC(port), bit, true);
			} else {
				return -1;	/* P0 Altmode P0_4,P0_5 only */
			}
		}
		return 0;
	}

	/* Port initialization */
	bit_modify(PIBC(port), bit, false);	/* Inputbuffer block */
	bit_modify(PBDC(port), bit, false);	/* Bidirection disabled */
	bit_modify(PM(port), bit, true);	/* Input mode(output disabled)*/
	bit_modify(PMC(port), bit, false);	/* Port mode */
	bit_modify(PIPC(port), bit, false);	/* software I/Ocontrol. */

	/* PBDC Setting */
	if ((dir == DIIO_PBDC_EN) || (dir == SWIO_OUT_PBDCEN))
		bit_modify(PBDC(port), bit, true);	/* Bidirection enable */

	/* Port Mode */
	if (mode == PMODE) {
		if (dir == DIR_IN) {
			bit_modify(PIBC(port), bit, true); /*Inputbuffer allow*/
			bit_modify(PM(port), bit, true);
		} else if (dir == PORT_OUT_LOW) {
			bit_modify(PORT(port), bit, false); /*Output low level*/
			bit_modify(PM(port), bit, false);  /* Output mode */
		} else if (dir == PORT_OUT_HIGH) {
			bit_modify(PORT(port), bit, true); /*Output high level*/
			bit_modify(PM(port), bit, false);  /* Output mode */
		} else {
			return -1;
		}
	/* direct I/O control & software I/Ocontrol */
	} else {
		/* PFC,PFCE,PFCAE Setting */
		set_mode(port, bit, mode); /* Alternative Function Select */

		/* PIPC Setting */
		if ((dir == DIIO_PBDC_DIS) || (dir == DIIO_PBDC_EN))
			bit_modify(PIPC(port), bit, true); /* direct I/O cont */
		/* PMC Setting */
		bit_modify(PMC(port), bit, true);	/* Alternative Mode */

		/* PM Setting : Output mode (output enabled)*/
		if ((dir == SWIO_OUT_PBDCDIS) || (dir == SWIO_OUT_PBDCEN))
			bit_modify(PM(port), bit, false);
	}

	return 0;
}
EXPORT_SYMBOL(r7s72100_pfc_pin_assign);
