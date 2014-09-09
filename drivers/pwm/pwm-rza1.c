/*
 * RZ/A1 PWM driver
 *
 * Copyright (C) 2014  Renesas Solutions Corp.
 *
 * Based on pwm-pxa.c:
 * 2008-02-13	initial version
 *      eric miao <eric.miao@marvell.com>
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>

#include <asm/div64.h>

#define HAS_SECONDARY_PWM	0x10

static DEFINE_RAW_SPINLOCK(mtu2_lock);

static const struct platform_device_id pwm_id_table[] = {
	/*   PWM    has_secondary_pwm? */
	{ "rza1-pwm", 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, pwm_id_table);

/* PWM registers and bits definitions */

#define TCR_4		(0x001)
#define TMDR_4		(0x003)
#define TIORH_4		(0x006)
#define TIORL_4		(0x007)
#define TIER_4		(0x009)
#define TSR_4		(0x02D)
#define TCNT_4		(0x012)
#define TGRA_4		(0x01C)
#define TGRB_4		(0x01E)

#define TOER		(0x00A)

#define TSTR		(0x000)
#define TSYR		(0x001)
#define TRWER		(0x004)

#define TGR_MAX_VALUE (0xFFFF)

struct rza1_pwm_chip {
	struct pwm_chip	chip;
	struct device	*dev;

	struct clk	*clk;
	int		clk_enabled;
	void __iomem	*mmio_base;
	void __iomem	*mmio_share_base;
};

static inline struct rza1_pwm_chip *to_rza1_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct rza1_pwm_chip, chip);
}

static int rza1_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			  int duty_ns, int period_ns)
{
	struct rza1_pwm_chip *pc = to_rza1_pwm_chip(chip);
	unsigned long long c, clk_rate;
	unsigned long period_cycles, prescale, pv, dc;
	int rc;
	unsigned char  reg_b;
	unsigned long flags;

	clk_rate = clk_get_rate(pc->clk);
	c = clk_rate * period_ns;
	do_div(c, 1000000000);
	period_cycles = c;

	if (period_cycles < 1)
		period_cycles = 1;

	c = clk_rate * duty_ns;
	do_div(c, 1000000000);

	/* prescale 1,4,16,64 */
	/* 1 Dividing */
	if ((period_cycles / TGR_MAX_VALUE) == 0) {
		prescale = 0;
		pv = period_cycles;
		dc = c;
	/* 4 Dividing */
	} else if ((period_cycles / (TGR_MAX_VALUE * 4)) == 0) {
		prescale = 1;
		pv = period_cycles / 4;
		dc = c / 4;
	/* 16 Dividing */
	} else if ((period_cycles / (TGR_MAX_VALUE * 16)) == 0) {
		prescale = 2;
		pv = period_cycles / 16;
		dc = c / 16;
	/* 64 Dividing */
	} else if ((period_cycles / (TGR_MAX_VALUE * 64)) == 0) {
		prescale = 3;
		pv = period_cycles / 64;
		dc = c / 64;
	/* Over 64 Dividing */
	} else {
		dev_err(chip->dev, "rza1-pwm prescale over!!\n");
		return -EINVAL;
	}

	if (duty_ns == period_ns)
		dc = pv;

	/* NOTE: the clock to PWM has to be enabled first
	 * before writing to the registers
	 */
	rc = clk_prepare_enable(pc->clk);
	if (rc < 0)
		return rc;

	/* PWM mode setting */
	/* Timer count stop */
	/*  TSTR:CST4 = 0 */
	raw_spin_lock_irqsave(&mtu2_lock, flags);
	reg_b = ioread8(pc->mmio_share_base + TSTR);
	reg_b &= 0x7F;	/* CST4 = 0 */
	iowrite8(reg_b, pc->mmio_share_base + TSTR);
	raw_spin_unlock_irqrestore(&mtu2_lock, flags);

	/* Counter 0 clear */
	iowrite16(0, pc->mmio_base + TCNT_4);

	/* Selection of a counter clock */
	/*  TCR:CKEG = 00b(counts by a rising edge) */
	/*  TCR:TPSC = prescale */
	/* Selection of a counter clear factor */
	/*  TCR:CCLR = 001b(counter clearance at TGRA) */
	iowrite8((0x20 | prescale), pc->mmio_base + TCR_4);

	/* Selection of a waveform output level */
	if (duty_ns <= 0) {	/* PWM OFF */
		/* TIOR:IOB = 0001b(initial output of TRGB = 0, Compar = 0) */
		/* TIOR:IOA = 0001b(initial output of TRGA = 0, Compar = 0) */
		iowrite8(0x11, pc->mmio_base + TIORH_4);
	} else {
		/* TIOR:IOB = 0101b(initial output of TRGB = 1, Compar = 0) */
		/* TIOR:IOA = 0110b(initial output of TRGA = 1, Compar = 1) */
		iowrite8(0x56, pc->mmio_base + TIORH_4);
	}

	/* setup of TGR */
	/*  TRGA Setting(PWM period Setting) */
	/*  TRGB Setting(PWM duty Setting) */
	iowrite16(pv, pc->mmio_base + TGRA_4);
	iowrite16(dc, pc->mmio_base + TGRB_4);

	/* setup in PWM mode */
	/*  TMDR:MD = 0010b(PWM mode 1) */
	iowrite8(0x02, pc->mmio_base + TMDR_4);

	/* Output Enable */
	/*  TOER:OE4A = 1(TIOC4A output enable) */
	reg_b = ioread8(pc->mmio_base + TOER);
	reg_b |= 0x02;	/* OE4A = 1 */
	iowrite8(reg_b, pc->mmio_base + TOER);

	/* Timer count start */
	/*  TSTR:CST4 = 1 */
	if (duty_ns > 0) {	/* PWM ON */
		raw_spin_lock_irqsave(&mtu2_lock, flags);
		reg_b = ioread8(pc->mmio_share_base + TSTR);
		reg_b |= 0x80;	/* CST4 = 1 */
		iowrite8(reg_b, pc->mmio_share_base + TSTR);
		raw_spin_unlock_irqrestore(&mtu2_lock, flags);
	}

	clk_disable_unprepare(pc->clk);

	return 0;
}

static int rza1_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rza1_pwm_chip *pc = to_rza1_pwm_chip(chip);
	int rc = 0;

	if (!pc->clk_enabled) {
		rc = clk_prepare_enable(pc->clk);
		if (!rc)
			pc->clk_enabled++;
	}
	return rc;
}

static void rza1_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rza1_pwm_chip *pc = to_rza1_pwm_chip(chip);

	if (pc->clk_enabled) {
		clk_disable_unprepare(pc->clk);
		pc->clk_enabled--;
	}
}

static struct pwm_ops rza1_pwm_ops = {
	.config = rza1_pwm_config,
	.enable = rza1_pwm_enable,
	.disable = rza1_pwm_disable,
	.owner = THIS_MODULE,
};

static int pwm_probe(struct platform_device *pdev)
{
	const struct platform_device_id *id = platform_get_device_id(pdev);
	struct rza1_pwm_chip *pwm;
	struct resource *r;
	int ret = 0;

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (pwm == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pwm->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pwm->clk))
		return PTR_ERR(pwm->clk);

	pwm->clk_enabled = 0;

	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &rza1_pwm_ops;
	pwm->chip.base = -1;
	pwm->chip.npwm = (id->driver_data & HAS_SECONDARY_PWM) ? 2 : 1;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		return -ENODEV;
	}

	pwm->mmio_base = devm_request_and_ioremap(&pdev->dev, r);
	if (pwm->mmio_base == NULL)
		return -EADDRNOTAVAIL;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (r == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		return -ENODEV;
	}

	pwm->mmio_share_base = ioremap(r->start, resource_size(r));
	if (pwm->mmio_share_base == NULL)
		return -EADDRNOTAVAIL;

	ret = pwmchip_add(&pwm->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "RZ/A1 PWM Driver probed\n");

	platform_set_drvdata(pdev, pwm);
	return 0;
}

static int pwm_remove(struct platform_device *pdev)
{
	struct rza1_pwm_chip *chip;

	chip = platform_get_drvdata(pdev);
	if (chip == NULL)
		return -ENODEV;

	return pwmchip_remove(&chip->chip);
}

static struct platform_driver pwm_driver = {
	.driver		= {
		.name	= "rza1-pwm",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_probe,
	.remove		= pwm_remove,
	.id_table	= pwm_id_table,
};
module_platform_driver(pwm_driver);

MODULE_ALIAS("platform:rza1-pwm");
MODULE_DESCRIPTION("Renesas RZA1 PWM Driver");
MODULE_LICENSE("GPL v2");
