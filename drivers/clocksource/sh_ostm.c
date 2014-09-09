/*
 * RZ/A1 Timer Driver - OSTM
 *
 * Copyright (C) 2014 Renesas Solutions Corp.
 *
 * Based on drivers/clocksource/sh_mtu2.c
 *  Copyright (C) 2009 Magnus Damm
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/sched_clock.h>
#include <clocksource/sh_ostm.h>

/*
 * [TODO] This driver doesn't support Power Management.
 */

struct rza1_ostm_clksrc {
	int irq;			/* unused */
	struct clk *clk;
	unsigned long rate;
	void __iomem *base;
};

struct rza1_ostm_clkevt {
	int irq;
	struct clk *clk;
	unsigned long rate;
	void __iomem *base;
	int mode;
	unsigned long ticks_per_jiffy;
	struct irqaction irqaction;
	struct clock_event_device evt;
};

struct rza1_ostm_priv {
	struct platform_device *pdev;
	struct rza1_ostm_clksrc clksrc;
	struct rza1_ostm_clkevt clkevt;
};

static struct rza1_ostm_priv *rza1_ostm_priv;
static void __iomem *system_clock;

/* OSTM REGISTERS */
#define	OSTM_CMP		0x000	/* RW,32 */
#define	OSTM_CNT		0x004	/* R,32 */
#define	OSTM_TE			0x010	/* R,8 */
#define	OSTM_TS			0x014	/* W,8 */
#define	OSTM_TT			0x018	/* W,8 */
#define	OSTM_CTL		0x020	/* RW,8 */

#define	TE			0x01
#define	TS			0x01
#define	TT			0x01
#define	CTL_PERIODIC		0x00
#define	CTL_ONESHOT		0x02
#define	CTL_FREERUN		0x02

/*********************************************************************/
/*
 * Setup clock-source device (which is ostm.0)
 * as free-running mode.
 */
static int __init rza1_ostm_init_clksrc(struct rza1_ostm_priv *priv)
{
	struct rza1_ostm_pdata *pdata;
	struct rza1_ostm_clksrc *cs;
	struct resource *res;
	int ret = -ENXIO;

	pdata = priv->pdev->dev.platform_data;
	cs = &priv->clksrc;

	res = platform_get_resource(priv->pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&priv->pdev->dev, "failed to get I/O memory\n");
		goto err;
	}
	cs->base = ioremap_nocache(res->start, resource_size(res));
	if (!cs->base) {
		dev_err(&priv->pdev->dev, "failed to remap I/O memory\n");
		goto err;
	}

	cs->irq = platform_get_irq(priv->pdev, 0);
	if (cs->irq < 0) {
		dev_err(&priv->pdev->dev, "failed to get irq\n");
		goto err_iounmap;
	}

	cs->clk = clk_get_sys(pdata->clksrc.name, NULL);
	if (!cs->clk) {
		dev_err(&priv->pdev->dev, "failed to get clock\n");
		goto err_iounmap;
	}
	cs->rate = clk_get_rate(cs->clk);
	ret = clk_enable(cs->clk);
	if (ret) {
		dev_err(&priv->pdev->dev, "failed to enable clock\n");
		goto err_iounmap;
	}

	if (ioread8(cs->base + OSTM_TE) & TE) {
		iowrite8(TT, cs->base + OSTM_TT);
		while (ioread8(cs->base + OSTM_TE) & TE)
			;
	}
	iowrite32(0, cs->base + OSTM_CMP);
	iowrite8(CTL_FREERUN, cs->base + OSTM_CTL);
	iowrite8(TS, cs->base + OSTM_TS);

	clocksource_mmio_init(cs->base + OSTM_CNT,
			"ostm_clksrc", cs->rate,
			pdata->clksrc.rating, 32, clocksource_mmio_readl_up);

	return ret;	/* ret == 0 */

err_iounmap:
	iounmap(cs->base);
err:
	return ret;
}

/*********************************************************************/
/*
 * Setup sched_clock using clocksource device.
 */
static u32 notrace rza1_ostm_read_sched_clock(void)
{
	return ioread32(system_clock);
}

static int __init rza1_ostm_init_sched_clock(struct rza1_ostm_clksrc *cs)
{
	unsigned long flags;

	system_clock = cs->base + OSTM_CNT;
	local_irq_save(flags);
	local_irq_disable();
	setup_sched_clock(rza1_ostm_read_sched_clock, 32, cs->rate);
	local_irq_restore(flags);
	return 0;
}

/*********************************************************************/
/*
 * Setup clock-event device (which is ostm.1)
 * (interrupt-driven).
 */
static void rza1_ostm_clkevt_timer_stop(struct rza1_ostm_clkevt *clkevt)
{
	if (ioread8(clkevt->base + OSTM_TE) & TE) {
		iowrite8(TT, clkevt->base + OSTM_TT);
		while (ioread8(clkevt->base + OSTM_TE) & TE)
			;
	}
}

static int rza1_ostm_clkevt_set_next_event(unsigned long delta,
		struct clock_event_device *evt)
{
	struct rza1_ostm_clkevt *clkevt = &rza1_ostm_priv->clkevt;

	rza1_ostm_clkevt_timer_stop(clkevt);

	iowrite32(delta, clkevt->base + OSTM_CMP);
	iowrite8(CTL_ONESHOT, clkevt->base + OSTM_CTL);
	iowrite8(TS, clkevt->base + OSTM_TS);

	return 0;
}

static void rza1_ostm_clkevt_set_mode(enum clock_event_mode mode,
		struct clock_event_device *evt)
{
	struct rza1_ostm_clkevt *clkevt = &rza1_ostm_priv->clkevt;

	switch (mode) {
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_ONESHOT:
		rza1_ostm_clkevt_timer_stop(clkevt);
		break;
	case CLOCK_EVT_MODE_PERIODIC:
		iowrite32(clkevt->ticks_per_jiffy - 1, clkevt->base + OSTM_CMP);
		iowrite8(CTL_PERIODIC, clkevt->base + OSTM_CTL);
		iowrite8(TS, clkevt->base + OSTM_TS);
		break;
	case CLOCK_EVT_MODE_RESUME:
	default:
		break;
	}
	clkevt->mode = mode;
}

static irqreturn_t rza1_ostm_timer_interrupt(int irq, void *dev_id)
{
#if 1
	struct rza1_ostm_clkevt *clkevt = dev_id;

	if (clkevt->mode == CLOCK_EVT_MODE_ONESHOT)
		rza1_ostm_clkevt_timer_stop(clkevt);

	if (clkevt->evt.event_handler)
		clkevt->evt.event_handler(&clkevt->evt);
	return IRQ_HANDLED;
#else
	struct clock_event_device *evt = dev_id;
	struct rza1_ostm_clkevt *clkevt;

	clkevt = container_of(evt, struct rza1_ostm_clkevt, evt);
	if (clkevt->mode == CLOCK_EVT_MODE_ONESHOT)
		rza1_ostm_clkevt_timer_stop(clkevt);

	if (evt->event_handler)
		evt->event_handler(evt);
	return IRQ_HANDLED;
#endif
}

static int __init rza1_ostm_init_clkevt(struct rza1_ostm_priv *priv)
{
	struct rza1_ostm_pdata *pdata;
	struct rza1_ostm_clkevt *ce;
	struct resource *res;
	struct clock_event_device *evt;
	int ret = -ENXIO;

	pdata = priv->pdev->dev.platform_data;
	ce = &priv->clkevt;

	res = platform_get_resource(priv->pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&priv->pdev->dev, "failed to get I/O memory\n");
		goto err;
	}
	ce->base = ioremap_nocache(res->start, resource_size(res));
	if (!ce->base) {
		dev_err(&priv->pdev->dev, "failed to remap I/O memory\n");
		goto err;
	}

	ce->irq = platform_get_irq(priv->pdev, 1);
	if (ce->irq < 0) {
		dev_err(&priv->pdev->dev, "failed to get irq\n");
		goto err_iounmap;
	}

	ce->clk = clk_get_sys(pdata->clkevt.name, NULL);
	if (!ce->clk) {
		dev_err(&priv->pdev->dev, "failed to get clock\n");
		goto err_iounmap;
	}
	ce->rate = clk_get_rate(ce->clk);
	ret = clk_enable(ce->clk);
	if (ret) {
		dev_err(&priv->pdev->dev, "failed to enable clock\n");
		goto err_iounmap;
	}

	ce->ticks_per_jiffy = (ce->rate + HZ / 2) / HZ;
	ce->mode = CLOCK_EVT_MODE_UNUSED;
	ce->irqaction.name = "ostm.1";
	ce->irqaction.handler = rza1_ostm_timer_interrupt;
#if 1
	ce->irqaction.dev_id = ce;
#else
	ce->irqaction.dev_id = &priv->clkevt.evt;
#endif
	ce->irqaction.irq = ce->irq;
	ce->irqaction.flags = IRQF_TRIGGER_RISING | IRQF_DISABLED | IRQF_TIMER;
	ret = setup_irq(ce->irq, &ce->irqaction);
	if (ret) {
		dev_err(&priv->pdev->dev, "failed to request irq\n");
		goto err_iounmap;
	}

	evt = &ce->evt;
	evt->name = "ostm";
	evt->features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC;
	evt->set_mode = rza1_ostm_clkevt_set_mode;
	evt->set_next_event = rza1_ostm_clkevt_set_next_event;
	evt->shift = 32;
	evt->rating = pdata->clkevt.rating;
	evt->cpumask = cpumask_of(0);
	clockevents_config_and_register(evt, ce->rate, 0xf, 0xffffffff);

	ret = 0;
	return ret;

err_iounmap:
	iounmap(ce->base);
err:
	return ret;
}

static int __init rza1_ostm_timer_init(struct rza1_ostm_priv *priv)
{
	int ret = 0;

	ret = rza1_ostm_init_clksrc(priv);
	if (ret)
		goto err;

	ret = rza1_ostm_init_sched_clock(&priv->clksrc);
	if (ret)
		goto err;

	ret = rza1_ostm_init_clkevt(priv);
err:
	return ret;
}

/***********************************************************************/
/*
 * The following source code is left only for the compatibiliy
 * with RZLSP-V2.0.0.
 */
static int __init rza1_ostm_probe(struct platform_device *pdev)
{
	struct rza1_ostm_priv *priv;
	struct rza1_ostm_pdata *pdata;
	int ret = 0;

	if (!is_early_platform_device(pdev)) {
		pm_runtime_set_active(&pdev->dev);
		pm_runtime_enable(&pdev->dev);
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "failed to get platform data\n");
		return -ENXIO;
	}

	priv = platform_get_drvdata(pdev);
	if (priv) {
		dev_info(&pdev->dev, "kept as earlytimer\n");
		goto out;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_info(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	rza1_ostm_priv = priv;

	priv->pdev = pdev;
	platform_set_drvdata(priv->pdev, priv);

	ret = rza1_ostm_timer_init(priv);
	if (ret) {
		kfree(priv);
		platform_set_drvdata(pdev, NULL);
		pm_runtime_idle(&pdev->dev);
		return ret;
	}

	if (is_early_platform_device(pdev))
		return ret;

out:
	if (pdata->clksrc.rating || pdata->clkevt.rating)
		pm_runtime_irq_safe(&pdev->dev);
	else
		pm_runtime_idle(&pdev->dev);

	return ret;
}

static int rza1_ostm_remove(struct platform_device *pdev)
{
	rza1_ostm_priv = NULL;
	system_clock = NULL;

	return -EBUSY;	/* cannot unregister */
}

static struct platform_driver rza1_ostm_timer = {
	.driver	= {
		.name	= "ostm",
	},
	.probe		= rza1_ostm_probe,
	.remove		= rza1_ostm_remove,
};

static int __init rza1_ostm_init(void)
{
	return platform_driver_register(&rza1_ostm_timer);
}

static void __exit rza1_ostm_exit(void)
{
	platform_driver_unregister(&rza1_ostm_timer);
}

early_platform_init("earlytimer", &rza1_ostm_timer);
subsys_initcall(rza1_ostm_init);
module_exit(rza1_ostm_exit);

MODULE_AUTHOR("Magnus Damm");	/* original author */
MODULE_DESCRIPTION("RZ/A1 OSTM Timer Driver");
MODULE_LICENSE("GPL");
