/*
 * SH ADC driver
 *
 * Copyright (C) 2013  Renesas Solutions Corp.
 *
 * Based on at91_adc.c:
 * Copyright 2011 Free Electrons
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/driver.h>
#include <linux/iio/machine.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_data/sh_adc.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>


#define DRIVER_VERSION	"2013-06-12"

#define SH_ADC_ADDRA	0x00
#define SH_ADC_ADDRB	0x02
#define SH_ADC_ADDRC	0x04
#define SH_ADC_ADDRD	0x06
#define SH_ADC_ADDRE	0x08
#define SH_ADC_ADDRF	0x0a
#define SH_ADC_ADDRG	0x0c
#define SH_ADC_ADDRH	0x0e
#define SH_ADC_ADCSR	0x60

#define SH_ADC_ADDR_MASK	0xffc0
#define SH_ADC_ADDR_SHIFT	6

#define SH_ADC_ADCSR_ADF	0x8000
#define SH_ADC_ADCSR_ADIE	0x4000
#define SH_ADC_ADCSR_ADST	0x2000
#define SH_ADC_ADCSR_TRGS_MASK	0x1e00
#define SH_ADC_ADCSR_TRGS_NON	0x0000	/* external trigger input is disable */
#define SH_ADC_ADCSR_TRGS_TRGAN	0x0200	/* MTU2, TRGAN */
#define SH_ADC_ADCSR_TRGS_TRG0N	0x0400	/* MTU2, TRG0N */
#define SH_ADC_ADCSR_TRGS_TRG4AN	0x0600	/* MTU2, TRG4AN */
#define SH_ADC_ADCSR_TRGS_TRG4BN	0x0800	/* MTU2, TRG4BN */
#define SH_ADC_ADCSR_TRGS_EXT	0x1200	/* external pin trigger */
#define SH_ADC_ADCSR_CKS_MASK	0x01c0
#define SH_ADC_ADCSR_CKS_256	0x0000
#define SH_ADC_ADCSR_CKS_298	0x0040
#define SH_ADC_ADCSR_CKS_340	0x0080
#define SH_ADC_ADCSR_CKS_382	0x00c0
#define SH_ADC_ADCSR_CKS_1054	0x0100
#define SH_ADC_ADCSR_CKS_1096	0x0140
#define SH_ADC_ADCSR_CKS_1390	0x0180
#define SH_ADC_ADCSR_CKS_1432	0x01c0
#define SH_ADC_ADCSR_MDS_MASK	0x0038
#define SH_ADC_ADCSR_MDS_SINGLE	0x0000
#define SH_ADC_ADCSR_MDS_M_1_4	0x0020	/* multi mode, channel 1 to 4 */
#define SH_ADC_ADCSR_MDS_M_1_8	0x0028	/* multi mode, channel 1 to 8 */
#define SH_ADC_ADCSR_MDS_S_1_4	0x0030	/* scan mode, channel 1 to 4 */
#define SH_ADC_ADCSR_MDS_S_1_8	0x0038	/* scan mode, channel 1 to 8 */
#define SH_ADC_ADCSR_CH_MASK	0x0003
#define SH_ADC_ADCSR_CH_AN0	0x0000
#define SH_ADC_ADCSR_CH_AN1	0x0001
#define SH_ADC_ADCSR_CH_AN2	0x0002
#define SH_ADC_ADCSR_CH_AN3	0x0003
#define SH_ADC_ADCSR_CH_AN4	0x0004
#define SH_ADC_ADCSR_CH_AN5	0x0005
#define SH_ADC_ADCSR_CH_AN6	0x0006
#define SH_ADC_ADCSR_CH_AN7	0x0007

#define SH_ADC_NUM_CHANNEL	8
#define sh_adc_get_reg_addr(ch)	(SH_ADC_ADDRA + ch * 2)

struct sh_adc {
	/* ADC */
	void __iomem	*reg;
	struct mutex	lock;
	bool		done;
	int		irq;
	struct clk	*clk;
	u16		*buffer;
	wait_queue_head_t wq_data_avail;

	/* MTU2 setting as a ADC trigger */
	void __iomem	*mtu2_share_reg;
	void __iomem	*mtu2_reg;
	struct clk	*mtu2_clk;
	u8		mtu2_ch;
	int		val;
	unsigned long	rate;
	unsigned long	periodic;
};

/* MTU2 */
#define TSTR -1 /* shared register */
#define TCR  0 /* channel register */
#define TMDR 1 /* channel register */
#define TIOR 2 /* channel register */
#define TIER 3 /* channel register */
#define TSR  4 /* channel register */
#define TCNT 5 /* channel register */
#define TGR  6 /* channel register */

#define SH_MTU2_TSR_TGFA 0x01

#define TIMER_BIT 1 /* MTU2 ch.1 */
#define CLK_DIV 64 /* count rate P0/64 (depend on TCR setting.) */
static DEFINE_RAW_SPINLOCK(sh_mtu2_lock);

static unsigned long mtu2_reg_offs[] = {
	[TCR] = 0,
	[TMDR] = 1,
	[TIOR] = 2,
	[TIER] = 4,
	[TSR] = 5,
	[TCNT] = 6,
	[TGR] = 8,
};

static inline unsigned long sh_mtu2_read(struct sh_adc *adc, int reg_nr)
{
	unsigned long offs;

	if (reg_nr == TSTR)
		return ioread8(adc->mtu2_share_reg);

	offs = mtu2_reg_offs[reg_nr];

	if ((reg_nr == TCNT) || (reg_nr == TGR))
		return ioread16(adc->mtu2_reg + offs);
	else
		return ioread8(adc->mtu2_reg + offs);
}

static inline void sh_mtu2_write(struct sh_adc *adc, int reg_nr,
				unsigned long value)
{
	unsigned long offs;

	if (reg_nr == TSTR) {
		iowrite8(value, adc->mtu2_share_reg);
		return;
	}

	offs = mtu2_reg_offs[reg_nr];

	if ((reg_nr == TCNT) || (reg_nr == TGR))
		iowrite16(value, adc->mtu2_reg + offs);
	else
		iowrite8(value, adc->mtu2_reg + offs);
}

static void sh_mtu2_start_stop_ch(struct sh_adc *adc, bool start)
{
	unsigned long flags, value;

	/* start stop register shared by multiple timer channels */
	raw_spin_lock_irqsave(&sh_mtu2_lock, flags);
	value = sh_mtu2_read(adc, TSTR);

	if (start)
		value |= 1 << adc->mtu2_ch;
	else
		value &= ~(1 << adc->mtu2_ch);

	sh_mtu2_write(adc, TSTR, value);
	raw_spin_unlock_irqrestore(&sh_mtu2_lock, flags);
}

static int sh_mtu2_set_trigger(struct sh_adc *adc, bool timer_start)
{
	if (adc->val <= 0)
		return -EINVAL;
	/* make sure channel is disabled */
	sh_mtu2_start_stop_ch(adc, 0);

	adc->rate = clk_get_rate(adc->mtu2_clk) / CLK_DIV;
	adc->periodic = ((adc->rate * adc->val) / 1000);
	if (adc->periodic > 0xffff)
		return -EINVAL;

	/* "Periodic Counter Operation" */
	sh_mtu2_write(adc, TGR, adc->periodic);
	sh_mtu2_write(adc, TCNT, 0);

	/* restart timer if timer_start is true */
	sh_mtu2_start_stop_ch(adc, timer_start);

	return 0;
}

static int sh_mtu2_enable(struct sh_adc *adc)
{
	/* make sure channel is disabled */
	sh_mtu2_start_stop_ch(adc, false);

	/* "Periodic Counter Operation" */
	sh_mtu2_write(adc, TCR, 0x23); /* TGRA clear, divide clock by 64 */
	sh_mtu2_write(adc, TIOR, 0);
	sh_mtu2_write(adc, TMDR, 0); /* normal operation(bit3-0: 0) */
	sh_mtu2_write(adc, TIER, 0x81); /* enable ADC trigger & TGFA bit INT */

	/* a default sampling rate is 10Hz */
	adc->val = 100;
	return sh_mtu2_set_trigger(adc, false);

}

/* ADC */
struct sh_adc_avail_chs {
	int	channel_mask;
	u8	an;
};
static struct sh_adc_avail_chs sh_adc_chs[][SH_ADC_NUM_CHANNEL] = {
	{
		{ 0x01, 0},
		{ 0x03, 1},
		{ 0x07, 2},
		{ 0x0f, 3},
		{ 0x10, 4},
		{ 0x30, 5},
		{ 0x70, 6},
		{ 0xf0, 7},
	},
	{
		{ 0x01, 0},
		{ 0x03, 1},
		{ 0x07, 2},
		{ 0x0f, 3},
		{ 0x1f, 4},
		{ 0x3f, 5},
		{ 0x7f, 6},
		{ 0xff, 7},
	},
};

static void sh_adc_write(struct sh_adc *adc, unsigned short data,
			 unsigned long offset)
{
	iowrite16(data, adc->reg + offset);
}

static unsigned short sh_adc_read(struct sh_adc *adc, unsigned long offset)
{
	return ioread16(adc->reg + offset);
}

static void sh_adc_set_bit(struct sh_adc *adc, unsigned short val,
			   unsigned long offset)
{
	unsigned short tmp;

	tmp = sh_adc_read(adc, offset);
	tmp |= val;
	sh_adc_write(adc, tmp, offset);
}

static void sh_adc_clear_bit(struct sh_adc *adc, unsigned short val,
			     unsigned long offset)
{
	unsigned short tmp;

	tmp = sh_adc_read(adc, offset);
	tmp &= ~val;
	sh_adc_write(adc, tmp, offset);
}

static int sh_adc_start_adc(struct sh_adc *adc, int channel)
{
	sh_adc_clear_bit(adc, SH_ADC_ADCSR_ADST, SH_ADC_ADCSR);
	sh_adc_clear_bit(adc, SH_ADC_ADCSR_TRGS_MASK, SH_ADC_ADCSR);

	if (channel >= 0) {
		sh_adc_clear_bit(adc, SH_ADC_ADCSR_MDS_MASK, SH_ADC_ADCSR);
		sh_adc_clear_bit(adc, SH_ADC_ADCSR_CH_MASK, SH_ADC_ADCSR);
		sh_adc_set_bit(adc, (SH_ADC_ADCSR_ADST | channel),
				SH_ADC_ADCSR);
	} else {
		sh_adc_set_bit(adc, SH_ADC_ADCSR_TRGS_TRGAN, SH_ADC_ADCSR);
		sh_mtu2_start_stop_ch(adc, true);
	}
	return 0;
}

static int sh_adc_stop_adc(struct sh_adc *adc)
{
	sh_mtu2_start_stop_ch(adc, false);
	return 0;
}

static int sh_adc_get_raw_value(struct iio_dev *idev, int *val, int channel)
{
	struct sh_adc *adc = iio_priv(idev);
	int ret = 1;

	mutex_lock(&adc->lock);

	if (!iio_buffer_enabled(idev)) {
		sh_adc_start_adc(adc, channel);
		ret = wait_event_interruptible_timeout(adc->wq_data_avail,
				adc->done, msecs_to_jiffies(1000));
		adc->done = false;
	}
	if (ret > 0)
		*val = sh_adc_read(adc, sh_adc_get_reg_addr(channel));
	mutex_unlock(&adc->lock);
	return ret;
}

static int sh_adc_read_raw(struct iio_dev *idev,
		struct iio_chan_spec const *chan,
		int *val, int *val2, long mask)
{
	int ret;
	struct sh_adc *adc = iio_priv(idev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = adc->val;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_RAW:
		ret = sh_adc_get_raw_value(idev, val, chan->channel);
		*val >>= chan->scan_type.shift;
		*val &= ((1 << chan->scan_type.realbits) - 1);
		if (ret > 0)
			ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int sh_adc_write_raw(struct iio_dev *idev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	int ret = 0;
	struct sh_adc *adc = iio_priv(idev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		adc->val = val;
		ret = sh_mtu2_set_trigger(adc, iio_buffer_enabled(idev));
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int sh_adc_get_ch(struct iio_dev *idev, int i)
{
	int j;
	for (j = 0; j < idev->num_channels; j++)
		if (sh_adc_chs[i][j].channel_mask == *idev->active_scan_mask)
			return sh_adc_chs[i][j].an;
	return -1;
}

static int sh_adc_get_mds_ch(struct iio_dev *idev)
{
	int i, mds_ch, ch, cnt = 0;

	ch = 0;
	for (i = 0; i < idev->masklength; i++)
		if (test_bit(i, idev->active_scan_mask)) {
			ch = i;
			cnt++;
		}

	mds_ch = -1;
	if (cnt == 1)
		mds_ch = (ch | SH_ADC_ADCSR_MDS_SINGLE);
	else if (cnt >= 2 && cnt <= idev->num_channels) {
		mds_ch = sh_adc_get_ch(idev, (cnt > 4));
		if (mds_ch < 0)
			return -1;
		mds_ch |= (cnt > 4 ? SH_ADC_ADCSR_MDS_M_1_8 :
				   SH_ADC_ADCSR_MDS_M_1_4);
	}
	return mds_ch;
}

static int sh_adc_configure_trigger(struct iio_trigger *trig, bool state)
{
	struct iio_dev *idev = (struct iio_dev *)dev_get_drvdata(&trig->dev);
	struct sh_adc *adc = iio_priv(idev);
	int mds_ch;

	if (state) {
		/* set trigger */
		adc->buffer = kzalloc(idev->scan_bytes, GFP_KERNEL);
		if (!adc->buffer)
			return -ENOMEM;

		mds_ch = sh_adc_get_mds_ch(idev);
		if (mds_ch < 0)
			return -1;
		sh_adc_set_bit(adc, mds_ch, SH_ADC_ADCSR);

		sh_adc_start_adc(adc, -1);
	} else {
		/* clear trigger */
		sh_adc_stop_adc(adc);
		kfree(adc->buffer);
	}
	return 0;
}

static const struct iio_trigger_ops sh_adc_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &sh_adc_configure_trigger,
};

static int sh_adc_trigger_init(struct iio_dev *idev)
{
	int ret;
	struct iio_trigger *trig;

	trig = iio_trigger_alloc("%s-dev%i", idev->name, idev->id);
	if (!trig)
		return -ENOMEM;
	trig->dev.parent = idev->dev.parent;
	dev_set_drvdata(&trig->dev, idev);
	trig->ops = &sh_adc_trigger_ops;

	ret = iio_trigger_register(trig);
	if (ret)
		iio_trigger_free(trig);
	else
		idev->trig = trig;

	return ret;
}

static irqreturn_t sh_adc_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *idev = pf->indio_dev;
	struct sh_adc *adc = iio_priv(idev);
	struct iio_chan_spec const *chans;
	int val, ret, dnum, i;

	chans = idev->channels;
	i = 0;
	for (dnum = 0; dnum < idev->masklength; dnum++) {
		/* skip no-mask channel. */
		if (!test_bit(dnum, idev->active_scan_mask))
			continue;

		/* read ch data */
		ret = sh_adc_get_raw_value(idev, &val, dnum);
		val >>= chans[dnum].scan_type.shift;
		val &= ((1 << chans[dnum].scan_type.realbits) - 1);
		if (ret > 0)
			adc->buffer[i] = (u16)val;
		i++;
	}

	if (idev->scan_timestamp) {
		s64 *timestamp = (s64 *)((u8 *)adc->buffer +
				ALIGN(i, sizeof(s64)));
		*timestamp = pf->timestamp;
	}
	iio_push_to_buffers(idev, (u8 *)adc->buffer);
	iio_trigger_notify_done(idev->trig);

	enable_irq(adc->irq);

	return IRQ_HANDLED;
}


static const struct iio_info sh_adc_info = {
	.write_raw = sh_adc_write_raw,
	.read_raw = sh_adc_read_raw,
	.driver_module = THIS_MODULE,
};

static inline void sh_adc_iio_map_unregister(struct platform_device *pdev,
					     struct sh_adc *adc, int map_stat)
{
	switch (map_stat) {
	case 3:
		devm_iounmap(&pdev->dev, adc->mtu2_reg);
	case 2:
		devm_iounmap(&pdev->dev, adc->mtu2_share_reg);
	case 1:
		devm_iounmap(&pdev->dev, adc->reg);
	}
}

static irqreturn_t sh_adc_irq(int irq, void *private)
{
	struct iio_dev *idev = private;
	struct sh_adc *adc = iio_priv(idev);
	u8 value;

	value = sh_mtu2_read(adc, TSR);
	if (value & SH_MTU2_TSR_TGFA) {
		value &= ~SH_MTU2_TSR_TGFA;
		sh_mtu2_write(adc, TSR, value);
		return IRQ_HANDLED;
	}

	sh_adc_clear_bit(adc, SH_ADC_ADCSR_ADF, SH_ADC_ADCSR);

	if (!iio_buffer_enabled(idev)) {
		adc->done = true;
		wake_up_interruptible(&adc->wq_data_avail);
	} else {
		adc->irq = irq;
		disable_irq_nosync(irq);
		iio_trigger_poll(idev->trig, iio_get_time_ns());
	}

	return IRQ_HANDLED;
}

static int sh_adc_get_clk(struct platform_device *pdev, struct clk **clk,
			  char *name)
{
	int ret;

	*clk = devm_clk_get(&pdev->dev, name);
	if (IS_ERR(*clk)) {
		dev_err(&pdev->dev, "Failed to get the clock.\n");
		return -1;
	}
	ret = clk_prepare_enable(*clk);
	if (ret) {
		dev_err(&pdev->dev,
			"Could not prepare or enable the clock.\n");
		return -1;
	}
	/* enable clock */
	ret = clk_enable(*clk);
	if (ret) {
		dev_err(&pdev->dev, "cannot enable clock\n");
		return -1;
	}
	return 0;
}

static int sh_adc_iio_free_irqs(struct platform_device *pdev, int nres)
{
	struct iio_dev *idev = platform_get_drvdata(pdev);
	struct resource *res;
	int irq;

	for (nres--; nres >= 0; nres--) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, nres);
		if (!res)
			continue;
		for (irq = res->start; irq <= res->end; irq++)
			free_irq(irq, idev);
	}
	return 0;
}

static int sh_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *idev;
	struct sh_adc *adc;
	struct resource *res;
	struct iio_chan_spec *chan_array, *timestamp;
	struct sh_adc_data *pdata = pdev->dev.platform_data;
	int i, nmap, nres, irq = 0;
	int ret = 0;

	idev = iio_device_alloc(sizeof(*adc));
	if (!idev)
		return -ENOMEM;

	adc = iio_priv(idev);
	platform_set_drvdata(pdev, idev);

	/* ADC base address */
	nmap = 0;
	res = platform_get_resource(pdev, IORESOURCE_MEM, nmap);
	adc->reg = devm_request_and_ioremap(&pdev->dev, res);
	if (!adc->reg)
		goto error_map;
	nmap++;
	/* MTU2 base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, nmap);
	adc->mtu2_share_reg = devm_request_and_ioremap(&pdev->dev, res);
	if (!adc->mtu2_share_reg)
		goto error_map;
	nmap++;
	res = platform_get_resource(pdev, IORESOURCE_MEM, nmap);
	adc->mtu2_reg = devm_request_and_ioremap(&pdev->dev, res);
	if (!adc->mtu2_reg)
		goto error_map;
	nmap++;

	init_waitqueue_head(&adc->wq_data_avail);
	mutex_init(&adc->lock);

	idev->dev.parent = &pdev->dev;
	idev->name = dev_name(&pdev->dev);
	idev->modes = INDIO_DIRECT_MODE;
	idev->info = &sh_adc_info;

	/* channel setup */
	idev->num_channels = pdata->num_channels;
	chan_array = devm_kzalloc(&idev->dev,
				  ((idev->num_channels + 1) *
					sizeof(struct iio_chan_spec)),
				  GFP_KERNEL);
	if (!chan_array) {
		ret = -ENOMEM;
		goto error_map;
	}
	for (i = 0; i < pdata->num_channels; i++) {
		struct iio_chan_spec *chan = chan_array + i;
		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = i;
		chan->scan_index = i;
		chan->scan_type.sign = 'u';
		chan->scan_type.realbits = 12;
		chan->scan_type.storagebits = 16;
		chan->scan_type.shift = 4;
		chan->info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
				  BIT(IIO_CHAN_INFO_SAMP_FREQ);
		chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	}
	timestamp = chan_array + i;
	timestamp->type = IIO_TIMESTAMP;
	timestamp->channel = -1;
	timestamp->scan_index = i;
	timestamp->scan_type.sign = 's';
	timestamp->scan_type.realbits = 64;
	timestamp->scan_type.storagebits = 64;
	idev->num_channels++;

	idev->channels = chan_array;

	/* request interrupt */
	for (nres = 0; nres < pdev->num_resources; nres++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, nres);
		if (!res)
			continue;
		for (irq = res->start; irq <= res->end; irq++) {
			ret = request_irq(irq, sh_adc_irq, IRQF_DISABLED,
					  idev->name, idev);
			if (ret < 0) {
				for (irq--; irq >= res->start; irq--)
					free_irq(irq, idev);
				goto error_free_irq;
			}
		}
	}
	sh_adc_set_bit(adc, SH_ADC_ADCSR_ADIE, SH_ADC_ADCSR);

	/* get clock */
	ret = sh_adc_get_clk(pdev, &adc->clk, "adc0");
	if (ret < 0)
		goto error_free_irq;
	ret = sh_adc_get_clk(pdev, &adc->mtu2_clk, "mtu2_fck");
	if (ret < 0)
		goto error_mtu2_clk;

	/* set buffer and regist the trigger */
	iio_triggered_buffer_setup(idev, &iio_pollfunc_store_time,
				 &sh_adc_trigger_handler, NULL);
	sh_adc_trigger_init(idev);

	/* register iio device */
	ret = iio_device_register(idev);
	if (ret) {
		dev_err(&idev->dev, "Error: iio dev can't register(%d)\n", ret);
		goto error_iio_dev_reg;
	}

	/* set convert rate */
	sh_adc_set_bit(adc, SH_ADC_ADCSR_CKS_1054, SH_ADC_ADCSR);

	dev_info(&pdev->dev, "version %s\n", DRIVER_VERSION);

	/* initialize mtu2 as a ADC trigger */
	adc->mtu2_ch = pdata->mtu2_ch;
	sh_mtu2_enable(adc);

	return 0;

error_iio_dev_reg:
	clk_disable(adc->mtu2_clk);
	devm_clk_put(&pdev->dev, adc->mtu2_clk);
error_mtu2_clk:
	clk_disable(adc->clk);
	devm_clk_put(&pdev->dev, adc->clk);
error_free_irq:
	sh_adc_iio_free_irqs(pdev, nres);
error_map:
	sh_adc_iio_map_unregister(pdev, adc, nmap);
	iio_device_free(idev);
	return ret;
}

static int sh_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *idev = platform_get_drvdata(pdev);
	struct sh_adc *adc = iio_priv(idev);

	iio_device_unregister(idev);

	clk_disable(adc->mtu2_clk);
	devm_clk_put(&pdev->dev, adc->mtu2_clk);
	clk_disable(adc->clk);
	devm_clk_put(&pdev->dev, adc->clk);

	sh_adc_iio_free_irqs(pdev, pdev->num_resources);
	sh_adc_iio_map_unregister(pdev, adc, 3);
	iio_device_free(idev);

	return 0;
}

static struct platform_driver sh_adc_driver = {
	.probe = sh_adc_probe,
	.remove = sh_adc_remove,
	.driver = {
		.name = "sh_adc",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(sh_adc_driver);

MODULE_DESCRIPTION("SH ADC Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sh-adc");
