/*
 * RIIC bus driver
 *
 * Copyright (C) 2013  Renesas Solutions Corp.
 *
 * Based on i2c-sh_mobile.c
 * Copyright (C) 2008 Magnus Damm
 *
 * Portions of the code based on out-of-tree driver i2c-sh7343.c
 * Copyright (c) 2006 Carlos Munoz <carlos@kenati.com>
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/riic_rza1.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clk.h>
#ifdef CONFIG_ARCH_R7S
#  include <mach/rza1.h>
#endif

#define DRIVER_VERSION	"2013-04-19"

#ifdef CONFIG_ARCH_R7S72100

#define RIIC_ICCR1	0x00
#define RIIC_ICCR2	0x04
#define RIIC_ICMR1	0x08
#define RIIC_ICMR2	0x0c
#define RIIC_ICMR3	0x10
#define RIIC_ICFER	0x14
#define RIIC_ICSER	0x18
#define RIIC_ICIER	0x1c
#define RIIC_ICSR1	0x20
#define RIIC_ICSR2	0x24
#define RIIC_ICBRL	0x34
#define RIIC_ICBRH	0x38
#define RIIC_ICDRT	0x3c
#define RIIC_ICDRR	0x40

/* ICCR1 */
#define ICCR1_ICE	0x80
#define ICCR1_IICRST	0x40
#define ICCR1_CLO	0x20
#define ICCR1_SOWP	0x10
#define ICCR1_SCLO	0x08
#define ICCR1_SDAO	0x04
#define ICCR1_SCLI	0x02
#define ICCR1_SDAI	0x01

/* ICCR2 */
#define ICCR2_BBSY	0x80
#define ICCR2_MST	0x40
#define ICCR2_TRS	0x20
#define ICCR2_SP	0x08
#define ICCR2_RS	0x04
#define ICCR2_ST	0x02

/* ICMR1 */
#define ICMR1_MTWP	0x80
#define ICMR1_CKS_MASK	0x70
#define ICMR1_BCWP	0x08
#define ICMR1_BC_MASK	0x07

#define ICMR1_CKS(_x)	(((_x) << 4) & ICMR1_CKS_MASK)
#define ICMR1_BC(_x)	((_x) & ICMR1_BC_MASK)

/* ICMR2 */
#define ICMR2_DLCS	0x80
#define ICMR2_SDDL_MASK	0x70
#define ICMR2_TMOH	0x04
#define ICMR2_TMOL	0x02
#define ICMR2_TMOS	0x01

/* ICMR3 */
#define ICMR3_SMBS	0x80
#define ICMR3_WAIT	0x40
#define ICMR3_RDRFS	0x20
#define ICMR3_ACKWP	0x10
#define ICMR3_ACKBT	0x08
#define ICMR3_ACKBR	0x04
#define ICMR3_NF_MASK	0x03

/* ICFER */
#define ICFER_FMPE	0x80
#define ICFER_SCLE	0x40
#define ICFER_NFE	0x20
#define ICFER_NACKE	0x10
#define ICFER_SALE	0x08
#define ICFER_NALE	0x04
#define ICFER_MALE	0x02
#define ICFER_TMOE	0x01

/* ICSER */
#define ICSER_HOAE	0x80
#define ICSER_DIDE	0x20
#define ICSER_GCAE	0x08
#define ICSER_SAR2E	0x04
#define ICSER_SAR1E	0x02
#define ICSER_SAR0E	0x01

/* ICIER */
#define ICIER_TIE	0x80
#define ICIER_TEIE	0x40
#define ICIER_RIE	0x20
#define ICIER_NAKIE	0x10
#define ICIER_SPIE	0x08
#define ICIER_STIE	0x04
#define ICIER_ALIE	0x02
#define ICIER_TMOIE	0x01

/* ICSR1 */
#define ICSR1_HOA	0x80
#define ICSR1_DID	0x20
#define ICSR1_GCA	0x08
#define ICSR1_AAS2	0x04
#define ICSR1_AAS1	0x02
#define ICSR1_AAS0	0x01

/* ICSR2 */
#define ICSR2_TDRE	0x80
#define ICSR2_TEND	0x40
#define ICSR2_RDRF	0x20
#define ICSR2_NACKF	0x10
#define ICSR2_STOP	0x08
#define ICSR2_START	0x04
#define ICSR2_AL	0x02
#define ICSR2_TMOF	0x01

/* SARLn */
#define SARL_SVA_MASK	0xfe	/* SVA[7:1] */
#define SARL_SVA	0x01

/* SARUn */
#define SARU_SVA_MASK	0x06	/* SVA[9:8] */
#define SARU_FS		0x01

/* ICBRH */
#define ICBRH_RESERVED	0xe0	/* The write value shoud always be 1 */
#define ICBRH_BRH_MASK	0x1f
#define ICBRH_SP100K	16	/* PCLK 33MHz */
#define ICBRH_SP400K	9
#define ICBRH_SP1000K	14

/* ICBRL */
#define ICBRL_RESERVED	0xe0	/* The write value shoud always be 1 */
#define ICBRL_BRL_MASK	0x1f
#define ICBRL_SP100K	19	/* PCLK 33MHz */
#define ICBRL_SP400K	21
#define ICBRL_SP1000K	14

#define RIIC_CORE_PARAM_NUM_SLAVE_BUFFERS	0

#define RIIC_NUM_CHANNELS	4

static const char * const riic_name[RIIC_NUM_CHANNELS] = {
	"riic.0", "riic.1", "riic.2", "riic.3",
};

enum {
	RIIC_TXI,
	RIIC_TEI,
	RIIC_RXI,
	RIIC_SPI,
	RIIC_STI,
	RIIC_NAK,
	RIIC_ALI,
	RIIC_TMOI,
};

#else /* default RIIC */

#define RIIC_ICCR1	0x00
#define RIIC_ICCR2	0x01
#define RIIC_ICMR1	0x02
#define RIIC_ICMR2	0x03
#define RIIC_ICMR3	0x04
#define RIIC_ICFER	0x05
#define RIIC_ICSER	0x06
#define RIIC_ICIER	0x07
#define RIIC_ICSR1	0x08
#define RIIC_ICSR2	0x09
#define RIIC_SARL0	0x0a
#define RIIC_SARU0	0x0b
#define RIIC_SARL1	0x0c
#define RIIC_SARU1	0x0d
#define RIIC_SARL2	0x0e
#define RIIC_SARU2	0x0f
#define RIIC_ICBRL	0x10
#define RIIC_ICBRH	0x11
#define RIIC_ICDRT	0x12
#define RIIC_ICDRR	0x13

/* ICCR1 */
#define ICCR1_ICE	0x80
#define ICCR1_IICRST	0x40
#define ICCR1_CLO	0x20
#define ICCR1_SOWP	0x10
#define ICCR1_SCLO	0x08
#define ICCR1_SDAO	0x04
#define ICCR1_SCLI	0x02
#define ICCR1_SDAI	0x01

/* ICCR2 */
#define ICCR2_BBSY	0x80
#define ICCR2_MST	0x40
#define ICCR2_TRS	0x20
#define ICCR2_SP	0x08
#define ICCR2_RS	0x04
#define ICCR2_ST	0x02

/* ICMR1 */
#define ICMR1_MTWP	0x80
#define ICMR1_CKS_MASK	0x70
#define ICMR1_BCWP	0x08
#define ICMR1_BC_MASK	0x07

#define ICMR1_CKS(_x)	((_x << 4) & ICMR1_CKS_MASK)
#define ICMR1_BC(_x)	((_x) & ICMR1_BC_MASK)

/* ICMR2 */
#define ICMR2_DLCS	0x80
#define ICMR2_SDDL_MASK	0x70
#define ICMR2_TMOH	0x04
#define ICMR2_TMOL	0x02
#define ICMR2_TMOS	0x01

/* ICMR3 */
#define ICMR3_SMBS	0x80
#define ICMR3_WAIT	0x40
#define ICMR3_RDRFS	0x20
#define ICMR3_ACKWP	0x10
#define ICMR3_ACKBT	0x08
#define ICMR3_ACKBR	0x04
#define ICMR3_NF_MASK	0x03

/* ICFER */
#define ICFER_FMPE	0x80
#define ICFER_SCLE	0x40
#define ICFER_NFE	0x20
#define ICFER_NACKE	0x10
#define ICFER_SALE	0x08
#define ICFER_NALE	0x04
#define ICFER_MALE	0x02
#define ICFER_TMOE	0x01

/* ICSER */
#define ICSER_HOAE	0x80
#define ICSER_DIDE	0x20
#define ICSER_GCAE	0x08
#define ICSER_SAR2E	0x04
#define ICSER_SAR1E	0x02
#define ICSER_SAR0E	0x01

/* ICIER */
#define ICIER_TIE	0x80
#define ICIER_TEIE	0x40
#define ICIER_RIE	0x20
#define ICIER_NAKIE	0x10
#define ICIER_SPIE	0x08
#define ICIER_STIE	0x04
#define ICIER_ALIE	0x02
#define ICIER_TMOIE	0x01

/* ICSR1 */
#define ICSR1_HOA	0x80
#define ICSR1_DID	0x20
#define ICSR1_GCA	0x08
#define ICSR1_AAS2	0x04
#define ICSR1_AAS1	0x02
#define ICSR1_AAS0	0x01

/* ICSR2 */
#define ICSR2_TDRE	0x80
#define ICSR2_TEND	0x40
#define ICSR2_RDRF	0x20
#define ICSR2_NACKF	0x10
#define ICSR2_STOP	0x08
#define ICSR2_START	0x04
#define ICSR2_AL	0x02
#define ICSR2_TMOF	0x01

/* SARLn */
#define SARL_SVA_MASK	0xfe	/* SVA[7:1] */
#define SARL_SVA	0x01

/* SARUn */
#define SARU_SVA_MASK	0x06	/* SVA[9:8] */
#define SARU_FS		0x01

/* ICBRH */
#define ICBRH_RESERVED	0xe0	/* The write value shoud always be 1 */
#define ICBRH_BRH_MASK	0x1f

/* ICBRL */
#define ICBRL_RESERVED	0xe0	/* The write value shoud always be 1 */
#define ICBRL_BRL_MASK	0x1f

#define RIIC_CORE_PARAM_NUM_SLAVE_BUFFERS	16

#define RIIC_NUM_CHANNELS	4

static const char * const riic_name[RIIC_NUM_CHANNELS] = {
	"riic.0", "riic.1", "riic.2", "riic.3",
};

enum {
	RIIC_TXI,
	RIIC_TEI,
	RIIC_RXI,
	RIIC_SPI,
	RIIC_STI,
	RIIC_NAK,
	RIIC_ALI,
	RIIC_TMOI,
};
#endif /* default RIIC */

#define RIIC_TIMEOUT	10000	/* 100msec (unit = 10usec) */

/* Error codes */
enum {
	RIIC_CORE_NO_ERROR = 0,
	RIIC_CORE_INVALID,
	RIIC_CORE_BUSY,
	RIIC_CORE_TIMEOUT,
	RIIC_CORE_AL_ADDR,
	RIIC_CORE_AL_DATA,
	RIIC_CORE_NACK_ADDR,
	RIIC_CORE_NACK_DATA,
};

enum riic_state {
	RIIC_STATE_IDLE,
	RIIC_STATE_MASTER,
	RIIC_STATE_SLAVE,
	RIIC_STATE_BUS_RECOVERY
};

enum riic_master_state {
	RIIC_MASTER_PREPARE,
	RIIC_MASTER_ISSUED_START,
	RIIC_MASTER_SENT_SLAVE_ADDRESS_NEXT,
	RIIC_MASTER_SENT_SLAVE_ADDRESS,
	RIIC_MASTER_DMAC_FINISHED,
	RIIC_MASTER_TRANSMITTING,
	RIIC_MASTER_TRANSMIT_LAST,
	RIIC_MASTER_RECEIVING,
	RIIC_MASTER_RECEIVE_LAST,
	RIIC_MASTER_ISSUED_STOP,
	RIIC_MASTER_END,
};

enum riic_slave_state {
	RIIC_SLAVE_ISSUED_START,
	RIIC_SLAVE_START_XFER,
	RIIC_SLAVE_DMAC_FINISHED,
	RIIC_SLAVE_ISSUED_STOP,
	RIIC_SLAVE_ISSUED_RESTART,
	RIIC_SLAVE_TIMEOUT,
};

#define NUM_IRQ 8

struct riic_data {
	struct device *dev;
	struct clk *clk;
	void __iomem *reg;
	struct i2c_adapter adap;
	struct i2c_msg *msg;
	int		channel;
	const char	*name;
	enum riic_state	state;
	int irq[NUM_IRQ];
	int irqn;
	spinlock_t	lock;

	/* for master mode */
	enum riic_master_state	master_state;
	struct riic_core_packet	*pkt;
	int			num_pkt;
	int			pkt_index;
	wait_queue_head_t	wait;
	int			sending_addr;	/* for _AL or _NACK */

	/* for slave mode */
#if 0 /* NO SLAVE SUPPORT */
	enum riic_slave_state	slave_state;
	struct riic_core_packet	slv_rx_pkt[RIIC_CORE_PARAM_NUM_SLAVE_BUFFERS];
	int			slv_rx_head;
	int			slv_rx_tail;
	int			slv_timeout;
	wait_queue_head_t	slv_wait;
	struct timer_list	slv_timer;

	/* status */
	unsigned	slave_enabled:1;
#endif

	/* status for master mode */
	unsigned	completed:1;
	unsigned	aled:1;
	unsigned	nacked:1;

	/* for debug */
	int		debug;
	enum riic_state	old_state;
	unsigned char	old_icsr2;
};

static unsigned char riic_read(struct riic_data *rd, unsigned long addr)
{
	return ioread8(rd->reg + addr);
}

static void riic_write(struct riic_data *rd, unsigned char data,
		       unsigned long addr)
{
	iowrite8(data, rd->reg + addr);
}

static void riic_set_bit(struct riic_data *rd, unsigned char val,
			 unsigned long offset)
{
	unsigned char tmp;

	tmp = riic_read(rd, offset) | val;
	riic_write(rd, tmp, offset);
}

static void riic_clear_bit(struct riic_data *rd, unsigned char val,
			   unsigned long offset)
{
	unsigned char tmp;

	tmp = riic_read(rd, offset) & ~val;
	riic_write(rd, tmp, offset);
}

static int riic_set_clock(struct riic_data *rd, int clock)
{
	switch (clock) {
	case 100:
		riic_clear_bit(rd, ICFER_FMPE, RIIC_ICFER);
		riic_clear_bit(rd, ICMR1_CKS_MASK, RIIC_ICMR1);
		riic_set_bit(rd, ICMR1_CKS(3), RIIC_ICMR1);
		riic_write(rd, ICBRH_RESERVED | ICBRH_SP100K, RIIC_ICBRH);
		riic_write(rd, ICBRL_RESERVED | ICBRL_SP100K, RIIC_ICBRL);
		break;
	case 400:
		riic_clear_bit(rd, ICFER_FMPE, RIIC_ICFER);
		riic_clear_bit(rd, ICMR1_CKS_MASK, RIIC_ICMR1);
		riic_set_bit(rd, ICMR1_CKS(1), RIIC_ICMR1);
		riic_write(rd, ICBRH_RESERVED | ICBRH_SP400K, RIIC_ICBRH);
		riic_write(rd, ICBRL_RESERVED | ICBRL_SP400K, RIIC_ICBRL);
		break;
	case 1000:
		riic_set_bit(rd, ICFER_FMPE, RIIC_ICFER);
		riic_clear_bit(rd, ICMR1_CKS_MASK, RIIC_ICMR1);
		riic_set_bit(rd, ICMR1_CKS(0), RIIC_ICMR1);
		riic_write(rd, ICBRH_RESERVED | ICBRH_SP1000K, RIIC_ICBRH);
		riic_write(rd, ICBRL_RESERVED | ICBRL_SP1000K, RIIC_ICBRL);
		break;

	default:
		dev_err(rd->dev, "unsupported clock (%dkHz)\n", clock);
		return -EINVAL;
	}

	return 0;
}

static int riic_init_setting(struct riic_data *rd, int clock)
{
	int ret;

	riic_clear_bit(rd, ICCR1_ICE, RIIC_ICCR1);
	riic_set_bit(rd, ICCR1_IICRST, RIIC_ICCR1);
	riic_clear_bit(rd, ICCR1_IICRST, RIIC_ICCR1);

#ifndef CONFIG_ARCH_R7S72100
	riic_write(rd, 0, RIIC_SARL0);
	riic_write(rd, 0, RIIC_SARU0);
#endif
	riic_write(rd, ICSER_SAR0E, RIIC_ICSER);

	riic_write(rd, ICMR1_BC(7), RIIC_ICMR1);
	ret = riic_set_clock(rd, clock);
	if (ret < 0)
		return ret;

	riic_set_bit(rd, ICCR1_ICE, RIIC_ICCR1);	/* Enable RIIC */
	riic_set_bit(rd, ICMR3_RDRFS | ICMR3_WAIT | ICMR3_ACKWP, RIIC_ICMR3);

	return 0;
}

static int riic_check_busy(struct riic_data *rd)
{
	if (riic_read(rd, RIIC_ICCR2) & ICCR2_BBSY) {
		dev_err(rd->dev, "i2c bus is busy.\n");
		return -EBUSY;
	}

	return 0;
}

static int riic_master_issue_stop(struct riic_data *rd)
{
	riic_clear_bit(rd, ICSR2_STOP | ICSR2_NACKF, RIIC_ICSR2);
	if (riic_read(rd, RIIC_ICCR2) & ICCR2_MST) {
		riic_set_bit(rd, ICIER_SPIE, RIIC_ICIER);
		riic_set_bit(rd, ICCR2_SP, RIIC_ICCR2);
		return 0;
	}

	return -EFAULT;
}

static int riic_send_slave_address(struct riic_data *rd, int read)
{
	unsigned char sa_rw[2];

	if (rd->pkt->rw == RIIC_CORE_RW_MASTER_TRANSMIT) {
		riic_set_bit(rd, ICIER_TEIE, RIIC_ICIER);
		sa_rw[0] = (rd->pkt->slave_address << 1);
	} else {
		sa_rw[0] = ((rd->pkt->slave_address << 1) | 1);
	}
	riic_write(rd, sa_rw[0], RIIC_ICDRT);

	return 0;
}

static void riic_master_try_to_free_sda(struct riic_data *rd)
{
	unsigned long flags;
	int i, j;

	if (!(riic_read(rd, RIIC_ICCR2) & ICCR2_MST))
		return;
	/* If SDA is already high, RIIC will not output extra SCL */
	if (riic_read(rd, RIIC_ICCR1) & ICCR1_SDAI)
		return;
	/* If SCL is low held by other device, RIIC cannot output extra SCL */
	if (!(riic_read(rd, RIIC_ICCR1) & ICCR1_SCLI))
		return;

	spin_lock_irqsave(&rd->lock, flags);
	riic_clear_bit(rd, ICFER_MALE, RIIC_ICFER);
	for (i = 0; i < 32; i++) {
		if (riic_read(rd, RIIC_ICCR1) & ICCR1_SDAI)
			break;

		riic_set_bit(rd, ICCR1_CLO, RIIC_ICCR1);
		j = 3000;
		while (riic_read(rd, RIIC_ICCR1) & ICCR1_CLO) {
			udelay(1);
			if (j-- < 0) {
				dev_err(rd->dev, "CLO timeout\n");
				break;
			}
		}
	}
	riic_set_bit(rd, ICFER_MALE, RIIC_ICFER);
	spin_unlock_irqrestore(&rd->lock, flags);
}

static void riic_master_clean_icier(struct riic_data *rd)
{
	/* disable for master mode */
	riic_clear_bit(rd, ICIER_NAKIE | ICIER_ALIE | ICIER_TEIE | ICIER_SPIE,
		       RIIC_ICIER);

	/* enable for slave mode */
	riic_set_bit(rd, ICIER_STIE, RIIC_ICIER);
}

static void riic_master_clean_up(struct riic_data *rd, int timeout)
{
	unsigned long flags;
	int ret;
	int retrycnt = 0;

retry:
	spin_lock_irqsave(&rd->lock, flags);
	if (rd->state != RIIC_STATE_MASTER) {
		spin_unlock_irqrestore(&rd->lock, flags);
		return;
	}
	if (!(riic_read(rd, RIIC_ICCR2) & ICCR2_MST)) {
		spin_unlock_irqrestore(&rd->lock, flags);
		goto out;
	}

	rd->master_state = RIIC_MASTER_END;
	/* dummy read if needed */
	if (riic_read(rd, RIIC_ICSR2) & ICSR2_RDRF)
		riic_read(rd, RIIC_ICDRR);
	riic_master_try_to_free_sda(rd);
	ret = riic_master_issue_stop(rd);
	rd->completed = 0;
	spin_unlock_irqrestore(&rd->lock, flags);

	if (!ret)
		ret = wait_event_interruptible_timeout(rd->wait, rd->completed,
					msecs_to_jiffies(timeout));

	/* if a signal happens, the driver sleeps a little to wait STOP */
	if (ret == -ERESTARTSYS)
		usleep_range(10000, 11000);

	if (riic_read(rd, RIIC_ICCR2) & ICCR2_MST) {
		/* riic_dump(rd, "re-issue stop"); */
		if (retrycnt++ < rd->adap.retries)
			goto retry;
		dev_err(rd->dev, "Can't cleanup a I2C bus (iccr2:%x)\n",
			riic_read(rd, RIIC_ICCR2));
	}

out:
	spin_lock_irqsave(&rd->lock, flags);
	rd->state = RIIC_STATE_IDLE;
	riic_master_clean_icier(rd);
	rd->pkt = NULL;
	spin_unlock_irqrestore(&rd->lock, flags);
}

static int riic_master(struct riic_data *rd)
{
	int ret = 0;
	int timeout = 1000;
	unsigned long flags;

	spin_lock_irqsave(&rd->lock, flags);
	rd->state = RIIC_STATE_MASTER;
	rd->master_state = RIIC_MASTER_PREPARE;
	rd->pkt_index = 0;
	rd->completed = rd->nacked = rd->aled = 0;

	riic_set_bit(rd, ICIER_STIE | ICIER_NAKIE | ICIER_ALIE, RIIC_ICIER);
	riic_set_bit(rd, ICIER_RIE, RIIC_ICIER);
	riic_clear_bit(rd, ICIER_TEIE, RIIC_ICIER);

	/* Send START */
	riic_set_bit(rd, ICCR2_ST, RIIC_ICCR2);
	spin_unlock_irqrestore(&rd->lock, flags);

	ret = wait_event_interruptible_timeout(rd->wait, rd->completed ||
			rd->nacked || rd->aled, msecs_to_jiffies(timeout));
	if ((!rd->completed && ret == 0) || ret < 0) {
		riic_master_clean_up(rd, timeout);
		ret = RIIC_CORE_TIMEOUT;
	} else if (rd->nacked) {
		/* already issued stop in riic_irq_master_nackf() */
		if (rd->sending_addr)
			ret = RIIC_CORE_NACK_ADDR;
		else
			ret = RIIC_CORE_NACK_DATA;
	} else if (rd->aled) {
		if (rd->sending_addr)
			ret = RIIC_CORE_AL_ADDR;
		else
			ret = RIIC_CORE_AL_DATA;
	} else {
		ret = RIIC_CORE_NO_ERROR;
	}

	return ret;
}

static int riic_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs,
		     int num)
{
	struct riic_data *rd = i2c_get_adapdata(adapter);
	struct riic_core_packet *pkt;
	int i, ret = 0;

	if (riic_check_busy(rd))
		return -EBUSY;

	/* msgs to pkt */
	pkt = kzalloc(sizeof(struct riic_core_packet) * num, GFP_KERNEL);
	if (!pkt)
		return -ENOMEM;
	for (i = 0; i < num; i++) {
		pkt[i].slave_address = msgs[i].addr;
		if (msgs[i].flags & I2C_M_RD)
			pkt[i].rw = RIIC_CORE_RW_MASTER_RECEIVE;
		else
			pkt[i].rw = RIIC_CORE_RW_MASTER_TRANSMIT;
		pkt[i].len = msgs[i].len;
		pkt[i].data = msgs[i].buf;
	}
	rd->pkt = pkt;
	rd->num_pkt = num;

	ret = riic_master(rd);
	switch (ret) {
	case RIIC_CORE_NO_ERROR:
		ret = num;
		break;
	case RIIC_CORE_BUSY:
		ret = -EBUSY;
		break;
	case RIIC_CORE_TIMEOUT:
		ret = -ETIMEDOUT;
		break;
	case RIIC_CORE_AL_ADDR:
		ret = -EFAULT;
		break;
	case RIIC_CORE_AL_DATA:
		ret = -EIO;
		break;
	case RIIC_CORE_NACK_ADDR:
		ret = -ENXIO;
		break;
	case RIIC_CORE_NACK_DATA:
		ret = -EAGAIN;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	kfree(pkt);

	return ret;
}

static u32 riic_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm riic_algorithm = {
	.functionality	= riic_func,
	.master_xfer	= riic_xfer,
};

static int riic_remove(struct platform_device *pdev)
{
	struct riic_data *rd = platform_get_drvdata(pdev);
	int i;

	if (!rd)
		return 0;

	i2c_del_adapter(&rd->adap);
	for (i = 0; i < rd->irqn; i++)
		free_irq(rd->irq[i], rd);
	iounmap(rd->reg);
	clk_disable(rd->clk);
	clk_put(rd->clk);
	kfree(rd);

	return 0;
}

static int riic_master_is_sending_address(struct riic_data *rd)
{
	int ret = 0;

	switch (rd->master_state) {
	case RIIC_MASTER_PREPARE:
	case RIIC_MASTER_ISSUED_START:
		ret = 1;
		break;
	default:
		break;
	}

	return ret;
}

#if 0 /* NO SLAVE SUPPORT */
static void riic_slave_start_rx_timer(struct riic_data *rd)
{
	mod_timer(&rd->slv_timer, jiffies + msecs_to_jiffies(rd->slv_timeout));
}
#endif

static void riic_set_receive_ack(struct riic_data *rd, int ack)
{
	if (ack)
		riic_clear_bit(rd, ICMR3_ACKBT, RIIC_ICMR3);
	else
		riic_set_bit(rd, ICMR3_ACKBT, RIIC_ICMR3);
}

#if 0 /* NO SLAVE SUPPORT */
static void riic_slave_prepare(struct riic_data *rd)
{
	riic_set_bit(rd, ICIER_SPIE, RIIC_ICIER);

	if (!rd->slave_enabled)
		return;

	rd->state = RIIC_STATE_SLAVE;
	rd->slave_state = RIIC_SLAVE_ISSUED_START;
	riic_clear_bit(rd, ICIER_STIE, RIIC_ICIER);
	riic_set_receive_ack(rd, 1);
	riic_slave_start_rx_timer(rd);
}
#endif

static void riic_irq_master_al(struct riic_data *rd, unsigned char icsr2)
{
	rd->state = RIIC_STATE_IDLE;
	rd->sending_addr = riic_master_is_sending_address(rd);
	riic_master_clean_icier(rd);
#if 0 /* NO SLAVE SUPPORT */
	riic_slave_prepare(rd);	/* enter RIIC_STATE_SLAVE */
#endif
	rd->pkt = NULL;
	rd->aled = 1;
	wake_up_interruptible(&rd->wait);
}

static void riic_irq_master_nackf(struct riic_data *rd)
{
	if (rd->master_state != RIIC_MASTER_END) {
		rd->sending_addr = riic_master_is_sending_address(rd);
		rd->master_state = RIIC_MASTER_END;
		riic_clear_bit(rd, ICIER_TEIE, RIIC_ICIER);
		riic_master_issue_stop(rd);
	}

	rd->nacked = 1;
}

/* Master Mode */
static int riic_master_is_last_packet(struct riic_data *rd)
{
	if (rd->num_pkt - (rd->pkt_index + 1) == 0)
		return 1;
	else
		return 0;
}

static int riic_pkt_buf_data_len(struct riic_data *rd)
{
	return rd->pkt->len - rd->pkt->buf_idx;
}

static void riic_write_pkt_buf(struct riic_data *rd)
{
	struct riic_core_packet *pkt = rd->pkt;
	unsigned char *buf = (unsigned char *)rd->pkt->data;

	riic_write(rd, buf[pkt->buf_idx++], RIIC_ICDRT);
}

static void riic_master_update_packet(struct riic_data *rd)
{
	/* update current pkt */
	rd->pkt->done = 1;

	/* increment pkt pointer if needed */
	if (!riic_master_is_last_packet(rd)) {
		rd->pkt++;
		rd->pkt_index++;
		rd->pkt->buf_idx = 0;
	} else {
		rd->pkt = NULL;
	}
}

static int riic_enable_interrupt_transmit_end(struct riic_data *rd)
{
	if (!riic_pkt_buf_data_len(rd)) {
		riic_clear_bit(rd, ICIER_TEIE, RIIC_ICIER);
		return -1;
	}
	riic_set_bit(rd, ICIER_TEIE, RIIC_ICIER);
	return 0;
}

static void riic_master_kick_transmit(struct riic_data *rd)
{
	if (riic_enable_interrupt_transmit_end(rd))
		return;

	if (riic_pkt_buf_data_len(rd) >= 2) {
		if (rd->master_state != RIIC_MASTER_TRANSMITTING) {
			rd->master_state = RIIC_MASTER_TRANSMITTING;
			rd->pkt->buf_idx = 0;
		}
	} else if (riic_pkt_buf_data_len(rd) == 1) {
		rd->master_state = RIIC_MASTER_TRANSMIT_LAST;
	} else {
		dev_err(rd->dev, "got a zero buffer packet!\n");
	}
	riic_write_pkt_buf(rd);
}

static int riic_wait_for_icsr2(struct riic_data *rd, unsigned short bit);

static void riic_read_receive_data(struct riic_data *rd, int clear_wait)
{
	struct riic_core_packet *pkt = rd->pkt;
	unsigned char *buf = pkt->data;

	if (riic_wait_for_icsr2(rd, ICSR2_RDRF) < 0)
		;	/* An error message is already generated */

	buf[pkt->buf_idx++] = riic_read(rd, RIIC_ICDRR);
	if (clear_wait)
		riic_clear_bit(rd, ICMR3_WAIT, RIIC_ICMR3);
}

static void riic_send_start_cond(struct riic_data *rd, int restart)
{
	riic_set_bit(rd, ICIER_STIE, RIIC_ICIER);
	if (restart)
		riic_set_bit(rd, ICCR2_RS, RIIC_ICCR2);
	else
		riic_set_bit(rd, ICCR2_ST, RIIC_ICCR2);
}

static int riic_wait_for_icsr2(struct riic_data *rd, unsigned short bit)
{
	unsigned char icsr2;
	int timeout = RIIC_TIMEOUT;

	while (timeout-- > 0) {
		icsr2 = riic_read(rd, RIIC_ICSR2);
		if (icsr2 & ICSR2_NACKF)
			return -EIO;
		if (icsr2 & bit)
			return 0;
		udelay(10);
	}

	dev_err(rd->dev, "Timeout!(bit = %x icsr2 = %x, iccr2 = %x)\n",
		bit, riic_read(rd, RIIC_ICSR2), riic_read(rd, RIIC_ICCR2));

	return -ETIMEDOUT;
}

static void riic_packet_receive(struct riic_data *rd)
{
	if (riic_pkt_buf_data_len(rd) >= 2) {
		rd->master_state = RIIC_MASTER_RECEIVING;
		riic_read_receive_data(rd, 0);
		riic_set_receive_ack(rd, 1);

	} else if (riic_pkt_buf_data_len(rd) == 1) {
		riic_set_receive_ack(rd, 0);
		if (riic_master_is_last_packet(rd)) {
			rd->master_state = RIIC_MASTER_RECEIVE_LAST;
			riic_wait_for_icsr2(rd, ICSR2_RDRF);

			/* issue STOP */
			riic_master_issue_stop(rd);
			riic_read_receive_data(rd, 1);

		} else {
			riic_read_receive_data(rd, 1);
			riic_master_update_packet(rd);

			/* issue RESTART */
			riic_send_start_cond(rd, 1);
		}
	} else {
		dev_err(rd->dev, "got a zero buffer packet!\n");
	}
}

static void riic_master_kick_receive(struct riic_data *rd)
{
	rd->pkt->buf_idx = 0;
	riic_clear_bit(rd, ICIER_TIE, RIIC_ICIER);
	riic_clear_bit(rd, ICSR2_TDRE, RIIC_ICSR2);

	/* dummy read */
	riic_read(rd, RIIC_ICDRR);

	riic_packet_receive(rd);
}

static void riic_irq_master_tend(struct riic_data *rd)
{
	switch (rd->master_state) {
	case RIIC_MASTER_ISSUED_START:
		rd->master_state = RIIC_MASTER_SENT_SLAVE_ADDRESS;
		riic_master_kick_transmit(rd);
		break;
	case RIIC_MASTER_SENT_SLAVE_ADDRESS_NEXT:
		rd->master_state = RIIC_MASTER_SENT_SLAVE_ADDRESS;
		riic_master_kick_transmit(rd);
		break;
	case RIIC_MASTER_TRANSMITTING:
		break;
	case RIIC_MASTER_TRANSMIT_LAST:
		if (riic_master_is_last_packet(rd)) {
			/* issue STOP */
			riic_master_issue_stop(rd);
		} else {
			riic_master_update_packet(rd);
			/* issue RESTART */
			riic_set_bit(rd, ICIER_STIE, RIIC_ICIER);
			riic_set_bit(rd, ICCR2_RS, RIIC_ICCR2);
		}
		break;

	default:
		dev_err(rd->dev, "ch%d, unexpect master_state (%d)\n",
			rd->channel, rd->master_state);
		break;
	}
}

static void riic_irq_master_rdrf(struct riic_data *rd)
{
	switch (rd->master_state) {
	case RIIC_MASTER_ISSUED_START:
		riic_master_kick_receive(rd);
		break;
	case RIIC_MASTER_RECEIVING:
	case RIIC_MASTER_RECEIVE_LAST:
		riic_packet_receive(rd);
		break;
	case RIIC_MASTER_END:
		break;
	default:
		break;
	}
}

static void riic_irq_master_start(struct riic_data *rd, unsigned char icsr2)
{
	if (!(riic_read(rd, RIIC_ICCR2) & ICCR2_MST)) {
#if 0 /* NO SLAVE SUPPORT */
		riic_slave_prepare(rd);
#endif
		return;
	}

	switch (rd->master_state) {
	case RIIC_MASTER_TRANSMIT_LAST:
	case RIIC_MASTER_RECEIVE_LAST:
	case RIIC_MASTER_PREPARE:
		rd->master_state = RIIC_MASTER_ISSUED_START;
		if (icsr2 & ICSR2_TDRE)
			riic_send_slave_address(rd, 0);
		break;
	default:
		dev_err(rd->dev, "ch%d, unexpect master_state (%d)\n",
			rd->channel, rd->master_state);
		break;
	}
}

static void riic_irq_master_stop(struct riic_data *rd)
{
	switch (rd->master_state) {
	case RIIC_MASTER_TRANSMIT_LAST:
	case RIIC_MASTER_RECEIVE_LAST:
	case RIIC_MASTER_END:
		rd->master_state = RIIC_MASTER_ISSUED_STOP;
		riic_master_clean_icier(rd);

		rd->state = RIIC_STATE_IDLE;
		if (!rd->nacked) {
			riic_master_update_packet(rd);
			rd->completed = 1;
		}
		wake_up_interruptible(&rd->wait);
		break;
	case RIIC_MASTER_TRANSMITTING:
	case RIIC_MASTER_ISSUED_START:
		/* avoid to output the following message */
		break;
	default:
		/* riic_dump(rd, __func__); */
		dev_err(rd->dev, "ch%d, unexpect master_state (%d)\n",
			rd->channel, rd->master_state);
		break;
	}
}

static void riic_irq_master(struct riic_data *rd, unsigned char icsr2)
{
	if (icsr2 & ICSR2_AL) {
		riic_clear_bit(rd, ICSR2_AL, RIIC_ICSR2);
		riic_irq_master_al(rd, icsr2);
		return;
	}
	if (icsr2 & ICSR2_NACKF) {
		riic_clear_bit(rd, ICSR2_NACKF, RIIC_ICSR2);
		riic_irq_master_nackf(rd);
		return;
	}
	if (icsr2 & ICSR2_TEND && riic_read(rd, RIIC_ICIER) & ICIER_TEIE) {
		riic_clear_bit(rd, ICIER_TEIE, RIIC_ICIER);
		riic_irq_master_tend(rd);
	}
	if (icsr2 & ICSR2_RDRF) {
		riic_irq_master_rdrf(rd);
		riic_clear_bit(rd, ICSR2_RDRF, RIIC_ICSR2);
		return;
	}
	if (icsr2 & ICSR2_START) {
		riic_clear_bit(rd, ICIER_STIE, RIIC_ICIER);
		riic_clear_bit(rd, ICSR2_START, RIIC_ICSR2);
		riic_irq_master_start(rd, icsr2);
	}
	if (icsr2 & ICSR2_STOP) {
		riic_clear_bit(rd, ICIER_SPIE, RIIC_ICIER);
		riic_clear_bit(rd, ICSR2_STOP, RIIC_ICSR2);
		riic_irq_master_stop(rd);
	}
	if (icsr2 & ICSR2_TDRE) {
		riic_clear_bit(rd, ICSR2_TDRE, RIIC_ICSR2);
		if (rd->pkt->rw != RIIC_CORE_RW_MASTER_TRANSMIT)
			return;
		riic_master_kick_transmit(rd);
	}
}

/* Idle */
static void riic_irq_idle_start(struct riic_data *rd, unsigned char icsr2)
{
	unsigned char iccr2 = riic_read(rd, RIIC_ICCR2);

	if (iccr2 & ICCR2_MST) {
		dev_err(rd->dev, "No longer in slave mode %d,%02x,%02x\n",
			rd->channel, icsr2, iccr2);
		return;
	}
	if (!(iccr2 & ICCR2_BBSY)) {
		dev_err(rd->dev, "no busy %d, %02x\n",
			rd->channel, iccr2);
		return;
	}

#if 0 /* NO SLAVE SUPPORT */
	riic_slave_prepare(rd);
#endif
}

static void riic_irq_idle(struct riic_data *rd, unsigned char icsr2)
{
	if (icsr2 & ICSR2_AL)
		riic_clear_bit(rd, ICSR2_AL, RIIC_ICSR2);
	if (icsr2 & ICSR2_NACKF)
		riic_clear_bit(rd, ICSR2_NACKF, RIIC_ICSR2);
	if (icsr2 & ICSR2_TEND &&
	    riic_read(rd, RIIC_ICIER) & ICIER_TEIE)
		riic_clear_bit(rd, ICIER_TEIE, RIIC_ICIER);

	switch ((icsr2 & (ICSR2_START | ICSR2_STOP))) {
	case (ICSR2_START | ICSR2_STOP):
		/*
		 * If the following sequence, we should clear the STOP flag:
		 *  START -> NACK -> STOP ---> START -> ACK -> (IRQ happens)
		 *  Then, the RIIC's ICSR2 will be set (START | STOP).
		 */
		riic_clear_bit(rd, ICSR2_STOP, RIIC_ICSR2);
		/* through */
	case ICSR2_START:
		riic_clear_bit(rd, ICSR2_START, RIIC_ICSR2);
		riic_irq_idle_start(rd, icsr2);
		break;
	case ICSR2_STOP:
		riic_clear_bit(rd, ICSR2_STOP, RIIC_ICSR2);
		riic_clear_bit(rd, ICIER_SPIE, RIIC_ICIER);
		break;
	default:
		break;
	}
}

/* Slave Mode */
static void riic_irq_slave(struct riic_data *rd, unsigned char icsr2)
{
	dev_err(rd->dev, "This version driver doesn't support slave mode!\n");
	return;
}

static irqreturn_t riic_irq(int irq, void *data)
{
	struct riic_data *rd = data;
	unsigned char icsr2 = riic_read(rd, RIIC_ICSR2);

	rd->old_state = rd->state;
	rd->old_icsr2 = icsr2;

	switch (rd->state) {
	case RIIC_STATE_MASTER:
		riic_irq_master(rd, icsr2);
		break;
	case RIIC_STATE_SLAVE:
		riic_irq_slave(rd, icsr2);
		break;
	case RIIC_STATE_IDLE:
		riic_irq_idle(rd, icsr2);
		break;

	default:
		break;
	}
	return IRQ_HANDLED;
}

static int riic_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	struct riic_data *rd = NULL;
	struct riic_platform_data *riic_data = NULL;
	struct i2c_adapter *adap;
	void __iomem *reg = NULL;
	int i, irq, j = 0;
	int ret = 0;
	char clk_name[16];

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "platform_get_resource failed.\n");
		goto clean_up;
	}

	if (!pdev->dev.platform_data) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "no platform data\n");
		goto clean_up;
	}
	riic_data = pdev->dev.platform_data;

	reg = ioremap(res->start, resource_size(res));
	if (reg == NULL) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "ioremap failed.\n");
		goto clean_up;
	}

	rd = kzalloc(sizeof(struct riic_data), GFP_KERNEL);
	if (rd == NULL) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "kzalloc failed.\n");
		goto clean_up;
	}

	rd->dev = &pdev->dev;
	rd->reg = reg;
	platform_set_drvdata(pdev, rd);

	adap = &rd->adap;
	i2c_set_adapdata(adap, rd);

	adap->owner = THIS_MODULE;
	adap->algo = &riic_algorithm;
	adap->dev.parent = &pdev->dev;
	adap->retries = 5;
	rd->channel = adap->nr = pdev->id;

	strlcpy(adap->name, dev_name(&pdev->dev), sizeof(adap->name));
	if (pdev->id < RIIC_NUM_CHANNELS) {
		rd->name = riic_name[pdev->id];
	} else {
		ret = -ENXIO;
		dev_err(&pdev->dev, "invalid platform device id.\n");
		goto clean_up;
	}

	snprintf(clk_name, sizeof(clk_name), "riic%d", pdev->id);
	rd->clk = clk_get(&pdev->dev, clk_name);
	if (IS_ERR(rd->clk)) {
		dev_err(&pdev->dev, "clk_get failed.\n");
		ret = PTR_ERR(rd->clk);
		goto clean_up;
	}
	clk_enable(rd->clk);

	ret = riic_init_setting(rd, riic_data->clock);
	if (ret < 0) {
		dev_err(&pdev->dev, "riic_init_setting failed.\n");
		goto clean_up;
	}

	i = j = 0;
	while ((res = platform_get_resource(pdev, IORESOURCE_IRQ, i++))) {
		for (irq = res->start; irq <= res->end; irq++) {
			if (j >= NUM_IRQ) {
				dev_err(&pdev->dev, "irq resource is over\n");
				ret = -ENODEV;
				goto clean_up;
			}
			if (res->flags & IORESOURCE_IRQ_HIGHEDGE)
				irq_set_irq_type(irq, IRQ_TYPE_EDGE_RISING);
			ret = request_irq(irq, riic_irq, IRQF_DISABLED,
					  rd->name, rd);
			if (ret < 0) {
				dev_err(&pdev->dev, "request_irq error\n");
				goto clean_up;
			}
			rd->irq[j++] = irq;
		}
	}
	rd->irqn = j;

	init_waitqueue_head(&rd->wait);

	ret = i2c_add_numbered_adapter(adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "i2c_add_numbered_adapter failed.\n");
		goto clean_up;
	}

#ifdef CONFIG_ARCH_R7S72100
	/* I2C pfc pin assign after resetting. */
//	rskrza1_board_i2c_pfc_assign(pdev->id);
#endif

	dev_info(&pdev->dev, "version %s: %d[kbps]\n",
		DRIVER_VERSION, riic_data->clock);
	return ret;

clean_up:
	for (i = 0; i < j; i++)
		free_irq(rd->irq[i], rd);
	if (rd) {
		if (!IS_ERR(rd->clk)) {
			clk_disable(rd->clk);
			clk_put(rd->clk);
		}
		kfree(rd);
	}
	if (reg)
		iounmap(reg);
	platform_set_drvdata(pdev, NULL);

	return ret;
}

static struct platform_driver riic_driver = {
	.probe =	riic_probe,
	.remove =	riic_remove,
	.driver		= {
		.name	= "i2c-riic",
		.owner	= THIS_MODULE,
	},
};

static int __init riic_init(void)
{
	return platform_driver_register(&riic_driver);
}
module_init(riic_init);

static void __exit riic_cleanup(void)
{
	platform_driver_unregister(&riic_driver);
}
module_exit(riic_cleanup);

MODULE_DESCRIPTION("Renesas RIIC Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yoshihiro Shimoda");
MODULE_ALIAS("platform:i2c-riic");
