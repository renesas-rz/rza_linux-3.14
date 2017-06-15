/*
 * rza1_can.c - CAN network driver for RZ/A1 SoC CAN controller
 *
 * (C) 2014 by Carlo Caione <carlo@caione.org>
 *
 * This software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2 as distributed in the 'COPYING'
 * file from the main directory of the linux kernel source.
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>
#include <linux/can/error.h>
#include <linux/can/dev.h>
#include <linux/can/platform/rza1_can.h>
//asdf #include <mach/rza1.h>

enum {
	CFM_RX_MODE = 0,
	CFM_TX_MODE,
	CFM_GW_MODE
};

enum {
	OP_MODE = 0,
	RST_MODE,
	TEST_MODE
};

#define DRV_NAME			"rz_can"
#define RZ_CAN_TX_ECHO_SKB_MAX		128
#define RZ_CAN_INC_BUF_ID(x)		(((x) + 1) % RZ_CAN_TX_ECHO_SKB_MAX)

#define RZ_CAN_CFCD_FULL		0x07
#define RZ_CAN_CFTML			0

#define RZ_CAN_RX_FIFO			0
#define RZ_CAN_TX_FIFO			1

#define RZ_CAN_FIFO_K(m,n)		(((m) * 3) + (n))

#define RZ_CAN_GAFLID_FIFO_M(x)		BIT(x)
#define RZ_CAN_GAFLID_TXRX_FIFO_M(x)	BIT(((x) + 8))

#define RZ_CAN_RSCAN0GCFG		0x0084
#define RZ_CAN_RSCAN0GCFG_DCS(x)	((x) << 4)

#define RZ_CAN_RSCAN0GSTS		0x008c
#define RZ_CAN_RSCAN0GSTS_GRAMINIT	BIT(3)

#define RZ_CAN_RSCAN0CmCFG(m)		(0x0000 + (m * 0x0010))
#define RZ_CAN_RSCAN0CmCFG_SJW(x)	(((x) & 0x03) << 24)
#define RZ_CAN_RSCAN0CmCFG_TSEG2(x)	(((x) & 0x07) << 20)
#define RZ_CAN_RSCAN0CmCFG_TSEG1(x)	(((x) & 0x0f) << 16)
#define RZ_CAN_RSCAN0CmCFG_BRP(x)	(((x) & 0x1ff) << 0)

#define RZ_CAN_RSCAN0GCTR		0x0088
#define RZ_CAN_RSCAN0GCTR_GSLPR		BIT(2)
#define RZ_CAN_RSCAN0GCTR_MEIE		BIT(9)

#define RZ_CAN_RSCAN0GCTR_GMDC_M	0x00000003
#define RZ_CAN_RSCAN0GCTR_GMDC(x)	(((x) & 0x03) << 0)

#define RZ_CAN_RSCAN0CmCTR(m)		(0x0004 + ((m) * 0x0010))
#define RZ_CAN_RSCAN0CmCTR_CSLPR	BIT(2)
#define RZ_CAN_RSCAN0CmCTR_BEIE		BIT(8)
#define RZ_CAN_RSCAN0CmCTR_EWIE		BIT(9)
#define RZ_CAN_RSCAN0CmCTR_EPIE		BIT(10)
#define RZ_CAN_RSCAN0CmCTR_BOEIE	BIT(11)
#define RZ_CAN_RSCAN0CmCTR_OLIE		BIT(13)

#define RZ_CAN_RSCAN0CmCTR_CHMDC_M	0x00000003
#define RZ_CAN_RSCAN0CmCTR_CHMDC(x)	(((x) & 0x03) << 0)

#define RZ_CAN_RSCAN0GAFLCFG0		0x009c
#define RZ_CAN_RSCAN0GAFLCFG0_RNC(m,x)	(((x) & 0xff) << (8 * (3 - (m))))

#define RZ_CAN_RSCAN0GAFLECTR		0x0098
#define RZ_CAN_RSCAN0GAFLECTR_AFLDAE	BIT(8)
#define RZ_CAN_RSCAN0GAFLECTR_AFLPN(x)	(((x) & 0x1f) << 0)

#define RZ_CAN_RSCAN0GAFLIDj(j)		(0x0500 + ((j) * 0x0010))
#define RZ_CAN_RSCAN0GAFLMj(j)		(0x0504 + ((j) * 0x0010))
#define RZ_CAN_RSCAN0GAFLP0j(j)		(0x0508 + ((j) * 0x0010))
#define RZ_CAN_RSCAN0GAFLP1j(j)		(0x050c + ((j) * 0x0010))

#define RZ_CAN_RSCAN0CFCCk(k)		(0x0118 + ((k) * 0x0004))
#define RZ_CAN_RSCAN0CFCCk_CFE		BIT(0)
#define RZ_CAN_RSCAN0CFCCk_CFRXIE	BIT(1)
#define RZ_CAN_RSCAN0CFCCk_CFTXIE	BIT(2)
#define RZ_CAN_RSCAN0CFCCk_CFIM		BIT(12)
#define RZ_CAN_RSCAN0CFCCk_CFITSS	BIT(18)
#define RZ_CAN_RSCAN0CFCCk_CFITR	BIT(19)
#define RZ_CAN_RSCAN0CFCCk_CFDC(x)	(((x) & 0x07) << 8)
#define RZ_CAN_RSCAN0CFCCk_CFIGCV(x)	(((x) & 0x07) << 13)
#define RZ_CAN_RSCAN0CFCCk_CFM(x)	(((x) & 0x03) << 16)
#define RZ_CAN_RSCAN0CFCCk_CFTML(x)	(((x) & 0x0f) << 20)
#define RZ_CAN_RSCAN0CFCCk_CFITT(x)	(((x) & 0xff) << 24)

#define RZ_CAN_RSCAN0CmSTS(m)		(0x0008 + ((m) * 0x0010))
#define RZ_CAN_RSCAN0CmSTS_TEC(x)	(((x) & 0xff000000) >> 24)
#define RZ_CAN_RSCAN0CmSTS_REC(x)	(((x) & 0x00ff0000) >> 16)

#define RZ_CAN_RSCAN0CmERFL(m)		(0x000c + ((m) * 0x0010))
#define RZ_CAN_RSCAN0CmERFL_BEF		BIT(0)
#define RZ_CAN_RSCAN0CmERFL_EWF		BIT(1)
#define RZ_CAN_RSCAN0CmERFL_EPF		BIT(2)
#define RZ_CAN_RSCAN0CmERFL_BOEF	BIT(3)
#define RZ_CAN_RSCAN0CmERFL_OVLF	BIT(5)
#define RZ_CAN_RSCAN0CmERFL_SERR	BIT(8)
#define RZ_CAN_RSCAN0CmERFL_FERR	BIT(9)
#define RZ_CAN_RSCAN0CmERFL_AERR	BIT(10)
#define RZ_CAN_RSCAN0CmERFL_CERR	BIT(11)
#define RZ_CAN_RSCAN0CmERFL_B1ERR	BIT(12)
#define RZ_CAN_RSCAN0CmERFL_B0ERR	BIT(13)
#define RZ_CAN_RSCAN0CmERFL_ADERR	BIT(14)

#define RZ_CAN_RSCAN0CFSTSk(k)		(0x0178 + ((k) * 0x0004))
#define RZ_CAN_RSCAN0CFSTSk_CFEMP	BIT(0)
#define RZ_CAN_RSCAN0CFSTSk_CFFLL	BIT(1)
#define RZ_CAN_RSCAN0CFSTSk_CFMLT	BIT(2)
#define RZ_CAN_RSCAN0CFSTSk_CFRXIF	BIT(3)
#define RZ_CAN_RSCAN0CFSTSk_CFTXIF	BIT(4)

#define RZ_CAN_RSCAN0CFIDk(k)		(0x0e80 + ((k) * 0x0010))
#define RZ_CAN_RSCAN0CFIDk_CFID_M	0x1fffffff
#define RZ_CAN_RSCAN0CFIDk_CFRTR	BIT(30)
#define RZ_CAN_RSCAN0CFIDk_CFIDE	BIT(31)

#define RZ_CAN_RSCAN0CFPTRk(k)		(0x0e84 + ((k) * 0x0010))
#define RZ_CAN_RSCAN0CFPTRk_CFDLC(x)	(((x) & 0x0f) << 28)
#define RZ_CAN_RSCAN0CFPTRk_CFDLC_G(x)	(((x) & 0xf0000000) >> 28)

#define RZ_CAN_RSCAN0CFDFbk(k,b)	((0x0e88 + ((b) * 0x04)) + ((k) * 0x0010))
#define RZ_CAN_RSCAN0CFDFbk_CFDB(x,s)	(((x) & 0xff) << ((s) * 8))

#define RZ_CAN_RSCAN0CFPCTRk(k)		(0x01d8 + ((k) * 0x0004))

struct rz_can_priv {
	struct can_priv can;	/* must be the first member */
	struct net_device *ndev;
	struct clk *clk;
	spinlock_t skb_lock;
	void __iomem *base;
	unsigned int bytes_queued;
	int frames_queued;
	int clock_select;
	int m;
	int k_rx;
	int k_tx;
	int rx_irq;
	int tx_irq;
	int err_irq_m;
	int err_irq_g;
};

static const struct can_bittiming_const rz_can_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 4,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 4,
	.brp_max = 1024,
	.brp_inc = 1,
};

static void rz_can_write(struct rz_can_priv *priv, unsigned long reg_offs,
			 u32 data)
{
	iowrite32(data, priv->base + reg_offs);
}

static u32 rz_can_read(struct rz_can_priv *priv, unsigned long reg_offs)
{
	return ioread32(priv->base + reg_offs);
}

static int rz_can_set_bittiming(struct net_device *ndev)
{
	struct rz_can_priv *priv = netdev_priv(ndev);
	struct can_bittiming *bt = &priv->can.bittiming;
	u32 cfg, dcs;

	dcs = RZ_CAN_RSCAN0GCFG_DCS(priv->clock_select);
	rz_can_write(priv, RZ_CAN_RSCAN0GCFG, dcs);

	cfg = RZ_CAN_RSCAN0CmCFG_SJW(bt->sjw - 1);
	cfg |= RZ_CAN_RSCAN0CmCFG_BRP(bt->brp - 1);
	cfg |= RZ_CAN_RSCAN0CmCFG_TSEG1(bt->phase_seg1 + bt->prop_seg - 1);
	cfg |= RZ_CAN_RSCAN0CmCFG_TSEG2(bt->phase_seg2 - 1);
	rz_can_write(priv, RZ_CAN_RSCAN0CmCFG(priv->m), cfg);

	return 0;
}

static int rz_can_get_berr_counter(const struct net_device *ndev,
				   struct can_berr_counter *bec)
{
	struct rz_can_priv *priv = netdev_priv(ndev);
	u32 reg;

	reg = rz_can_read(priv, RZ_CAN_RSCAN0CmSTS(priv->m));
	bec->txerr = RZ_CAN_RSCAN0CmSTS_TEC(reg);
	bec->rxerr = RZ_CAN_RSCAN0CmSTS_REC(reg);

	return 0;
}

static int rz_can_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct rz_can_priv *priv = netdev_priv(ndev);
	struct can_frame *cf = (struct can_frame *)skb->data;
	unsigned long flags;
	u8 dlc = cf->can_dlc;
	canid_t id = cf->can_id;
	u8 *data = cf->data;
	u32 reg;
	int b, s;

	reg = rz_can_read(priv, RZ_CAN_RSCAN0CFCCk(priv->k_tx));
	reg |= RZ_CAN_RSCAN0CFCCk_CFE;
	rz_can_write(priv, RZ_CAN_RSCAN0CFCCk(priv->k_tx), reg);

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	reg = rz_can_read(priv, RZ_CAN_RSCAN0CFSTSk(priv->k_tx));
	if (reg & RZ_CAN_RSCAN0CFSTSk_CFFLL)
		netif_stop_queue(ndev);

	if (id & CAN_EFF_FLAG) {
		/* Extended frame format */
		reg = (id & CAN_EFF_MASK) | RZ_CAN_RSCAN0CFIDk_CFIDE;
	} else {
		/* Standard frame format */
		reg = (id & CAN_SFF_MASK);
	}

	if (id & CAN_RTR_FLAG) {
		/* Remote transmission request */
		reg |= RZ_CAN_RSCAN0CFIDk_CFRTR;
	}

	rz_can_write(priv, RZ_CAN_RSCAN0CFIDk(priv->k_tx), reg);

	rz_can_write(priv, RZ_CAN_RSCAN0CFPTRk(priv->k_tx),
		     RZ_CAN_RSCAN0CFPTRk_CFDLC(dlc));

	for (b = 0; b < 2; b++) {
		reg = 0;
		for (s = 0; s < 4; s++)
			reg |= RZ_CAN_RSCAN0CFDFbk_CFDB(data[(b * 4) + s], s);
		rz_can_write(priv, RZ_CAN_RSCAN0CFDFbk(priv->k_tx, b), reg);
	}

	spin_lock_irqsave(&priv->skb_lock, flags);
	can_put_echo_skb(skb, ndev, priv->frames_queued++);
	priv->bytes_queued += dlc;
	spin_unlock_irqrestore(&priv->skb_lock, flags);

	rz_can_write(priv, RZ_CAN_RSCAN0CFPCTRk(priv->k_tx), 0xff);

	return 0;
}

static void rz_can_rx_pkt(struct net_device *ndev)
{
	struct rz_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct sk_buff *skb;
	struct can_frame *cf;
	int b, s;
	u32 reg;

	while (!(rz_can_read(priv, RZ_CAN_RSCAN0CFSTSk(priv->k_rx)) & RZ_CAN_RSCAN0CFSTSk_CFEMP)) {

		skb = alloc_can_skb(ndev, &cf);
		if (!skb) {
			stats->rx_dropped++;
			return;
		}

		reg = rz_can_read(priv, RZ_CAN_RSCAN0CFIDk(priv->k_rx));

		if (reg & RZ_CAN_RSCAN0CFIDk_CFIDE) {
			/* Extended ID */
			cf->can_id = (reg & CAN_EFF_MASK) | CAN_EFF_FLAG;
		} else {
			/* Standard ID */
			cf->can_id = (reg & CAN_SFF_MASK);
		}

		if (reg & RZ_CAN_RSCAN0CFIDk_CFRTR)
			cf->can_id |= CAN_RTR_FLAG;

		for (b = 0; b < 2; b++) {
			reg = rz_can_read(priv, RZ_CAN_RSCAN0CFDFbk(priv->k_rx, b));
//			printk(KERN_EMERG "[CAN-rx-pkt] 0x%08X\n", reg);
			for (s = 0; s < 4; s++)	{
				cf->data[(b * 4) + s] = reg & 0x000000ff;
				reg >>= 8;
			}
		}

		reg = rz_can_read(priv, RZ_CAN_RSCAN0CFPTRk(priv->k_rx));
		cf->can_dlc = RZ_CAN_RSCAN0CFPTRk_CFDLC_G(reg);

		rz_can_write(priv, RZ_CAN_RSCAN0CFPCTRk(priv->k_rx), 0xff);
		netif_rx(skb);

		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;
	}
}

static void rz_can_tx_failure_cleanup(struct net_device *ndev)
{
	int i;

	for (i = 0; i < RZ_CAN_TX_ECHO_SKB_MAX; i++)
		can_free_echo_skb(ndev, i);	//asdf BUG: Can't call from ISR
}

static void rz_can_err(struct net_device *ndev)
{
	struct rz_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	u8 txerr = 0, rxerr = 0;
	u32 reg;

	skb = alloc_can_err_skb(ndev, &cf);
	if (!skb)
		return;

	reg = rz_can_read(priv, RZ_CAN_RSCAN0CmERFL(priv->m));
	if (reg & (RZ_CAN_RSCAN0CmERFL_EPF | RZ_CAN_RSCAN0CmERFL_EWF)) {
		u32 sts = rz_can_read(priv, RZ_CAN_RSCAN0CmSTS(priv->m));
		cf->can_id |= CAN_ERR_CRTL;
		txerr = RZ_CAN_RSCAN0CmSTS_TEC(sts);
		rxerr = RZ_CAN_RSCAN0CmSTS_REC(sts);
		cf->data[6] = txerr;
		cf->data[7] = rxerr;
	}

	if (reg & RZ_CAN_RSCAN0CmERFL_BEF) {
		int rx_errors = 0, tx_errors = 0;

		netdev_dbg(priv->ndev, "Bus error interrupt:\n");
		cf->can_id |= CAN_ERR_BUSERROR | CAN_ERR_PROT;
		cf->data[2] = CAN_ERR_PROT_UNSPEC;

		if (reg & RZ_CAN_RSCAN0CmERFL_ADERR) {
			netdev_dbg(priv->ndev, "ACK Delimiter Error\n");
			cf->data[3] |= CAN_ERR_PROT_LOC_ACK_DEL;
			tx_errors++;
			reg &= ~RZ_CAN_RSCAN0CmERFL_ADERR;
		}

		if (reg & RZ_CAN_RSCAN0CmERFL_B0ERR) {
			netdev_dbg(priv->ndev, "Bit Error (dominant)\n");
			cf->data[2] |= CAN_ERR_PROT_BIT0;
			tx_errors++;
			reg &= ~RZ_CAN_RSCAN0CmERFL_B0ERR;
		}

		if (reg & RZ_CAN_RSCAN0CmERFL_B1ERR) {
			netdev_dbg(priv->ndev, "Bit Error (recessive)\n");
			cf->data[2] |= CAN_ERR_PROT_BIT1;
			tx_errors++;
			reg &= ~RZ_CAN_RSCAN0CmERFL_B1ERR;
		}

		if (reg & RZ_CAN_RSCAN0CmERFL_CERR) {
			netdev_dbg(priv->ndev, "CRC Error\n");
			cf->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
			rx_errors++;
			reg &= ~RZ_CAN_RSCAN0CmERFL_CERR;
		}

		if (reg & RZ_CAN_RSCAN0CmERFL_AERR) {
			netdev_dbg(priv->ndev, "ACK Error\n");
			cf->can_id |= CAN_ERR_ACK;
			cf->data[3] |= CAN_ERR_PROT_LOC_ACK;
			tx_errors++;
			reg &= ~RZ_CAN_RSCAN0CmERFL_AERR;
		}

		if (reg & RZ_CAN_RSCAN0CmERFL_FERR) {
			netdev_dbg(priv->ndev, "Form Error\n");
			cf->data[2] |= CAN_ERR_PROT_FORM;
			rx_errors++;
			reg &= ~RZ_CAN_RSCAN0CmERFL_FERR;
		}

		if (reg & RZ_CAN_RSCAN0CmERFL_SERR) {
			netdev_dbg(priv->ndev, "Stuff Error\n");
			cf->data[2] |= CAN_ERR_PROT_STUFF;
			rx_errors++;
			reg &= ~RZ_CAN_RSCAN0CmERFL_SERR;
		}

		priv->can.can_stats.bus_error++;
		ndev->stats.rx_errors += rx_errors;
		ndev->stats.tx_errors += tx_errors;

		reg &= ~RZ_CAN_RSCAN0CmERFL_BEF;
		rz_can_write(priv, RZ_CAN_RSCAN0CmERFL(priv->m), reg);
	}

	if (reg & RZ_CAN_RSCAN0CmERFL_EWF) {
		netdev_dbg(priv->ndev, "Error warning interrupt\n");
		priv->can.state = CAN_STATE_ERROR_WARNING;
		priv->can.can_stats.error_warning++;
		cf->data[1] |= txerr > rxerr ? CAN_ERR_CRTL_TX_WARNING :
					       CAN_ERR_CRTL_RX_WARNING;
		reg &= ~RZ_CAN_RSCAN0CmERFL_EWF;
		rz_can_write(priv, RZ_CAN_RSCAN0CmERFL(priv->m), reg);
	}

	if (reg & RZ_CAN_RSCAN0CmERFL_EPF) {
		netdev_dbg(priv->ndev, "Error passive interrupt\n");
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
		priv->can.can_stats.error_passive++;
		cf->data[1] |= txerr > rxerr ? CAN_ERR_CRTL_TX_PASSIVE :
					       CAN_ERR_CRTL_RX_PASSIVE;
		reg &= ~RZ_CAN_RSCAN0CmERFL_EPF;
		rz_can_write(priv, RZ_CAN_RSCAN0CmERFL(priv->m), reg);
	}

	if (reg & RZ_CAN_RSCAN0CmERFL_BOEF) {
		netdev_dbg(priv->ndev, "Bus-off entry interrupt\n");
		rz_can_tx_failure_cleanup(ndev);
		priv->can.state = CAN_STATE_BUS_OFF;
		cf->can_id |= CAN_ERR_BUSOFF;
		reg &= ~RZ_CAN_RSCAN0CmERFL_BOEF;
		rz_can_write(priv, RZ_CAN_RSCAN0CmERFL(priv->m), reg);
		can_bus_off(ndev);
	}

	if (reg & RZ_CAN_RSCAN0CmERFL_OVLF) {
		netdev_dbg(priv->ndev, "Overload Frame Transmission error interrupt\n");
		cf->can_id |= CAN_ERR_PROT;
		cf->data[2] |= CAN_ERR_PROT_OVERLOAD;
		ndev->stats.rx_over_errors++;
		ndev->stats.rx_errors++;
		reg &= ~RZ_CAN_RSCAN0CmERFL_OVLF;
		rz_can_write(priv, RZ_CAN_RSCAN0CmERFL(priv->m), reg);
	}

	netif_rx(skb);
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
}

irqreturn_t rz_can_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct rz_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	u32 reg_tx, reg_rx;
	int i;

	reg_tx = rz_can_read(priv, RZ_CAN_RSCAN0CFSTSk(priv->k_tx));
	reg_rx = rz_can_read(priv, RZ_CAN_RSCAN0CFSTSk(priv->k_rx));

	if ((irq == priv->tx_irq) && (reg_tx & RZ_CAN_RSCAN0CFSTSk_CFTXIF)) {
		spin_lock(&priv->skb_lock);
		for (i = 0; i < priv->frames_queued; i++)
			can_get_echo_skb(ndev, i);
		stats->tx_bytes += priv->bytes_queued;
		stats->tx_packets += priv->frames_queued;
		priv->bytes_queued = 0;
		priv->frames_queued = 0;
		spin_unlock(&priv->skb_lock);

		netif_wake_queue(ndev);

		reg_tx &= ~RZ_CAN_RSCAN0CFSTSk_CFTXIF;
		rz_can_write(priv, RZ_CAN_RSCAN0CFSTSk(priv->k_tx), reg_tx);
	}

	if ((irq == priv->rx_irq) && (reg_rx & RZ_CAN_RSCAN0CFSTSk_CFRXIF)) {
		rz_can_rx_pkt(ndev);
		reg_rx &= ~RZ_CAN_RSCAN0CFSTSk_CFRXIF;
		rz_can_write(priv, RZ_CAN_RSCAN0CFSTSk(priv->k_rx), reg_rx);
	}

	if (irq == priv->err_irq_g) {
		reg_tx &= ~RZ_CAN_RSCAN0CFSTSk_CFMLT;
		reg_rx &= ~RZ_CAN_RSCAN0CFSTSk_CFMLT;

		rz_can_write(priv, RZ_CAN_RSCAN0CFSTSk(priv->k_tx), reg_tx);
		rz_can_write(priv, RZ_CAN_RSCAN0CFSTSk(priv->k_rx), reg_rx);

		netdev_dbg(ndev, "A transmit/receive FIFO message is lost.\n");
	}

	if (irq == priv->err_irq_m)
		rz_can_err(ndev);

	return IRQ_HANDLED;
}

static int rz_can_wait(struct rz_can_priv *priv, unsigned long offset,
		       unsigned int mask, unsigned int ms_timeout)
{
	const unsigned long timeout = jiffies + msecs_to_jiffies(ms_timeout);
	u32 reg;

	do {
		reg = rz_can_read(priv, offset);
		reg = ~reg;
		reg &= mask;

		if (reg == mask)
			return 0;

	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

static int rz_can_start(struct net_device *ndev)
{
	struct rz_can_priv *priv = netdev_priv(ndev);
	u32 reg;

	if (rz_can_wait(priv, RZ_CAN_RSCAN0GSTS, RZ_CAN_RSCAN0GSTS_GRAMINIT, 50))
		return -ETIMEDOUT;

	/* Go to reset mode */
	reg = rz_can_read(priv, RZ_CAN_RSCAN0GCTR);
	reg &= ~RZ_CAN_RSCAN0GCTR_GSLPR;
	rz_can_write(priv, RZ_CAN_RSCAN0GCTR, reg);

	/* From channel stop mode to channel reset mode */
	reg = rz_can_read(priv, RZ_CAN_RSCAN0CmCTR(priv->m));
	reg &= ~RZ_CAN_RSCAN0CmCTR_CSLPR;
	rz_can_write(priv, RZ_CAN_RSCAN0CmCTR(priv->m), reg);

	/* Clock and bittiming */
	rz_can_set_bittiming(ndev);

	/* Receive rule setting */

	/* 1 rule for channel m */
	rz_can_write(priv, RZ_CAN_RSCAN0GAFLCFG0,
		      RZ_CAN_RSCAN0GAFLCFG0_RNC(priv->m, 1));

	/* Page 0 */
	reg = RZ_CAN_RSCAN0GAFLECTR_AFLDAE;
	reg |= RZ_CAN_RSCAN0GAFLECTR_AFLPN(0);
	rz_can_write(priv, RZ_CAN_RSCAN0GAFLECTR, reg);

	/* Create the receive rule */
	rz_can_write(priv, RZ_CAN_RSCAN0GAFLMj(0), 0);
	rz_can_write(priv, RZ_CAN_RSCAN0GAFLP0j(0), 0);
	rz_can_write(priv, RZ_CAN_RSCAN0GAFLP1j(0),
		     RZ_CAN_GAFLID_TXRX_FIFO_M(priv->k_rx));

	reg = rz_can_read(priv, RZ_CAN_RSCAN0GAFLECTR);
	reg &= ~RZ_CAN_RSCAN0GAFLECTR_AFLDAE;
	rz_can_write(priv, RZ_CAN_RSCAN0GAFLECTR, reg);

	/* Buffer settings */

	/* RX: transmit/receive FIFO buffer (full depth, IRQ for each ptk) */
	reg = RZ_CAN_RSCAN0CFCCk_CFM(CFM_RX_MODE);
	reg |= RZ_CAN_RSCAN0CFCCk_CFIM;
	reg |= RZ_CAN_RSCAN0CFCCk_CFDC(RZ_CAN_CFCD_FULL);
	reg |= RZ_CAN_RSCAN0CFCCk_CFRXIE;
//	reg |= RZ_CAN_RSCAN0CFCCk_CFE;
	rz_can_write(priv, RZ_CAN_RSCAN0CFCCk(priv->k_rx), reg);

	/* TX: transmit/receive FIFO buffer (full depth, IRQ for each ptk) */
	reg = RZ_CAN_RSCAN0CFCCk_CFM(CFM_TX_MODE);
	reg |= RZ_CAN_RSCAN0CFCCk_CFIM;
	reg |= RZ_CAN_RSCAN0CFCCk_CFDC(RZ_CAN_CFCD_FULL);
	reg |= RZ_CAN_RSCAN0CFCCk_CFTML(RZ_CAN_CFTML);
	reg |= RZ_CAN_RSCAN0CFCCk_CFTXIE;
	rz_can_write(priv, RZ_CAN_RSCAN0CFCCk(priv->k_tx), reg);

	/* RSCAN0CmCTR / RSCAN0CmCTR register setting */
	reg = rz_can_read(priv, RZ_CAN_RSCAN0GCTR);
	reg |= RZ_CAN_RSCAN0GCTR_MEIE;
	rz_can_write(priv, RZ_CAN_RSCAN0GCTR, reg);

	reg = rz_can_read(priv, RZ_CAN_RSCAN0CmCTR(priv->m));
	reg |= (RZ_CAN_RSCAN0CmCTR_EPIE | RZ_CAN_RSCAN0CmCTR_EWIE);
	reg |= (RZ_CAN_RSCAN0CmCTR_BOEIE | RZ_CAN_RSCAN0CmCTR_BEIE);
	reg |= RZ_CAN_RSCAN0CmCTR_OLIE;
	rz_can_write(priv, RZ_CAN_RSCAN0CmCTR(priv->m), reg);

	/* Transition to global operating mode */
	reg = rz_can_read(priv, RZ_CAN_RSCAN0GCTR);
	reg &= ~RZ_CAN_RSCAN0GCTR_GMDC_M;
	reg |= RZ_CAN_RSCAN0GCTR_GMDC(OP_MODE);
	rz_can_write(priv, RZ_CAN_RSCAN0GCTR, reg);

	/* Transition to channel comm mode */
	reg = rz_can_read(priv, RZ_CAN_RSCAN0CmCTR(priv->m));
	reg &= ~RZ_CAN_RSCAN0CmCTR_CHMDC_M;
	reg |= RZ_CAN_RSCAN0CmCTR_CHMDC(OP_MODE);
	rz_can_write(priv, RZ_CAN_RSCAN0CmCTR(priv->m), reg);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	reg = rz_can_read(priv, RZ_CAN_RSCAN0CFCCk(priv->k_rx));
	reg |= RZ_CAN_RSCAN0CFCCk_CFE;
	rz_can_write(priv, RZ_CAN_RSCAN0CFCCk(priv->k_rx), reg);


	return 0;
}

static int rz_can_open(struct net_device *ndev)
{
	struct rz_can_priv *priv = netdev_priv(ndev);
	int err;

	err = clk_enable(priv->clk);
	if (err < 0)
		goto exit_open;

	/* common open */
	err = open_candev(ndev);
	if (err)
		goto exit_open;

	/* register interrupt handler */
	err = request_irq(priv->rx_irq, &rz_can_interrupt, 0,
			"rz-can-rx", ndev);
	if (err)
		goto exit_rx_irq;

	err = request_irq(priv->tx_irq, &rz_can_interrupt, 0,
			"rz-can-tx", ndev);
	if (err)
		goto exit_tx_irq;

	err = request_irq(priv->err_irq_m, &rz_can_interrupt, 0,
			"rz-can-err-m", ndev);
	if (err)
		goto exit_err_irq_m;

	err = request_irq(priv->err_irq_g, &rz_can_interrupt, 0,
			"rz-can-err-g", ndev);
	if (err)
		goto exit_err_irq_g;

	err = rz_can_start(ndev);
	if (err)
		goto exit_err_irq_g;

	netif_start_queue(ndev);

	return 0;

exit_err_irq_g:
	free_irq(priv->err_irq_m, ndev);
exit_err_irq_m:
	free_irq(priv->tx_irq, ndev);
exit_tx_irq:
	free_irq(priv->rx_irq, ndev);
exit_rx_irq:
	close_candev(ndev);
exit_open:
	return err;
}

static void rz_can_stop(struct net_device *ndev)
{
	struct rz_can_priv *priv = netdev_priv(ndev);
	u32 reg;

	/* Transition to global stop mode */
	reg = rz_can_read(priv, RZ_CAN_RSCAN0GCTR);
	reg &= ~RZ_CAN_RSCAN0GCTR_GMDC_M;
	reg |= RZ_CAN_RSCAN0GCTR_GMDC(RST_MODE);
	rz_can_write(priv, RZ_CAN_RSCAN0GCTR, reg);

	/* Transition to channel comm mode */
	reg = rz_can_read(priv, RZ_CAN_RSCAN0CmCTR(priv->m));
	reg &= ~RZ_CAN_RSCAN0CmCTR_CHMDC_M;
	reg |= RZ_CAN_RSCAN0CmCTR_CHMDC(RST_MODE);
	rz_can_write(priv, RZ_CAN_RSCAN0CmCTR(priv->m), reg);

	/* Go to stop mode */
	reg = rz_can_read(priv, RZ_CAN_RSCAN0GCTR);
	reg |= RZ_CAN_RSCAN0GCTR_GSLPR;
	rz_can_write(priv, RZ_CAN_RSCAN0GCTR, reg);

	/* From channel reset mode to channel stop mode */
	reg = rz_can_read(priv, RZ_CAN_RSCAN0CmCTR(priv->m));
	reg |= RZ_CAN_RSCAN0CmCTR_CSLPR;
	rz_can_write(priv, RZ_CAN_RSCAN0CmCTR(priv->m), reg);
}

static int rz_can_close(struct net_device *ndev)
{
	struct rz_can_priv *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);
	rz_can_stop(ndev);
	free_irq(priv->tx_irq, ndev);
	free_irq(priv->rx_irq, ndev);
	free_irq(priv->err_irq_m, ndev);
	free_irq(priv->err_irq_g, ndev);
	clk_disable(priv->clk);
	close_candev(ndev);

	return 0;
}

static int rz_can_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		rz_can_start(ndev);
		if (netif_queue_stopped(ndev))
			netif_wake_queue(ndev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct net_device_ops rz_can_netdev_ops = {
	.ndo_open               = rz_can_open,
	.ndo_stop               = rz_can_close,
	.ndo_start_xmit         = rz_can_start_xmit,
};

static int rz_can_probe(struct platform_device *pdev)
{
	struct rz_can_platform_data *pdata;
	struct rz_can_priv *priv;
	struct net_device *ndev;
	struct resource *mem;
	int err = -ENODEV;
	int rx_irq, tx_irq, err_irq_m, err_irq_g;

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data provided!\n");
		goto fail;
	}

#ifdef CONFIG_MACH_RSKRZA1
//asdf	rskrza1_board_can_pfc_assign(pdata->channel);
#endif

#ifdef CONFIG_MACH_HACHIKO
	 hachiko_board_can_pfc_assign(pdata->channel);
#endif

	rx_irq = platform_get_irq(pdev, 0);
	tx_irq = platform_get_irq(pdev, 1);
	err_irq_m = platform_get_irq(pdev, 2);
	err_irq_g = platform_get_irq(pdev, 3);
	if (!rx_irq || !tx_irq || !err_irq_m || !err_irq_g) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		goto fail;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "No mem resource\n");
		goto fail;
	}

	if (!request_mem_region(mem->start, resource_size(mem),
				dev_name(&pdev->dev))) {
		err = -EBUSY;
		goto fail;
	}

	ndev = alloc_candev(sizeof(struct rz_can_priv), RZ_CAN_TX_ECHO_SKB_MAX);
	if (!ndev) {
		dev_err(&pdev->dev, "alloc_candev failed\n");
		err = -ENOMEM;
		goto fail;
	}

	priv = netdev_priv(ndev);

	priv->clk = devm_clk_get(&pdev->dev, "can");
	if (IS_ERR(priv->clk)) {
		err = PTR_ERR(priv->clk);
		dev_err(&pdev->dev, "cannot get clock: %d\n", err);
		goto fail_clk;
	}

	ndev->netdev_ops = &rz_can_netdev_ops;
	ndev->flags |= IFF_ECHO;
	priv->ndev = ndev;
	priv->base = (void __iomem *) mem->start;
	priv->clock_select = pdata->clock_select;
	priv->m = pdata->channel;
	priv->tx_irq = tx_irq;
	priv->rx_irq = rx_irq;
	priv->err_irq_m = err_irq_m;
	priv->err_irq_g = err_irq_g;
	priv->k_tx = RZ_CAN_FIFO_K(priv->m, RZ_CAN_TX_FIFO);
	priv->k_rx = RZ_CAN_FIFO_K(priv->m, RZ_CAN_RX_FIFO);

	priv->can.clock.freq = (clk_get_rate(priv->clk) / 2);
	priv->can.bittiming_const = &rz_can_bittiming_const;
	priv->can.do_set_mode = rz_can_do_set_mode;
	priv->can.do_get_berr_counter = rz_can_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_BERR_REPORTING;

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);
	spin_lock_init(&priv->skb_lock);

	err = register_candev(ndev);
	if (err) {
		dev_err(&pdev->dev, "register_candev() failed\n");
		goto fail_clk;
	}

	dev_info(&pdev->dev, "device registered (clock: %d, ch: %d, k_tx: %d, k_rx: %d)\n",
			priv->can.clock.freq, priv->m, priv->k_tx, priv->k_rx);

	return 0;

fail_clk:
	free_candev(ndev);
fail:
	return err;
}

static int rz_can_remove(struct platform_device *pdev)
{
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);

	unregister_candev(ndev);
	free_candev(ndev);

	return 0;
}

static struct platform_driver rz_can_driver = {
	.probe = rz_can_probe,
	.remove = rz_can_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
};

module_platform_driver(rz_can_driver);

MODULE_AUTHOR("Carlo Caione <carlo@caione.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RZ/A1 on-chip CAN netdevice driver");
MODULE_ALIAS("platform:" DRV_NAME);

