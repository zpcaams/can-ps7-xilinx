/*
 * can-xilinx-ps7.c
 *
 *  Created on: Dec 19, 2012
 *      Author: root
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/skbuff.h>
#include <linux/delay.h>

#include <linux/can/dev.h>
#include <linux/can/error.h>

#include "can-xilinx-ps7.h"

#define DRV_NAME "xcanps"

MODULE_AUTHOR("Oliver Hartkopp <oliver.hartkopp@volkswagen.de>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION(DRV_NAME "CAN netdevice driver");

static struct can_bittiming_const xcanps_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,
};

static void xcanps_write_cmdreg(struct xcanps_priv *priv, u8 val)
{
//	unsigned long flags;
//
//	/*
//	 * The command register needs some locking and time to settle
//	 * the write_reg() operation - especially on SMP systems.
//	 */
//	spin_lock_irqsave(&priv->cmdreg_lock, flags);
//	priv->write_reg(priv, REG_CMR, val);
//	priv->read_reg(priv, REG_SR);
//	spin_unlock_irqrestore(&priv->cmdreg_lock, flags);
	printk("can write cmd reg\n");
}

static void set_normal_mode(struct net_device *dev)
{
//	struct xcanps_priv *priv = netdev_priv(dev);
//	unsigned char status = priv->read_reg(priv, REG_MOD);
//	int i;
//
//	for (i = 0; i < 100; i++) {
//		/* check reset bit */
//		if ((status & MOD_RM) == 0) {
//			priv->can.state = CAN_STATE_ERROR_ACTIVE;
//			/* enable interrupts */
//			if (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING)
//				priv->write_reg(priv, REG_IER, IRQ_ALL);
//			else
//				priv->write_reg(priv, REG_IER,
//						IRQ_ALL & ~IRQ_BEI);
//			return;
//		}
//
//		/* set chip to normal mode */
//		priv->write_reg(priv, REG_MOD, 0x00);
//		udelay(10);
//		status = priv->read_reg(priv, REG_MOD);
//	}
//
//	dev_err(dev->dev.parent, "setting SJA1000 into normal mode failed!\n");
	printk("can set nromal mode\n");
}

static void xcanps_start(struct net_device *dev)
{
//	struct xcanps_priv *priv = netdev_priv(dev);
//
//	/* leave reset mode */
//	if (priv->can.state != CAN_STATE_STOPPED)
//		set_reset_mode(dev);
//
//	/* Clear error counters and error code capture */
//	priv->write_reg(priv, REG_TXERR, 0x0);
//	priv->write_reg(priv, REG_RXERR, 0x0);
//	priv->read_reg(priv, REG_ECC);
//
//	/* leave reset mode */
//	set_normal_mode(dev);
	printk("can start\n");
}

static int xcanps_set_mode(struct net_device *dev, enum can_mode mode)
{
//	struct xcanps_priv *priv = netdev_priv(dev);
//
//	if (!priv->open_time)
//		return -EINVAL;
//
//	switch (mode) {
//	case CAN_MODE_START:
//		xcanps_start(dev);
//		if (netif_queue_stopped(dev))
//			netif_wake_queue(dev);
//		break;
//
//	default:
//		return -EOPNOTSUPP;
//	}
	printk("can set mode\n");
	return 0;
}

static int xcanps_set_bittiming(struct net_device *dev)
{
//	struct xcanps_priv *priv = netdev_priv(dev);
//	struct can_bittiming *bt = &priv->can.bittiming;
//	u8 btr0, btr1;
//
//	btr0 = ((bt->brp - 1) & 0x3f) | (((bt->sjw - 1) & 0x3) << 6);
//	btr1 = ((bt->prop_seg + bt->phase_seg1 - 1) & 0xf) |
//		(((bt->phase_seg2 - 1) & 0x7) << 4);
//	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
//		btr1 |= 0x80;
//
//	dev_info(dev->dev.parent,
//		 "setting BTR0=0x%02x BTR1=0x%02x\n", btr0, btr1);
//
//	priv->write_reg(priv, REG_BTR0, btr0);
//	priv->write_reg(priv, REG_BTR1, btr1);
	printk("can set bittiming\n");
	return 0;
}

static int xcanps_get_berr_counter(const struct net_device *dev,
				    struct can_berr_counter *bec)
{
//	struct xcanps_priv *priv = netdev_priv(dev);
//
//	bec->txerr = priv->read_reg(priv, REG_TXERR);
//	bec->rxerr = priv->read_reg(priv, REG_RXERR);
	printk("can get bit counter\n");
	return 0;
}

/*
 * transmit a CAN message
 * message layout in the sk_buff should be like this:
 * xx xx xx xx	 ff	 ll   00 11 22 33 44 55 66 77
 * [  can-id ] [flags] [len] [can data (up to 8 bytes]
 */
static netdev_tx_t xcanps_start_xmit(struct sk_buff *skb,
					    struct net_device *dev)
{
//	struct xcanps_priv *priv = netdev_priv(dev);
//	struct can_frame *cf = (struct can_frame *)skb->data;
//	uint8_t fi;
//	uint8_t dlc;
//	canid_t id;
//	uint8_t dreg;
//	int i;
//
//	if (can_dropped_invalid_skb(dev, skb))
//		return NETDEV_TX_OK;
//
//	netif_stop_queue(dev);
//
//	fi = dlc = cf->can_dlc;
//	id = cf->can_id;
//
//	if (id & CAN_RTR_FLAG)
//		fi |= FI_RTR;
//
//	if (id & CAN_EFF_FLAG) {
//		fi |= FI_FF;
//		dreg = EFF_BUF;
//		priv->write_reg(priv, REG_FI, fi);
//		priv->write_reg(priv, REG_ID1, (id & 0x1fe00000) >> (5 + 16));
//		priv->write_reg(priv, REG_ID2, (id & 0x001fe000) >> (5 + 8));
//		priv->write_reg(priv, REG_ID3, (id & 0x00001fe0) >> 5);
//		priv->write_reg(priv, REG_ID4, (id & 0x0000001f) << 3);
//	} else {
//		dreg = SFF_BUF;
//		priv->write_reg(priv, REG_FI, fi);
//		priv->write_reg(priv, REG_ID1, (id & 0x000007f8) >> 3);
//		priv->write_reg(priv, REG_ID2, (id & 0x00000007) << 5);
//	}
//
//	for (i = 0; i < dlc; i++)
//		priv->write_reg(priv, dreg++, cf->data[i]);
//
//	can_put_echo_skb(skb, dev, 0);
//
//	xcanps_write_cmdreg(priv, CMD_TR);
	printk("can tx\n");
	return NETDEV_TX_OK;
}

static void xcanps_rx(struct net_device *dev)
{
//	struct xcanps_priv *priv = netdev_priv(dev);
//	struct net_device_stats *stats = &dev->stats;
//	struct can_frame *cf;
//	struct sk_buff *skb;
//	uint8_t fi;
//	uint8_t dreg;
//	canid_t id;
//	int i;
//
//	/* create zero'ed CAN frame buffer */
//	skb = alloc_can_skb(dev, &cf);
//	if (skb == NULL)
//		return;
//
//	fi = priv->read_reg(priv, REG_FI);
//
//	if (fi & FI_FF) {
//		/* extended frame format (EFF) */
//		dreg = EFF_BUF;
//		id = (priv->read_reg(priv, REG_ID1) << (5 + 16))
//		    | (priv->read_reg(priv, REG_ID2) << (5 + 8))
//		    | (priv->read_reg(priv, REG_ID3) << 5)
//		    | (priv->read_reg(priv, REG_ID4) >> 3);
//		id |= CAN_EFF_FLAG;
//	} else {
//		/* standard frame format (SFF) */
//		dreg = SFF_BUF;
//		id = (priv->read_reg(priv, REG_ID1) << 3)
//		    | (priv->read_reg(priv, REG_ID2) >> 5);
//	}
//
//	cf->can_dlc = get_can_dlc(fi & 0x0F);
//	if (fi & FI_RTR) {
//		id |= CAN_RTR_FLAG;
//	} else {
//		for (i = 0; i < cf->can_dlc; i++)
//			cf->data[i] = priv->read_reg(priv, dreg++);
//	}
//
//	cf->can_id = id;
//
//	/* release receive buffer */
//	xcanps_write_cmdreg(priv, CMD_RRB);
//
//	netif_rx(skb);
//
//	stats->rx_packets++;
//	stats->rx_bytes += cf->can_dlc;
	printk("can rx\n");
}

static int xcanps_err(struct net_device *dev, uint8_t isrc, uint8_t status)
{
//	struct xcanps_priv *priv = netdev_priv(dev);
//	struct net_device_stats *stats = &dev->stats;
//	struct can_frame *cf;
//	struct sk_buff *skb;
//	enum can_state state = priv->can.state;
//	uint8_t ecc, alc;
//
//	skb = alloc_can_err_skb(dev, &cf);
//	if (skb == NULL)
//		return -ENOMEM;
//
//	if (isrc & IRQ_DOI) {
//		/* data overrun interrupt */
//		dev_dbg(dev->dev.parent, "data overrun interrupt\n");
//		cf->can_id |= CAN_ERR_CRTL;
//		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
//		stats->rx_over_errors++;
//		stats->rx_errors++;
//		xcanps_write_cmdreg(priv, CMD_CDO);	/* clear bit */
//	}
//
//	if (isrc & IRQ_EI) {
//		/* error warning interrupt */
//		dev_dbg(dev->dev.parent, "error warning interrupt\n");
//
//		if (status & SR_BS) {
//			state = CAN_STATE_BUS_OFF;
//			cf->can_id |= CAN_ERR_BUSOFF;
//			can_bus_off(dev);
//		} else if (status & SR_ES) {
//			state = CAN_STATE_ERROR_WARNING;
//		} else
//			state = CAN_STATE_ERROR_ACTIVE;
//	}
//	if (isrc & IRQ_BEI) {
//		/* bus error interrupt */
//		priv->can.can_stats.bus_error++;
//		stats->rx_errors++;
//
//		ecc = priv->read_reg(priv, REG_ECC);
//
//		cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
//
//		switch (ecc & ECC_MASK) {
//		case ECC_BIT:
//			cf->data[2] |= CAN_ERR_PROT_BIT;
//			break;
//		case ECC_FORM:
//			cf->data[2] |= CAN_ERR_PROT_FORM;
//			break;
//		case ECC_STUFF:
//			cf->data[2] |= CAN_ERR_PROT_STUFF;
//			break;
//		default:
//			cf->data[2] |= CAN_ERR_PROT_UNSPEC;
//			cf->data[3] = ecc & ECC_SEG;
//			break;
//		}
//		/* Error occurred during transmission? */
//		if ((ecc & ECC_DIR) == 0)
//			cf->data[2] |= CAN_ERR_PROT_TX;
//	}
//	if (isrc & IRQ_EPI) {
//		/* error passive interrupt */
//		dev_dbg(dev->dev.parent, "error passive interrupt\n");
//		if (status & SR_ES)
//			state = CAN_STATE_ERROR_PASSIVE;
//		else
//			state = CAN_STATE_ERROR_ACTIVE;
//	}
//	if (isrc & IRQ_ALI) {
//		/* arbitration lost interrupt */
//		dev_dbg(dev->dev.parent, "arbitration lost interrupt\n");
//		alc = priv->read_reg(priv, REG_ALC);
//		priv->can.can_stats.arbitration_lost++;
//		stats->tx_errors++;
//		cf->can_id |= CAN_ERR_LOSTARB;
//		cf->data[0] = alc & 0x1f;
//	}
//
//	if (state != priv->can.state && (state == CAN_STATE_ERROR_WARNING ||
//					 state == CAN_STATE_ERROR_PASSIVE)) {
//		uint8_t rxerr = priv->read_reg(priv, REG_RXERR);
//		uint8_t txerr = priv->read_reg(priv, REG_TXERR);
//		cf->can_id |= CAN_ERR_CRTL;
//		if (state == CAN_STATE_ERROR_WARNING) {
//			priv->can.can_stats.error_warning++;
//			cf->data[1] = (txerr > rxerr) ?
//				CAN_ERR_CRTL_TX_WARNING :
//				CAN_ERR_CRTL_RX_WARNING;
//		} else {
//			priv->can.can_stats.error_passive++;
//			cf->data[1] = (txerr > rxerr) ?
//				CAN_ERR_CRTL_TX_PASSIVE :
//				CAN_ERR_CRTL_RX_PASSIVE;
//		}
//		cf->data[6] = txerr;
//		cf->data[7] = rxerr;
//	}
//
//	priv->can.state = state;
//
//	netif_rx(skb);
//
//	stats->rx_packets++;
//	stats->rx_bytes += cf->can_dlc;
	printk("can err\n");
	return 0;
}

irqreturn_t xcanps_interrupt(int irq, void *dev_id)
{
//	struct net_device *dev = (struct net_device *)dev_id;
//	struct xcanps_priv *priv = netdev_priv(dev);
//	struct net_device_stats *stats = &dev->stats;
//	uint8_t isrc, status;
//	int n = 0;
//
//	/* Shared interrupts and IRQ off? */
//	if (priv->read_reg(priv, REG_IER) == IRQ_OFF)
//		return IRQ_NONE;
//
//	if (priv->pre_irq)
//		priv->pre_irq(priv);
//
//	while ((isrc = priv->read_reg(priv, REG_IR)) && (n < SJA1000_MAX_IRQ)) {
//		n++;
//		status = priv->read_reg(priv, REG_SR);
//		/* check for absent controller due to hw unplug */
//		if (status == 0xFF && xcanps_is_absent(priv))
//			return IRQ_NONE;
//
//		if (isrc & IRQ_WUI)
//			dev_warn(dev->dev.parent, "wakeup interrupt\n");
//
//		if (isrc & IRQ_TI) {
//			/* transmission complete interrupt */
//			stats->tx_bytes += priv->read_reg(priv, REG_FI) & 0xf;
//			stats->tx_packets++;
//			can_get_echo_skb(dev, 0);
//			netif_wake_queue(dev);
//		}
//		if (isrc & IRQ_RI) {
//			/* receive interrupt */
//			while (status & SR_RBS) {
//				xcanps_rx(dev);
//				status = priv->read_reg(priv, REG_SR);
//				/* check for absent controller */
//				if (status == 0xFF && xcanps_is_absent(priv))
//					return IRQ_NONE;
//			}
//		}
//		if (isrc & (IRQ_DOI | IRQ_EI | IRQ_BEI | IRQ_EPI | IRQ_ALI)) {
//			/* error interrupt */
//			if (xcanps_err(dev, isrc, status))
//				break;
//		}
//	}
//
//	if (priv->post_irq)
//		priv->post_irq(priv);
//
//	if (n >= SJA1000_MAX_IRQ)
//		dev_dbg(dev->dev.parent, "%d messages handled in ISR", n);
//
//	return (n) ? IRQ_HANDLED : IRQ_NONE;
	printk("can interrupt\n");
	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(xcanps_interrupt);

static int xcanps_open(struct net_device *dev)
{
//	struct xcanps_priv *priv = netdev_priv(dev);
//	int err;
//
//	/* set chip into reset mode */
//	set_reset_mode(dev);
//
//	/* common open */
//	err = open_candev(dev);
//	if (err)
//		return err;
//
//	/* register interrupt handler, if not done by the device driver */
//	if (!(priv->flags & SJA1000_CUSTOM_IRQ_HANDLER)) {
//		err = request_irq(dev->irq, xcanps_interrupt, priv->irq_flags,
//				  dev->name, (void *)dev);
//		if (err) {
//			close_candev(dev);
//			return -EAGAIN;
//		}
//	}
//
//	/* init and start chi */
//	xcanps_start(dev);
//	priv->open_time = jiffies;
//
//	netif_start_queue(dev);
	printk("can open\n");
	return 0;
}

static int xcanps_close(struct net_device *dev)
{
//	struct xcanps_priv *priv = netdev_priv(dev);
//
//	netif_stop_queue(dev);
//	set_reset_mode(dev);
//
//	if (!(priv->flags & SJA1000_CUSTOM_IRQ_HANDLER))
//		free_irq(dev->irq, (void *)dev);
//
//	close_candev(dev);
//
//	priv->open_time = 0;
	printk("can close\n");
	return 0;
}

struct net_device *alloc_xcanpsdev(int sizeof_priv)
{
	struct net_device *dev;
	struct xcanps_priv *priv;

	dev = alloc_candev(sizeof(struct xcanps_priv) + sizeof_priv,
		SJA1000_ECHO_SKB_MAX);
	if (!dev)
		return NULL;

	priv = netdev_priv(dev);

	priv->dev = dev;
	priv->can.bittiming_const = &xcanps_bittiming_const;
	priv->can.do_set_bittiming = xcanps_set_bittiming;
	priv->can.do_set_mode = xcanps_set_mode;
	priv->can.do_get_berr_counter = xcanps_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES |
		CAN_CTRLMODE_BERR_REPORTING;

	spin_lock_init(&priv->cmdreg_lock);

	if (sizeof_priv)
		priv->priv = (void *)priv + sizeof(struct xcanps_priv);

	return dev;
}
EXPORT_SYMBOL_GPL(alloc_xcanpsdev);

void free_xcanpsdev(struct net_device *dev)
{
	free_candev(dev);
}
EXPORT_SYMBOL_GPL(free_xcanpsdev);

static const struct net_device_ops xcanps_netdev_ops = {
       .ndo_open               = xcanps_open,
       .ndo_stop               = xcanps_close,
       .ndo_start_xmit         = xcanps_start_xmit,
};

int register_xcanpsdev(struct net_device *dev)
{
    dev->type  = ARPHRD_CAN; /* the netdevice hardware type */
    dev->flags |= IFF_NOARP;  /* CAN has no arp */

    dev->mtu   = sizeof(struct can_frame);

	dev->flags |= IFF_ECHO;	/* we support local echo */
	dev->netdev_ops = &xcanps_netdev_ops;

	printk("can register_xcanpsdev\n");

	//run self test and reset can peripheral
	//set_reset_mode(dev);
	//chipset_init(dev);

	return register_candev(dev);
}
EXPORT_SYMBOL_GPL(register_xcanpsdev);

void unregister_xcanpsdev(struct net_device *dev)
{
	//reset the can peripheral
	//set_reset_mode(dev);
	printk("can unregister_xcanpsdev\n");

	unregister_candev(dev);
}
EXPORT_SYMBOL_GPL(unregister_xcanpsdev);

static __init int xcanps_init(void)
{
	printk(KERN_INFO "%s CAN PS7 netdevice driver\n", DRV_NAME);

	return 0;
}

module_init(xcanps_init);

static __exit void xcanps_exit(void)
{
	printk(KERN_INFO "%s: driver removed\n", DRV_NAME);
}

module_exit(xcanps_exit);
