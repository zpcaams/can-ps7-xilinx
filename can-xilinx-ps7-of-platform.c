/*
 * can-xilinx-ps7-of-platform.c
 *
 * This is a generic driver for CAN PS7 on the Zynq platform for Zedboard.
 * You need a ps7_can node definition in your flattened device tree
 * source (DTS) file similar to:
 *
 *		ps7_can_0: ps7-can@e0008000 {
 *			compatible = "xlnx,ps7-can-1.00.a";
 *			interrupts = < 0 28 4 >;
 *			reg = < 0xe0008000 0x1000 >;
 *		xlnx,can-clk-freq-hz = <0x5f5e100>;
 *		} ;
 *
 */

#include <linux/export.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/xilinx_devices.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_i2c.h>

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/can/dev.h>

#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/prom.h>

#include "sja1000.h"

#define DRIVER_NAME		"xcanps"

#define SJA1000_OFP_CAN_CLOCK  (16000000 / 2)
#define SJA1000_OFP_OCR        OCR_TX0_PULLDOWN
#define SJA1000_OFP_CDR        (CDR_CBP | CDR_CLK_OFF)


static u32 sja1000_ofp_read_reg(const struct sja1000_priv *priv, int reg)
{
	return ioread32(priv->reg_base + reg);
}

static void sja1000_ofp_write_reg(const struct sja1000_priv *priv,
				  int reg, u32 val)
{
	iowrite32(val, (priv->reg_base + reg));
}

/************************/
/* Platform bus binding */
/************************/

/**
 * xi2cps_probe - Platform registration call
 * @pdev:	Handle to the platform device structure
 *
 * Returns zero on success, negative error otherwise
 *
 * This function does all the memory allocation and registration for the i2c
 * device. User can modify the address mode to 10 bit address mode using the
 * ioctl call with option I2C_TENBIT.
 */
static int __devinit xi2cps_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct net_device *dev;
	struct sja1000_priv *priv;
	struct resource res;
	const u32 *prop;
	int err, irq, res_size;
	void __iomem *base;

	printk("can ps7 probe\n");

	err = of_address_to_resource(np, 0, &res);
	if (err) {
		dev_err(&pdev->dev, "invalid address\n");
		return err;
	}

	res_size = resource_size(&res);

	base = ioremap(res.start, res_size);
	if (!base) {
		dev_err(&pdev->dev, "couldn't ioremap %pR\n", &res);
		err = -ENOMEM;
		goto exit_release_mem;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (irq == NO_IRQ) {
		dev_err(&pdev->dev, "no irq found\n");
		err = -ENODEV;
		goto exit_unmap_mem;
	}

	dev = alloc_sja1000dev(0);
	if (!dev) {
		err = -ENOMEM;
		goto exit_dispose_irq;
	}

	priv = netdev_priv(dev);

	priv->read_reg = sja1000_ofp_read_reg;
	priv->write_reg = sja1000_ofp_write_reg;

	prop = of_get_property(np, "xlnx,can-clk-freq-hz", NULL);
	if (prop)
		priv->can.clock.freq = be32_to_cpup(prop);
	else {
		dev_err(&pdev->dev, "couldn't determine input-clk\n");
		goto exit_dispose_irq ;
	}

	priv->ocr |= OCR_TX0_PULLDOWN; /* default */
	priv->cdr |= CDR_CLK_OFF; /* default */

	priv->irq_flags = IRQF_SHARED;
	priv->reg_base = base;

	dev->irq = irq;

	dev_info(&pdev->dev,
		 "reg_base=0x%p irq=%d \n clock=%d ocr=0x%02x cdr=0x%02x\n",
		 priv->reg_base, dev->irq, priv->can.clock.freq,
		 priv->ocr, priv->cdr);

	dev_set_drvdata(&pdev->dev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

//	err = register_sja1000dev(dev);
//	if (err) {
//		dev_err(&ofdev->dev, "registering %s failed (err=%d)\n",
//			DRV_NAME, err);
//		goto exit_free_sja1000;
//	}

	return 0;

exit_free_sja1000:
	free_sja1000dev(dev);
exit_dispose_irq:
	irq_dispose_mapping(irq);
exit_unmap_mem:
	iounmap(base);
exit_release_mem:
	release_mem_region(res.start, res_size);

	return err;
}

/**
 * xi2cps_remove - Unregister the device after releasing the resources
 * @pdev:	Handle to the platform device structure
 *
 * Returns zero always
 *
 * This function frees all the resources allocated to the device.
 */
static int __devexit xi2cps_remove(struct platform_device *pdev)
{
	struct net_device *dev = dev_get_drvdata(&pdev->dev);
	struct sja1000_priv *priv = netdev_priv(dev);
	struct device_node *np = pdev->dev.of_node;
	struct resource res;

	printk("can ps7 remove\n");

	dev_set_drvdata(&pdev->dev, NULL);

	//unregister_sja1000dev(dev);
	free_sja1000dev(dev);
	iounmap(priv->reg_base);
	irq_dispose_mapping(dev->irq);

	of_address_to_resource(np, 0, &res);
	//release_mem_region(res.start, resource_size(&res));

	return 0;
}

static struct of_device_id xi2cps_of_match[] __devinitdata = {
	{ .compatible = "xlnx,ps7-can-1.00.a", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, xi2cps_of_match);

static struct platform_driver xi2cps_drv = {
	.driver = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = xi2cps_of_match,
	},
	.probe  = xi2cps_probe,
	.remove = __devexit_p(xi2cps_remove),
};

/**
 * xi2cps_init - Initial driver registration function
 *
 * Returns zero on success, otherwise negative error.
 */
static int __init xi2cps_init(void)
{
	return platform_driver_register(&xi2cps_drv);
}

/**
 * xi2cps_exit - Driver Un-registration function
 */
static void __exit xi2cps_exit(void)
{
	platform_driver_unregister(&xi2cps_drv);
}

module_init(xi2cps_init);
module_exit(xi2cps_exit);

MODULE_AUTHOR("ZhouPeng<zp_caams@163.com>");
MODULE_DESCRIPTION("Xilinx PS CAN bus driver");
MODULE_LICENSE("GPL");
