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
 *			xlnx,can-clk-freq-hz = <0x5f5e100>;
 *		} ;
 *
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/kernel.h>
#include <linux/netdevice.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>

#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/prom.h>

#include "can-xilinx-ps7.h"

static int __devexit xcanps_remove(struct platform_device *pdev)
{
	struct net_device *dev = dev_get_drvdata(&pdev->dev);
	struct xcanps_priv *priv = netdev_priv(dev);
	struct device_node *np = pdev->dev.of_node;
	struct resource res;

	dev_set_drvdata(&pdev->dev, NULL);
	unregister_xcanpsdev(dev);
	free_xcanpsdev(dev);
	iounmap(priv->CanConfig.BaseAddr);
	irq_dispose_mapping(dev->irq);

	of_address_to_resource(np, 0, &res);
	release_mem_region(res.start, resource_size(&res));

	return 0;
}

static int __devinit xcanps_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct net_device *dev;
	struct xcanps_priv *priv;
	struct resource res;
	const u32 *prop;
	int err, irq, res_size;
	u32 clk;
	void __iomem *base;

	err = of_address_to_resource(np, 0, &res);
	if (err) {
		dev_err(&pdev->dev, "invalid address\n");
		return err;
	}

	res_size = resource_size(&res);

	if (!request_mem_region(res.start, res_size, pdev->name)) {
		dev_err(&pdev->dev, "couldn't request %pR\n", &res);
		return -EBUSY;
	}

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

	dev = alloc_xcanpsdev(0);
	if (!dev) {
		err = -ENOMEM;
		goto exit_dispose_irq;
	}

	prop = of_get_property(np, "xlnx,can-clk-freq-hz", NULL);
	if (prop)
		clk = be32_to_cpup(prop);
	else {
		dev_err(&pdev->dev, "couldn't determine input-clk\n");
		goto exit_dispose_irq ;
	}

	dev->irq = irq;

	priv = netdev_priv(dev);
	priv->irq_flags = IRQF_SHARED;
	priv->CanConfig.BaseAddr = base;
	priv->can.clock.freq = clk;


	dev_set_drvdata(&pdev->dev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	err = register_xcanpsdev(dev);
	if (err) {
		dev_err(&pdev->dev, "registering %s failed (err=%d)\n",
			DRIVER_NAME, err);
		goto exit_free_xcanps;
	}

	dev_info(&pdev->dev,
		 "reg_base=0x%p irq=%d clock=%d\n",
		 priv->CanConfig.BaseAddr, dev->irq, priv->can.clock.freq);

	return 0;

exit_free_xcanps:
	free_xcanpsdev(dev);
exit_dispose_irq:
	irq_dispose_mapping(irq);
exit_unmap_mem:
	iounmap(base);
exit_release_mem:
	release_mem_region(res.start, res_size);
	return err;
}


static struct of_device_id xcanps_of_match[] __devinitdata = {
	{ .compatible = "xlnx,ps7-can-1.00.a", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, xcanps_of_match);

static struct platform_driver xcanps_drv = {
	.driver = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = xcanps_of_match,
	},
	.probe  = xcanps_probe,
	.remove = __devexit_p(xcanps_remove),
};

/**
 * xcanps_init - Initial driver registration function
 *
 * Returns zero on success, otherwise negative error.
 */
static int __init xcanps_init(void)
{
	return platform_driver_register(&xcanps_drv);
}

/**
 * xcanps_exit - Driver Un-registration function
 */
static void __exit xcanps_exit(void)
{
	platform_driver_unregister(&xcanps_drv);
}

module_init(xcanps_init);
module_exit(xcanps_exit);

