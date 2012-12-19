/*
 * can-xilinx-ps7-of-platform.c
 *
 *  Created on: Dec 19, 2012
 *      Author: root
 */
/*
 * Xilinx I2C bus driver for the PS I2C Interfaces.
 *
 * 2009-2011 (c) Xilinx, Inc.
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any
 * later version.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA
 * 02139, USA.
 *
 *
 * Workaround in Receive Mode
 *	If there is only one message to be processed, then based on length of
 *	the message we set the HOLD bit.
 *	If the length is less than the FIFO depth, then we will directly
 *	receive a COMP interrupt and the transaction is done.
 *	If the length is more than the FIFO depth, then we enable the HOLD bit
 *	and write FIFO depth to the transfer size register.
 *	We will receive the DATA interrupt, we calculate the remaining bytes
 *	to receive and write to the transfer size register and we process the
 *	data in FIFO.
 *	In the meantime, we are receiving the complete interrupt also and the
 *	controller waits for the default timeout period before generating a stop
 *	condition even though the HOLD bit is set. So we are unable to generate
 *	the data interrupt again.
 *	To avoid this, we wrote the expected bytes to receive as FIFO depth + 1
 *	instead of FIFO depth. This generated the second DATA interrupt as there
 *	are still outstanding bytes to be received.
 *
 *	The bus hold flag logic provides support for repeated start.
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

#define DRIVER_NAME		"xcanps"

/**
 * struct xi2cps - I2C device private data structure
 * @membase:		Base address of the I2C device
 * @adap:		I2C adapter instance
 * @p_msg:		Message pointer
 * @err_status:		Error status in Interrupt Status Register
 * @xfer_done:		Transfer complete status
 * @p_send_buf:		Pointer to transmit buffer
 * @p_recv_buf:		Pointer to receive buffer
 * @send_count:		Number of bytes still expected to send
 * @recv_count:		Number of bytes still expected to receive
 * @irq:		IRQ number
 * @cur_timeout:	The current timeout value used by the device
 * @input_clk:		Input clock to I2C controller
 * @bus_hold_flag:	Flag used in repeated start for clearing HOLD bit
 */
struct xi2cps {
	void __iomem *membase;
	struct i2c_adapter adap;
	struct i2c_msg	*p_msg;
	int err_status;
	struct completion xfer_done;
	unsigned char *p_send_buf;
	unsigned char *p_recv_buf;
	int send_count;
	int recv_count;
	int irq;
	int cur_timeout;
	unsigned int input_clk;
	unsigned int bus_hold_flag;
};

/**
 * xi2cps_isr - Interrupt handler for the I2C device
 * @irq:	irq number for the I2C device
 * @ptr:	void pointer to xi2cps structure
 *
 * Returns IRQ_HANDLED always
 *
 * This function handles the data interrupt, transfer complete interrupt and
 * the error interrupts of the I2C device.
 */
static irqreturn_t xi2cps_isr(int irq, void *ptr)
{
	return 0;
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
	struct resource *r_mem = NULL;
	struct xi2cps *id;
	int ret;
	const unsigned int *prop;

	printk("xcanps7 probe!\n");
	/*
	 * Allocate memory for xi2cps structure.
	 * Initialize the structure to zero and set the platform data.
	 * Obtain the resource base address from platform data and remap it.
	 * Get the irq resource from platform data.Initialize the adapter
	 * structure members and also xi2cps structure.
	 */
	id = kzalloc(sizeof(struct xi2cps), GFP_KERNEL);
	if (!id) {
		dev_err(&pdev->dev, "no mem for i2c private data\n");
		return -ENOMEM;
	}
	memset((void *)id, 0, sizeof(struct xi2cps));
	platform_set_drvdata(pdev, id);

	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		dev_err(&pdev->dev, "no mmio resources\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	id->membase = ioremap(r_mem->start, r_mem->end - r_mem->start + 1);
	if (id->membase == NULL) {
		dev_err(&pdev->dev, "Couldn't ioremap memory at 0x%08lx\n",
			(unsigned long)r_mem->start);
		ret = -ENOMEM;
		goto err_free_mem;
	}

	id->irq = platform_get_irq(pdev, 0);
	if (id->irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource:%d\n", id->irq);
		ret = -ENXIO;
		goto err_unmap;
	}

	prop = of_get_property(pdev->dev.of_node, "xlnx,can-clk-freq-hz", NULL);
	if (prop)
		id->input_clk = be32_to_cpup(prop);
	else {
		ret = -ENXIO;
		dev_err(&pdev->dev, "couldn't determine input-clk\n");
		goto err_unmap ;
	}

	if (request_irq(id->irq, xi2cps_isr, 0, DRIVER_NAME, id)) {
		dev_err(&pdev->dev, "cannot get irq %d\n", id->irq);
		ret = -EINVAL;
		goto err_unmap;
	}

	printk("mem %x addressed to %x, irq %d requested\n input clock is %d!\n"
				, r_mem->start, (u32)id->membase, id->irq, id->input_clk);
	return 0;

err_free_irq:
	free_irq(id->irq, id);
err_unmap:
	iounmap(id->membase);
err_free_mem:
	return ret;
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
	struct xi2cps *id = platform_get_drvdata(pdev);

	printk("xcanps7 remorve!\n");

	free_irq(id->irq, id);
	iounmap(id->membase);
	kfree(id);
	platform_set_drvdata(pdev, NULL);
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
