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

/**
 * xi2cps_init - Initial driver registration function
 *
 * Returns zero on success, otherwise negative error.
 */
static int __init xi2cps_init(void)
{
	//return platform_driver_register(&xi2cps_drv);
	printk("hello\n");
	return 0;
}

/**
 * xi2cps_exit - Driver Un-registration function
 */
static void __exit xi2cps_exit(void)
{
	//platform_driver_unregister(&xi2cps_drv);
	return;
}

module_init(xi2cps_init);
module_exit(xi2cps_exit);

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION("Xilinx PS I2C bus driver");
MODULE_LICENSE("GPL");
