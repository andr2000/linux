/*
 * Xen para-virtual sound device
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 * Based on: sound/drivers/dummy.c
 *
 * Copyright (C) 2016-2017 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#include <linux/module.h>

#include <xen/platform_pci.h>
#include <xen/xen.h>
#include <xen/xenbus.h>

#include <xen/interface/io/sndif.h>

#include "xen_front.h"

static void xenbus_drv_remove_internal(struct drv_info *drv_info)
{
}

static void xenbus_drv_be_on_changed(struct xenbus_device *xb_dev,
	enum xenbus_state backend_state)
{
}

static int xenbus_drv_probe(struct xenbus_device *xb_dev,
	const struct xenbus_device_id *id)
{
	struct drv_info *drv_info;

	drv_info = devm_kzalloc(&xb_dev->dev, sizeof(*drv_info), GFP_KERNEL);
	if (!drv_info) {
		xenbus_dev_fatal(xb_dev, -ENOMEM, "allocating device memory");
		return -ENOMEM;
	}

	drv_info->xb_dev = xb_dev;
	spin_lock_init(&drv_info->io_lock);
	mutex_init(&drv_info->mutex);
	dev_set_drvdata(&xb_dev->dev, drv_info);
	return 0;
}

static int xenbus_drv_remove(struct xenbus_device *dev)
{
	struct drv_info *drv_info = dev_get_drvdata(&dev->dev);

	mutex_lock(&drv_info->mutex);
	xenbus_drv_remove_internal(drv_info);
	mutex_unlock(&drv_info->mutex);
	return 0;
}

static const struct xenbus_device_id xenbus_drv_ids[] = {
	{ XENSND_DRIVER_NAME },
	{ "" }
};

static struct xenbus_driver xenbus_driver = {
	.ids = xenbus_drv_ids,
	.probe = xenbus_drv_probe,
	.remove = xenbus_drv_remove,
	.otherend_changed = xenbus_drv_be_on_changed,
};

static int __init xenbus_drv_init(void)
{
	if (!xen_domain())
		return -ENODEV;

	pr_info("Initialising Xen " XENSND_DRIVER_NAME " frontend driver\n");
	return xenbus_register_frontend(&xenbus_driver);
}

static void __exit xenbus_drv_cleanup(void)
{
	pr_info("Unregistering Xen " XENSND_DRIVER_NAME " frontend driver\n");
	xenbus_unregister_driver(&xenbus_driver);
}

module_init(xenbus_drv_init);
module_exit(xenbus_drv_cleanup);

MODULE_DESCRIPTION("Xen virtual sound device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:"XENSND_DRIVER_NAME);
MODULE_SUPPORTED_DEVICE("{{ALSA,Virtual soundcard}}");
