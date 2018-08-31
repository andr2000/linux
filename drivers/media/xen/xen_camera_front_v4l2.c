// SPDX-License-Identifier: GPL-2.0 OR MIT

/*
 * Xen para-virtual camera device
 *
 * Based on V4L2 PCI Skeleton Driver: samples/v4l/v4l2-pci-skeleton.c
 *
 * Copyright (C) 2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#include <xen/xenbus.h>

#include "xen_camera_front.h"
#include "xen_camera_front_v4l2.h"

int xen_camera_front_v4l2_init(struct xen_camera_front_info *front_info)
{
	struct device *dev = &front_info->xb_dev->dev;
	struct xen_camera_front_v4l2_info *v4l2_info;
	int ret;

	v4l2_info = devm_kzalloc(dev, sizeof(*v4l2_info), GFP_KERNEL);
	if (!v4l2_info)
		return -ENOMEM;

	v4l2_info->front_info = front_info;
	front_info->v4l2_info = v4l2_info;

	ret = v4l2_device_register(dev, &v4l2_info->v4l2_dev);
	if (ret < 0)
		return ret;

	return 0;
}

void xen_camera_front_v4l2_fini(struct xen_camera_front_info *front_info)
{
	if (!front_info->v4l2_info)
		return;

	v4l2_device_unregister(&front_info->v4l2_info->v4l2_dev);
}

