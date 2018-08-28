/* SPDX-License-Identifier: GPL-2.0 OR MIT */

/*
 * Xen para-virtual camera device
 *
 * Copyright (C) 2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_CAMERA_FRONT_H
#define __XEN_CAMERA_FRONT_H

struct xen_camera_front_info {
	struct xenbus_device *xb_dev;
};

#endif /* __XEN_CAMERA_FRONT_H */
