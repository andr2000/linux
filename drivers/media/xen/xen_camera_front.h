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

#include "xen_camera_front_evtchnl.h"

struct xen_camera_front_info {
	struct xenbus_device *xb_dev;

	struct xen_camera_front_evtchnl_pair evt_pair;

	/* to protect data between backend IO code and interrupt handler */
	spinlock_t io_lock;
};

#endif /* __XEN_CAMERA_FRONT_H */
