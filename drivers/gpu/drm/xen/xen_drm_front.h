/* SPDX-License-Identifier: GPL-2.0 OR MIT */

/*
 *  Xen para-virtual DRM device
 *
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_DRM_FRONT_H_
#define __XEN_DRM_FRONT_H_

#include "xen_drm_front_cfg.h"

struct xen_drm_front_info {
	struct xenbus_device *xb_dev;
	struct xen_drm_front_cfg cfg;
};

#endif /* __XEN_DRM_FRONT_H_ */
