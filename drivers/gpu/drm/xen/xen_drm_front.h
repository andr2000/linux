/*
 *  Xen para-virtual DRM device
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
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_DRM_FRONT_H_
#define __XEN_DRM_FRONT_H_

#include "xen_drm_front_cfg.h"

#ifndef GRANT_INVALID_REF
/*
 * Note on usage of grant reference 0 as invalid grant reference:
 * grant reference 0 is valid, but never exposed to a PV driver,
 * because of the fact it is already in use/reserved by the PV console.
 */
#define GRANT_INVALID_REF	0
#endif

struct xen_drm_front_ops {
	/* CAUTION! this is called with a spin_lock held! */
	void (*on_frame_done)(struct platform_device *pdev,
			int conn_idx, uint64_t fb_cookie);
};

struct xen_drm_front_info {
	struct xenbus_device *xb_dev;
	/* to protect data between backend IO code and interrupt handler */
	spinlock_t io_lock;
	/* virtual DRM platform device */
	struct platform_device *drm_pdev;

	int num_evt_pairs;
	struct xen_drm_front_evtchnl_pair *evt_pairs;
	struct xen_drm_front_cfg cfg;
};

#endif /* __XEN_DRM_FRONT_H_ */
