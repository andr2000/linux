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
 * Copyright (C) 2016-2017 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_FRONT_H
#define __XEN_FRONT_H

#include "xen_front_cfg.h"

struct xen_front_evtchnl_pair_info;

struct drv_info {
	struct xenbus_device *xb_dev;
	spinlock_t io_lock;
	struct mutex mutex;
	bool snd_drv_registered;
	struct platform_device *snd_drv_pdev;
	int num_evt_pairs;
	struct xen_front_evtchnl_pair_info *evt_pairs;
	struct xen_front_cfg_card_plat_data cfg_plat_data;
};

#endif /* __XEN_FRONT_H */
