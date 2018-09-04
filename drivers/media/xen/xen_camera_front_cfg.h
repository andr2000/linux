/* SPDX-License-Identifier: GPL-2.0 OR MIT */

/*
 * Xen para-virtual camera device
 *
 * Copyright (C) 2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_CAMERA_FRONT_CFG_H
#define __XEN_CAMERA_FRONT_CFG_H

#include <xen/interface/io/cameraif.h>

struct xen_camera_front_info;

struct xen_camera_front_cfg_ctrl {
	u32 v4l2_cid;

	s32 minimum;
	s32 maximum;
	s32 default_value;
	s32 step;
};

struct xen_camera_front_cfg_card {
	int num_controls;
	struct xen_camera_front_cfg_ctrl ctrl[XENCAMERA_MAX_CTRL];
};

int xen_camera_front_cfg_to_v4l2_cid(int xen_type);

#endif /* __XEN_CAMERA_FRONT_CFG_H */
