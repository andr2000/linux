// SPDX-License-Identifier: GPL-2.0 OR MIT

/*
 * Xen para-virtual camera device
 *
 * Copyright (C) 2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#include <xen/xenbus.h>

#include "xen_camera_front.h"

struct cfg_control_ids {
	u32 xen_type;
	u32 v4l2_cid;
};

static const struct cfg_control_ids CFG_CONTROL_IDS[] = {
	{
		.xen_type = XENCAMERA_CTRL_BRIGHTNESS,
		.v4l2_cid = V4L2_CID_BRIGHTNESS,
	},
	{
		.xen_type = XENCAMERA_CTRL_CONTRAST,
		.v4l2_cid = V4L2_CID_CONTRAST,
	},
	{
		.xen_type = XENCAMERA_CTRL_SATURATION,
		.v4l2_cid = V4L2_CID_SATURATION,
	},
	{
		.xen_type = XENCAMERA_CTRL_HUE,
		.v4l2_cid = V4L2_CID_HUE
	},
};

int xen_camera_front_cfg_to_v4l2_cid(int xen_type)
{
	int i;

	for (i = 0; i < XENCAMERA_MAX_CTRL; i++)
		if (CFG_CONTROL_IDS[i].xen_type == xen_type)
			return CFG_CONTROL_IDS[i].v4l2_cid;
	return -1;
}
