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

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>

#include "xen_camera_front_cfg.h"
#include "xen_camera_front_evtchnl.h"

struct xen_camera_front_info {
	struct xenbus_device *xb_dev;
	struct xen_camera_front_v4l2_info *v4l2_info;

	struct xen_camera_front_evtchnl_pair evt_pair;

	/* To protect data between backend IO code and interrupt handler. */
	spinlock_t io_lock;

	struct xen_camera_front_cfg_card cfg;
};

struct xen_camera_front_v4l2_info {
	struct xen_camera_front_info *front_info;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_ctrl_handler ctrl_handler;
	/* ioctl serialization mutex. */
	struct mutex lock;

	struct vb2_queue queue;
};

#endif /* __XEN_CAMERA_FRONT_H */
