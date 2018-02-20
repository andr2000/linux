/* SPDX-License-Identifier: GPL-2.0 OR MIT */

/*
 *  Xen para-virtual DRM device
 *
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_DRM_FRONT_KMS_H_
#define __XEN_DRM_FRONT_KMS_H_

#include "xen_drm_front_drv.h"

int xen_drm_front_kms_init(struct xen_drm_front_drm_info *drm_info);

void xen_drm_front_kms_on_frame_done(
		struct xen_drm_front_drm_pipeline *pipeline,
		uint64_t fb_cookie);

void xen_drm_front_kms_send_pending_event(
		struct xen_drm_front_drm_pipeline *pipeline);

#endif /* __XEN_DRM_FRONT_KMS_H_ */
