/* SPDX-License-Identifier: GPL-2.0 OR MIT */

/*
 *  Xen para-virtual DRM device
 *
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_DRM_FRONT_CONN_H_
#define __XEN_DRM_FRONT_CONN_H_

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_encoder.h>

#include <linux/wait.h>

struct xen_drm_front_drm_info;

const uint32_t *xen_drm_front_conn_get_formats(int *format_count);

int xen_drm_front_conn_init(struct xen_drm_front_drm_info *drm_info,
		struct drm_connector *connector);

#endif /* __XEN_DRM_FRONT_CONN_H_ */
