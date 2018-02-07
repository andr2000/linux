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

#ifndef __XEN_DRM_FRONT_CONN_H_
#define __XEN_DRM_FRONT_CONN_H_

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_encoder.h>

#include <linux/wait.h>

#define XEN_DRM_CRTC_VREFRESH_HZ	60

struct xen_drm_front_drm_info;
struct xen_drm_front_drm_pipeline;

const uint32_t *xen_drm_front_conn_get_formats(int *format_count);

int xen_drm_front_conn_init(struct xen_drm_front_drm_info *drm_info,
	struct drm_connector *connector);

#endif /* __XEN_DRM_FRONT_CONN_H_ */
