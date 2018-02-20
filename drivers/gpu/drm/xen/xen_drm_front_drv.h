/* SPDX-License-Identifier: GPL-2.0 OR MIT */

/*
 *  Xen para-virtual DRM device
 *
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_DRM_FRONT_DRV_H_
#define __XEN_DRM_FRONT_DRV_H_

#include <drm/drmP.h>
#include <drm/drm_simple_kms_helper.h>

#include "xen_drm_front.h"
#include "xen_drm_front_cfg.h"
#include "xen_drm_front_conn.h"

struct xen_drm_front_drm_pipeline {
	struct xen_drm_front_drm_info *drm_info;

	int index;

	struct drm_simple_display_pipe pipe;

	struct drm_connector conn;
	/* these are only for connector mode checking */
	int width, height;
	/* last backend error seen on page flip */
	int pgflip_last_error;
};

struct xen_drm_front_drm_info {
	struct xen_drm_front_info *front_info;
	struct xen_drm_front_ops *front_ops;
	const struct xen_drm_front_gem_ops *gem_ops;
	struct drm_device *drm_dev;
	struct xen_drm_front_cfg *cfg;

	struct xen_drm_front_drm_pipeline pipeline[XEN_DRM_FRONT_MAX_CRTCS];
};

static inline uint64_t xen_drm_front_fb_to_cookie(
		struct drm_framebuffer *fb)
{
	return (uint64_t)fb;
}

static inline uint64_t xen_drm_front_dbuf_to_cookie(
		struct drm_gem_object *gem_obj)
{
	return (uint64_t)gem_obj;
}

int xen_drm_front_drv_probe(struct platform_device *pdev,
		struct xen_drm_front_ops *front_ops);

int xen_drm_front_drv_remove(struct platform_device *pdev);

bool xen_drm_front_drv_is_used(struct platform_device *pdev);

#endif /* __XEN_DRM_FRONT_DRV_H_ */

