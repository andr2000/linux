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

#include "xen_drm_front.h"
#include "xen_drm_front_cfg.h"

struct xen_drm_front_drm_pipeline {
	struct xen_drm_front_drm_info *drm_info;

	int index;
};

struct xen_drm_front_drm_info {
	struct xen_drm_front_info *front_info;
	struct xen_drm_front_ops *front_ops;
	struct drm_device *drm_dev;
	struct xen_drm_front_cfg *cfg;
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

