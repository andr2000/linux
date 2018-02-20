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
	/* These are only for connector mode checking */
	int width, height;

	struct drm_pending_vblank_event *pending_event;

	/*
	 * pflip_timeout is set to current jiffies once we send a page flip and
	 * reset to 0 when we receive frame done event from the backed.
	 * It is checked during drm_connector_helper_funcs.detect_ctx to detect
	 * time-outs for frame done event, e.g. due to backend errors.
	 *
	 * This must be protected with front_info->io_lock, so races between
	 * interrupt handler and rest of the code are properly handled.
	 */
	unsigned long pflip_timeout;

	bool conn_connected;
};

struct xen_drm_front_drm_info {
	struct xen_drm_front_info *front_info;
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

int xen_drm_front_drv_probe(struct platform_device *pdev);

int xen_drm_front_drv_remove(struct platform_device *pdev);

bool xen_drm_front_drv_is_used(struct platform_device *pdev);

void xen_drm_front_on_frame_done(struct platform_device *pdev,
		int conn_idx, uint64_t fb_cookie);

#endif /* __XEN_DRM_FRONT_DRV_H_ */

