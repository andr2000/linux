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

#ifndef __XEN_DRM_FRONT_CRTC_H_
#define __XEN_DRM_FRONT_CRTC_H_

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_encoder.h>

#include <linux/wait.h>

#define XEN_DRM_CRTC_VREFRESH_HZ	60

struct xen_drm_front_drm_info;
struct xen_drm_front_cfg_connector;

struct xen_drm_front_connector {
	struct drm_connector base;
	/* these are only for mode checking */
	int width, height;
};

struct xen_drm_front_crtc {
	int index;
	struct xen_drm_front_drm_info *drm_info;
	struct drm_plane primary;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct xen_drm_front_connector xen_connector;

	/* vblank and flip handling */
	spinlock_t pg_flip_event_lock;
	struct drm_pending_vblank_event *pg_flip_event;
	wait_queue_head_t flip_wait;
	/* page flip event time-out handling */
	struct timer_list pg_flip_to_timer;
	/* current fb cookie */
	uint64_t fb_cookie;
	/* last page flip error code while communicating with the backend */
	int pg_flip_last_status;
};

int xen_drm_front_crtc_init(struct xen_drm_front_drm_info *drm_info,
	struct xen_drm_front_crtc *xen_crtc, int index, int width, int height);

void xen_drm_front_crtc_on_page_flip_done(struct xen_drm_front_crtc *xen_crtc,
	uint64_t fb_cookie);

#endif /* __XEN_DRM_FRONT_CRTC_H_ */
