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


#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>

#include <video/videomode.h>

#include "xen_drm_front_conn.h"
#include "xen_drm_front_drv.h"

/*
 * timeout for page flip event reception: should be a little
 * bit more than frontend/backend i/o timeout
 */
#define XEN_DRM_CRTC_PFLIP_TO_MS	(VDRM_WAIT_BACK_MS + 100)

static struct xen_drm_front_drm_pipeline *
to_xen_drm_pipeline(struct drm_connector *connector)
{
	return container_of(connector, struct xen_drm_front_drm_pipeline, conn);
}

static const uint32_t plane_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB4444,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_ARGB1555,
};

const uint32_t *xen_drm_front_conn_get_formats(int *format_count)
{
	*format_count = ARRAY_SIZE(plane_formats);
	return plane_formats;
}

static enum drm_connector_status connector_detect(
	struct drm_connector *connector, bool force)
{
	if (drm_dev_is_unplugged(connector->dev))
		return connector_status_disconnected;

	return connector_status_connected;
}

#define XEN_DRM_NUM_VIDEO_MODES	1

static int connector_get_modes(struct drm_connector *connector)
{
	struct xen_drm_front_drm_pipeline *pipeline =
		to_xen_drm_pipeline(connector);
	struct drm_display_mode *mode;
	struct videomode videomode;
	int width, height;

	mode = drm_mode_create(connector->dev);
	if (!mode)
		return 0;

	memset(&videomode, 0, sizeof(videomode));
	videomode.hactive = pipeline->width;
	videomode.vactive = pipeline->height;
	width = videomode.hactive + videomode.hfront_porch +
		videomode.hback_porch + videomode.hsync_len;
	height = videomode.vactive + videomode.vfront_porch +
		videomode.vback_porch + videomode.vsync_len;
	videomode.pixelclock = width * height * XEN_DRM_CRTC_VREFRESH_HZ;
	mode->type = DRM_MODE_TYPE_PREFERRED | DRM_MODE_TYPE_DRIVER;
	drm_display_mode_from_videomode(&videomode, mode);
	drm_mode_probed_add(connector, mode);
	return XEN_DRM_NUM_VIDEO_MODES;
}

static int connector_mode_valid(struct drm_connector *connector,
	struct drm_display_mode *mode)
{
	struct xen_drm_front_drm_pipeline *pipeline =
		to_xen_drm_pipeline(connector);

	if (mode->hdisplay != pipeline->width)
		return MODE_ERROR;

	if (mode->vdisplay != pipeline->height)
		return MODE_ERROR;

	return MODE_OK;
}

static const struct drm_connector_helper_funcs connector_helper_funcs = {
	.get_modes = connector_get_modes,
	.mode_valid = connector_mode_valid,
};

static const struct drm_connector_funcs connector_funcs = {
	.detect = connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

int xen_drm_front_conn_init(struct xen_drm_front_drm_info *drm_info,
	struct drm_connector *connector)
{
	drm_connector_helper_add(connector, &connector_helper_funcs);

	return drm_connector_init(drm_info->drm_dev, connector,
		&connector_funcs, DRM_MODE_CONNECTOR_VIRTUAL);
}

#if 0
static bool crtc_page_flip_pending(struct xen_drm_front_crtc *xen_crtc)
{
	bool pending;
	unsigned long flags;

	spin_lock_irqsave(&xen_crtc->pg_flip_event_lock, flags);
	pending = xen_crtc->pg_flip_event != NULL;
	spin_unlock_irqrestore(&xen_crtc->pg_flip_event_lock, flags);
	return pending;
}

static void crtc_ntfy_page_flip_completed(struct xen_drm_front_crtc *xen_crtc)
{
	struct drm_device *dev = xen_crtc->crtc.dev;
	unsigned long flags;

	del_timer(&xen_crtc->pg_flip_to_timer);

	spin_lock_irqsave(&xen_crtc->pg_flip_event_lock, flags);

	if (unlikely(!xen_crtc->pg_flip_event)) {
		spin_unlock_irqrestore(&xen_crtc->pg_flip_event_lock, flags);
		return;
	}

	spin_lock(&dev->event_lock);
	drm_crtc_send_vblank_event(&xen_crtc->crtc, xen_crtc->pg_flip_event);
	spin_unlock(&dev->event_lock);

	xen_crtc->pg_flip_event = NULL;
	xen_crtc->fb_cookie = xen_drm_front_fb_to_cookie(NULL);

	spin_unlock_irqrestore(&xen_crtc->pg_flip_event_lock, flags);

	wake_up(&xen_crtc->flip_wait);

	drm_crtc_vblank_put(&xen_crtc->crtc);
}

static void crtc_on_page_flip_to(struct timer_list *t)
{
	struct xen_drm_front_crtc *xen_crtc =
		from_timer(xen_crtc, t, pg_flip_to_timer);

	if (crtc_page_flip_pending(xen_crtc)) {
		DRM_ERROR("Flip event timed-out, releasing\n");
		crtc_ntfy_page_flip_completed(xen_crtc);
	}
}

void xen_drm_front_crtc_on_page_flip_done(struct xen_drm_front_crtc *xen_crtc,
	uint64_t fb_cookie)
{
	uint64_t fb_cookie_expected;
	unsigned long flags;

	spin_lock_irqsave(&xen_crtc->pg_flip_event_lock, flags);
	fb_cookie_expected = xen_crtc->fb_cookie;
	spin_unlock_irqrestore(&xen_crtc->pg_flip_event_lock, flags);
	if (unlikely(fb_cookie_expected != fb_cookie)) {
		DRM_ERROR("Out-of-order page flip event: expected %llx got %llx\n",
			fb_cookie_expected, fb_cookie);
		return;
	}

	crtc_ntfy_page_flip_completed(xen_crtc);
}


static void crtc_disable(struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state)
{
	struct xen_drm_front_crtc *xen_crtc = to_xen_drm_crtc(crtc);

	del_timer_sync(&xen_crtc->pg_flip_to_timer);

	if (wait_event_timeout(xen_crtc->flip_wait,
			!crtc_page_flip_pending(xen_crtc),
			msecs_to_jiffies(XEN_DRM_CRTC_PFLIP_TO_MS)) == 0) {
		crtc_ntfy_page_flip_completed(xen_crtc);
	}

	drm_crtc_vblank_off(crtc);
}

static void crtc_enable(struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state)
{
	drm_crtc_vblank_on(crtc);
}

static int crtc_atomic_check(struct drm_crtc *crtc,
	struct drm_crtc_state *state)
{
	struct xen_drm_front_crtc *xen_crtc = to_xen_drm_crtc(crtc);
	int ret;

	ret = xen_crtc->pg_flip_last_status;
	/*
	 * let the backend a chance to recover: reset the last error, so another
	 * .atomic_check will have a chance to let .atomc_flush run
	 */
	xen_crtc->pg_flip_last_status = 0;
	return ret;
}

static void crtc_atomic_flush(struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state)
{
	struct xen_drm_front_crtc *xen_crtc = to_xen_drm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct drm_framebuffer *fb = crtc->primary->fb;
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);
	event = crtc->state->event;
	crtc->state->event = NULL;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	if (fb) {
		struct xen_drm_front_drm_info *drm_info = xen_crtc->drm_info;
		int ret;

		WARN_ON(drm_crtc_vblank_get(crtc) != 0);

		spin_lock_irqsave(&xen_crtc->pg_flip_event_lock, flags);
		if (event && (event->event.base.type == DRM_EVENT_FLIP_COMPLETE)) {
			/*
			 * save the event, so we can deliver it later when
			 * page flip event from the backend comes in
			 */
			xen_crtc->pg_flip_event = event;
			/* make sure we do not use it down the code */
			event = NULL;
		}
		xen_crtc->fb_cookie = xen_drm_front_fb_to_cookie(fb);
		spin_unlock_irqrestore(&xen_crtc->pg_flip_event_lock, flags);

		ret = drm_info->front_ops->page_flip(drm_info->front_info,
			xen_crtc->index, xen_drm_front_fb_to_cookie(fb));
		xen_crtc->pg_flip_last_status = ret;
		if (ret) {
			/*
			 * .atomic_flush is a point of no return, we cannot
			 * fail here. So, if the below request to the backend
			 * fails, then the best we can do is to signal it is
			 * done, so at least guest user-space is unblocked now.
			 * Also, we remember the error code, so we can report it
			 * during .atomic_check
			 */
			crtc_ntfy_page_flip_completed(xen_crtc);
			DRM_ERROR("Failed to flip the page on backend side, ret %d\n", ret);
		} else
			mod_timer(&xen_crtc->pg_flip_to_timer,
				jiffies + msecs_to_jiffies(XEN_DRM_CRTC_PFLIP_TO_MS));
	}

	if (event) {
		spin_lock_irqsave(&dev->event_lock, flags);
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}
}
#endif
