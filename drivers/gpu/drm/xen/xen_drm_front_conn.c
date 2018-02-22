// SPDX-License-Identifier: GPL-2.0 OR MIT

/*
 *  Xen para-virtual DRM device
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

#define XEN_DRM_NUM_VIDEO_MODES		1
#define XEN_DRM_CRTC_VREFRESH_HZ	60

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
#if LINUX_VERSION_CODE < PV_DRM_LINUX_VERSION
	.dpms = drm_atomic_helper_connector_dpms,
#else
	.dpms = drm_helper_connector_dpms,
#endif
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
