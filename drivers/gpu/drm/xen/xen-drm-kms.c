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
 * Copyright (C) 2016 EPAM Systems Inc.
 */

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>

#include "xen-drm.h"
#include "xen-drm-front.h"
#include "xen-drm-gem.h"
#include "xen-drm-kms.h"

static void xendrm_fb_destroy(struct drm_framebuffer *fb)
{
	struct xendrm_device *xendrm_dev = fb->dev->dev_private;

	xendrm_dev->front_ops->fb_detach(xendrm_dev->xdrv_info,
		xendrm_fb_to_cookie(fb));
	xendrm_gem_fb_destroy(fb);
}

static struct drm_framebuffer_funcs xendr_du_fb_funcs = {
	.destroy = xendrm_fb_destroy,
};

static struct drm_framebuffer *
xendrm_fb_create(struct drm_device *dev, struct drm_file *file_priv,
	const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct xendrm_device *xendrm_dev = dev->dev_private;
	static struct drm_framebuffer *fb;

	fb = xendrm_gem_fb_create_with_funcs(dev, file_priv,
		mode_cmd, &xendr_du_fb_funcs);
	if (!IS_ERR_OR_NULL(fb)) {
		/* FIXME: we take the first handle */
		if (xendrm_dev->front_ops->fb_attach(
				xendrm_dev->xdrv_info, mode_cmd->handles[0],
				xendrm_fb_to_cookie(fb), fb->width, fb->height,
				fb->pixel_format) < 0) {
			DRM_ERROR("Back failed to attach FB %p\n", fb);
			xendrm_gem_fb_destroy(fb);
			fb = NULL;
		}
	}
	return fb;
}

static const struct drm_mode_config_funcs xendrm_mode_config_funcs = {
	.fb_create = xendrm_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

int xendrm_modeset_init(struct xendrm_device *xendrm_dev)
{
	struct drm_device *drm_dev = xendrm_dev->drm;
	int i, ret;

	drm_mode_config_init(drm_dev);

	drm_dev->mode_config.min_width = 0;
	drm_dev->mode_config.min_height = 0;
	drm_dev->mode_config.max_width = 4095;
	drm_dev->mode_config.max_height = 2047;
	drm_dev->mode_config.funcs = &xendrm_mode_config_funcs;

	for (i = 0; i < xendrm_dev->num_crtcs; i++) {
		struct xendrm_crtc *crtc;

		crtc = &xendrm_dev->crtcs[i];
		ret = xendrm_crtc_create(xendrm_dev, crtc, i);
		if (ret < 0)
			goto fail;
		ret = xendrm_encoder_create(xendrm_dev, crtc);
		if (ret)
			goto fail;
		ret = xendrm_connector_create(xendrm_dev, crtc,
			&xendrm_dev->platdata->connectors[i]);
		if (ret)
			goto fail;
	}
	drm_mode_config_reset(drm_dev);

	return 0;
fail:
	drm_mode_config_cleanup(drm_dev);
	return ret;
}
