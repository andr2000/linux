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

#include "xen_drm_front_kms.h"

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_framebuffer_helper.h>

#include "xen_drm_front.h"
#include "xen_drm_front_conn.h"
#include "xen_drm_front_drv.h"
#include "xen_drm_front_gem.h"

static struct xen_drm_front_drm_pipeline *
to_xen_drm_pipeline(struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct xen_drm_front_drm_pipeline, pipe);
}

static void fb_destroy(struct drm_framebuffer *fb)
{
	struct xen_drm_front_drm_info *drm_info = fb->dev->dev_private;

	drm_info->front_ops->fb_detach(drm_info->front_info,
		xen_drm_front_fb_to_cookie(fb));
	drm_gem_fb_destroy(fb);
}

static struct drm_framebuffer_funcs fb_funcs = {
	.destroy = fb_destroy,
};

static struct drm_framebuffer *fb_create(struct drm_device *dev,
	struct drm_file *file_priv, const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct xen_drm_front_drm_info *drm_info = dev->dev_private;
	static struct drm_framebuffer *fb;
	struct drm_gem_object *gem_obj;
	int ret;

	fb = drm_gem_fb_create_with_funcs(dev, file_priv, mode_cmd, &fb_funcs);
	if (IS_ERR(fb))
		return fb;

	gem_obj = drm_gem_object_lookup(file_priv, mode_cmd->handles[0]);
	if (!gem_obj) {
		DRM_ERROR("Failed to lookup GEM object\n");
		ret = -ENOENT;
		goto fail;
	}

	drm_gem_object_unreference_unlocked(gem_obj);

	ret = drm_info->front_ops->fb_attach(
		drm_info->front_info, xen_drm_front_dbuf_to_cookie(gem_obj),
		xen_drm_front_fb_to_cookie(fb), fb->width, fb->height,
		fb->format->format);
	if (ret < 0) {
		DRM_ERROR("Back failed to attach FB %p: %d\n", fb, ret);
		goto fail;
	}

	return fb;

fail:
	drm_gem_fb_destroy(fb);
	return ERR_PTR(ret);
}

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int display_set_config(struct drm_simple_display_pipe *pipe,
	struct drm_framebuffer *fb)
{
	struct xen_drm_front_drm_pipeline *pipeline =
			to_xen_drm_pipeline(pipe);
	struct drm_crtc *crtc = &pipe->crtc;
	struct xen_drm_front_drm_info *drm_info = pipeline->drm_info;
	int ret;

	if (fb) {
		ret = drm_info->front_ops->mode_set(pipeline,
			crtc->x, crtc->y,
			fb->width, fb->height, fb->format->cpp[0] * 8,
			xen_drm_front_fb_to_cookie(fb));
		if (ret)
			DRM_ERROR("Failed to set mode to back: %d\n", ret);
	} else {
		ret = drm_info->front_ops->mode_set(pipeline,
			0, 0, 0, 0, 0, 0);
		if (ret)
			DRM_ERROR("Failed to set mode to back: %d\n", ret);
	}
	return ret;
}

static void display_enable(struct drm_simple_display_pipe *pipe,
	struct drm_crtc_state *crtc_state)
{
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_framebuffer *fb = pipe->plane.state->fb;

	if (display_set_config(pipe, fb) == 0)
		drm_crtc_vblank_on(crtc);
	else
		DRM_ERROR("Failed to enable display\n");
}

static void display_disable(struct drm_simple_display_pipe *pipe)
{
	struct drm_crtc *crtc = &pipe->crtc;

	display_set_config(pipe, NULL);
	drm_crtc_vblank_off(crtc);
}

void xen_drm_front_kms_on_page_flip_done(
	struct xen_drm_front_drm_pipeline *pipeline, uint64_t fb_cookie)
{
	drm_crtc_handle_vblank(&pipeline->pipe.crtc);
}

static int display_prepare_fb(struct drm_simple_display_pipe *pipe,
	struct drm_plane_state *plane_state)
{
	struct drm_atomic_state *state = plane_state->state;
	struct drm_plane_state *old_plane_state;

	old_plane_state = drm_atomic_get_old_plane_state(state,
		plane_state->plane);

	/*
	 * We talk to the backend here as we can return error code now if any,
	 * effectively stopping the commit. Talking in .update callback will
	 * be too late.
	 *
	 * If old_plane_state->fb is NULL and plane_state->fb is not,
	 * then this is an atomic commit which will enable display
	 * later on in this commit sequence (display_enable will be called).
	 * Ignore this and do not send page flip as this framebuffer will be
	 * set to the backend as a part of display_set_config
	 */
	if (old_plane_state->fb && plane_state->fb) {
		struct xen_drm_front_drm_pipeline *pipeline =
			to_xen_drm_pipeline(pipe);
		struct xen_drm_front_drm_info *drm_info = pipeline->drm_info;
		int ret;

		ret = drm_info->front_ops->page_flip(drm_info->front_info,
			pipeline->index,
			xen_drm_front_fb_to_cookie(plane_state->fb));
		if (ret) {
			DRM_ERROR("Failed to send page flip request to backend: %d\n", ret);
			/*
			 * according to drm_plane_helper_funcs.prepare_fb
			 * the list of allowed error codes from this callback
			 * is limited by drm_mode_config_funcs.atomic_commit:
			 * we can only return -EBUSY, -ENOMEM, -ENOSPC,
			 * -EIO, -EINTR, -EAGAIN or -ERESTARTSYS
			 *
			 * - -EIO, if the hardware completely died.
			 *
			 * - -EAGAIN if the IOCTL should be restarted.
			 *
			 */
			if (ret == -EIO)
				return ret;

			return -EAGAIN;
		}
	}

	return drm_gem_fb_prepare_fb(&pipe->plane, plane_state);
}

static void display_update(struct drm_simple_display_pipe *pipe,
	struct drm_plane_state *old_plane_state)
{
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_device *dev = crtc->dev;
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	event = crtc->state->event;
	if (event) {
		crtc->state->event = NULL;

		spin_lock_irqsave(&dev->event_lock, flags);
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}
}

static const struct drm_simple_display_pipe_funcs display_funcs = {
	.enable = display_enable,
	.disable = display_disable,
	.prepare_fb = display_prepare_fb,
	.update = display_update,
};

static int display_pipe_init(struct xen_drm_front_drm_info *drm_info,
	int index, struct xen_drm_front_cfg_connector *cfg,
	struct xen_drm_front_drm_pipeline *pipeline)
{
	struct drm_device *dev = drm_info->drm_dev;
	const uint32_t *formats;
	int format_count;
	int ret;

	pipeline->drm_info = drm_info;
	pipeline->index = index;
	pipeline->height = cfg->height;
	pipeline->width = cfg->width;

	ret = xen_drm_front_conn_init(drm_info, &pipeline->conn);
	if (ret)
		return ret;

	formats = xen_drm_front_conn_get_formats(&format_count);

	return drm_simple_display_pipe_init(dev, &pipeline->pipe,
		&display_funcs, formats, format_count, NULL, &pipeline->conn);
}

int xen_drm_front_kms_init(struct xen_drm_front_drm_info *drm_info)
{
	struct drm_device *dev = drm_info->drm_dev;
	int i, ret;

	drm_mode_config_init(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = 4095;
	dev->mode_config.max_height = 2047;
	dev->mode_config.funcs = &mode_config_funcs;

	for (i = 0; i < drm_info->cfg->num_connectors; i++) {
		struct xen_drm_front_cfg_connector *cfg =
			&drm_info->cfg->connectors[i];
		struct xen_drm_front_drm_pipeline *pipeline =
			&drm_info->pipeline[i];

		ret = display_pipe_init(drm_info, i, cfg, pipeline);
		if (ret)
			goto fail;
	}

	drm_mode_config_reset(dev);
	return 0;

fail:
	drm_mode_config_cleanup(dev);
	return ret;
}
