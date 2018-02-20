// SPDX-License-Identifier: GPL-2.0 OR MIT

/*
 *  Xen para-virtual DRM device
 *
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#include <drm/drmP.h>
#include <drm/drm_gem.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "xen_drm_front.h"
#include "xen_drm_front_drv.h"
#include "xen_drm_front_gem.h"

struct drm_gem_object *xen_drm_front_gem_import_sg_table(struct drm_device *dev,
		struct dma_buf_attachment *attach, struct sg_table *sgt)
{
	struct xen_drm_front_drm_info *drm_info = dev->dev_private;
	struct drm_gem_object *gem_obj;
	struct drm_gem_cma_object *cma_obj;
	int ret;

	gem_obj = drm_gem_cma_prime_import_sg_table(dev, attach, sgt);
	if (IS_ERR_OR_NULL(gem_obj))
		return gem_obj;

	cma_obj = to_drm_gem_cma_obj(gem_obj);

	ret = xen_drm_front_dbuf_create_from_sgt(
			drm_info->front_info,
			xen_drm_front_dbuf_to_cookie(gem_obj),
			0, 0, 0, gem_obj->size,
			drm_gem_cma_prime_get_sg_table(gem_obj));
	if (ret < 0)
		return ERR_PTR(ret);

	DRM_DEBUG("Imported CMA buffer of size %zu\n", gem_obj->size);

	return gem_obj;
}

struct sg_table *xen_drm_front_gem_get_sg_table(struct drm_gem_object *gem_obj)
{
	return drm_gem_cma_prime_get_sg_table(gem_obj);
}

int xen_drm_front_gem_dumb_create(struct drm_file *filp, struct drm_device *dev,
	struct drm_mode_create_dumb *args)
{
	struct xen_drm_front_drm_info *drm_info = dev->dev_private;

	if (drm_info->cfg->be_alloc) {
		/* This use-case is not yet supported and probably won't be */
		DRM_ERROR("Backend allocated buffers and CMA helpers are not supported at the same time\n");
		return -EINVAL;
	}

	return drm_gem_cma_dumb_create(filp, dev, args);
}

void xen_drm_front_gem_free_object(struct drm_gem_object *gem_obj)
{
	drm_gem_cma_free_object(gem_obj);
}

struct page **xen_drm_front_gem_get_pages(struct drm_gem_object *gem_obj)
{
	return NULL;
}
