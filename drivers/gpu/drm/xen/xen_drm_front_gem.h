/* SPDX-License-Identifier: GPL-2.0 OR MIT */

/*
 *  Xen para-virtual DRM device
 *
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_DRM_FRONT_GEM_H
#define __XEN_DRM_FRONT_GEM_H

#include <drm/drmP.h>

#include "xen_drm_front_backport.h"

#ifndef CONFIG_DRM_XEN_FRONTEND_CMA
#include <drm/drm_gem.h>

struct xen_gem_object {
	struct drm_gem_object base;

	size_t num_pages;
	struct page **pages;

	/* set for buffers allocated by the backend */
	bool be_alloc;

	/* this is for imported PRIME buffer */
	struct sg_table *sgt_imported;
};

static inline struct xen_gem_object *to_xen_gem_obj(
		struct drm_gem_object *gem_obj)
{
	return container_of(gem_obj, struct xen_gem_object, base);
}

#endif

struct drm_gem_object *xen_drm_front_gem_create(struct drm_device *dev,
		size_t size);

int xen_drm_front_gem_dumb_create(struct drm_file *filp, struct drm_device *dev,
		struct drm_mode_create_dumb *args);

struct drm_gem_object *xen_drm_front_gem_import_sg_table(struct drm_device *dev,
		struct dma_buf_attachment *attach, struct sg_table *sgt);

struct sg_table *xen_drm_front_gem_get_sg_table(struct drm_gem_object *gem_obj);

struct page **xen_drm_front_gem_get_pages(struct drm_gem_object *obj);

void xen_drm_front_gem_free_object_unlocked(struct drm_gem_object *gem_obj);

#ifndef CONFIG_DRM_XEN_FRONTEND_CMA

int xen_drm_front_gem_mmap(struct file *filp, struct vm_area_struct *vma);

void *xen_drm_front_gem_prime_vmap(struct drm_gem_object *gem_obj);

void xen_drm_front_gem_prime_vunmap(struct drm_gem_object *gem_obj,
		void *vaddr);

int xen_drm_front_gem_prime_mmap(struct drm_gem_object *gem_obj,
		struct vm_area_struct *vma);
#endif

#endif /* __XEN_DRM_FRONT_GEM_H */
