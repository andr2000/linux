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

#ifndef __XEN_DRM_FRONT_GEM_H
#define __XEN_DRM_FRONT_GEM_H

#include <drm/drmP.h>

struct xen_drm_front_gem_ops {
	void (*free_object_unlocked)(struct drm_gem_object *obj);

	struct sg_table *(*prime_get_sg_table)(struct drm_gem_object *obj);
	struct drm_gem_object *(*prime_import_sg_table)(
		struct drm_device *dev, struct dma_buf_attachment *attach,
		struct sg_table *sgt);

	void *(*prime_vmap)(struct drm_gem_object *obj);
	void (*prime_vunmap)(struct drm_gem_object *obj, void *vaddr);
	int (*prime_mmap)(struct drm_gem_object *obj,
		struct vm_area_struct *vma);

	int (*dumb_create)(struct drm_file *file_priv, struct drm_device *dev,
		struct drm_mode_create_dumb *args);

	int (*mmap)(struct file *filp, struct vm_area_struct *vma);

	struct page **(*get_pages)(struct drm_gem_object *obj);
};

const struct xen_drm_front_gem_ops *xen_drm_front_gem_get_ops(void);

#endif /* __XEN_DRM_FRONT_GEM_H */
