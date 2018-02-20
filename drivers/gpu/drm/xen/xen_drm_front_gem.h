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

struct xen_drm_front_gem_ops {
	void (*free_object_unlocked)(struct drm_gem_object *obj);

	struct sg_table *(*prime_get_sg_table)(struct drm_gem_object *obj);
	struct drm_gem_object *(*prime_import_sg_table)(struct drm_device *dev,
			struct dma_buf_attachment *attach,
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
