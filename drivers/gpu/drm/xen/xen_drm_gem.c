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
 *
 * Author: Oleksandr Andrushchenko <Oleksandr_Andrushchenko@epam.com>
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem.h>

#include <linux/dma-buf.h>
#include <linux/scatterlist.h>
#include <linux/shmem_fs.h>

#include "xen_drm.h"
#include "xen_drm_gem.h"

struct xen_gem_object {
	struct drm_gem_object base;
	/* number of pages */
	size_t num_pages;
	/* for buffer pages allocated either by us or by the backend,
	 * imported PRIME will never be here
	 */
	struct page **pages;

	/* set if buffer was allocated by the backend */
	bool be_alloc;

	/* this is for imported PRIME buffer */
	struct sg_table *sgt_imported;
};

struct xen_fb {
	struct drm_framebuffer fb;
	struct xen_gem_object *xen_obj;
};

static inline struct xen_gem_object *to_xen_gem_obj(
	struct drm_gem_object *gem_obj)
{
	return container_of(gem_obj, struct xen_gem_object, base);
}

static inline struct xen_fb *to_xen_fb(struct drm_framebuffer *fb)
{
	return container_of(fb, struct xen_fb, fb);
}

static struct xen_gem_object *xendrm_gem_create_obj(struct drm_device *dev,
	size_t size)
{
	struct xen_gem_object *xen_obj;
	int ret;

	xen_obj = kzalloc(sizeof(*xen_obj), GFP_KERNEL);
	if (!xen_obj)
		return ERR_PTR(-ENOMEM);
	ret = drm_gem_object_init(dev, &xen_obj->base, size);
	if (ret)
		goto error;
	return xen_obj;

error:
	kfree(xen_obj);
	return ERR_PTR(ret);
}

static struct xen_gem_object *xendrm_gem_create(struct drm_device *dev,
	size_t size)
{
	struct xendrm_device *xendrm_dev = dev->dev_private;
	struct xen_gem_object *xen_obj;

	DRM_ERROR("%s size %zu\n", __FUNCTION__, size);
	size = round_up(size, PAGE_SIZE);
	xen_obj = xendrm_gem_create_obj(dev, size);
	if (IS_ERR(xen_obj))
		return xen_obj;
	xen_obj->num_pages = DIV_ROUND_UP(size, PAGE_SIZE);
	/* if backend has already allocated space for this buffer
	 * then we are done: pages array will be allocated on
	 * xendrm_gem_set_ext_sg_table
	 */
	if (xendrm_dev->platdata->be_alloc)
		return xen_obj;
	/* need to allocate this buffer now, so we can share it with the
	 * backend
	 */
	xen_obj->pages = drm_gem_get_pages(&xen_obj->base);
	if (IS_ERR(xen_obj->pages))
		goto fail;
	return xen_obj;

fail:
	DRM_ERROR("Failed to allocate buffer with size %zu\n", size);
	drm_gem_object_unreference_unlocked(&xen_obj->base);
	return xen_obj;
}

static struct xen_gem_object *xendrm_gem_create_with_handle(
	struct drm_file *file_priv, struct drm_device *dev,
	size_t size, uint32_t *handle)
{
	struct xen_gem_object *xen_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	DRM_ERROR("%s size %zu\n", __FUNCTION__, size);
	xen_obj = xendrm_gem_create(dev, size);
	if (IS_ERR(xen_obj))
		return xen_obj;
	gem_obj = &xen_obj->base;
	mapping_set_gfp_mask(gem_obj->filp->f_mapping, GFP_USER | __GFP_DMA32);
	ret = drm_gem_handle_create(file_priv, gem_obj, handle);
	/* handle holds the reference */
	drm_gem_object_unreference_unlocked(gem_obj);
	if (ret)
		return ERR_PTR(ret);
	return xen_obj;
}

int xendrm_gem_dumb_create(struct drm_file *file_priv,
	struct drm_device *dev, struct drm_mode_create_dumb *args)
{
	struct xen_gem_object *xen_obj;

	args->pitch = DIV_ROUND_UP(args->width * args->bpp, 8);
	args->size = args->pitch * args->height;

	xen_obj = xendrm_gem_create_with_handle(file_priv, dev, args->size,
		&args->handle);
	DRM_ERROR("%s handle %d\n", __FUNCTION__, args->handle);
	return PTR_ERR_OR_ZERO(xen_obj);
}

void xendrm_gem_free_object(struct drm_gem_object *gem_obj)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);

	DRM_ERROR("%s\n", __FUNCTION__);
	if (xen_obj->pages) {
		if (xen_obj->be_alloc)
			drm_free_large(xen_obj->pages);
		else
			drm_gem_put_pages(&xen_obj->base, xen_obj->pages,
				true, false);
	}
	if (xen_obj->base.import_attach)
		drm_prime_gem_destroy(&xen_obj->base, xen_obj->sgt_imported);
	drm_gem_object_release(gem_obj);
	kfree(xen_obj);
}

struct sg_table *xendrm_gem_get_sg_table(struct drm_gem_object *gem_obj)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);

	DRM_ERROR("%s xen_obj->pages %p\n", __FUNCTION__, xen_obj->pages);
	if (!xen_obj->pages)
		return NULL;
	return drm_prime_pages_to_sg(xen_obj->pages, xen_obj->num_pages);
}

struct drm_gem_object *xendrm_gem_import_sg_table(struct drm_device *dev,
	struct dma_buf_attachment *attach, struct sg_table *sgt)
{
	struct xen_gem_object *xen_obj;

	DRM_ERROR("%s\n", __FUNCTION__);
	xen_obj = xendrm_gem_create_obj(dev, attach->dmabuf->size);
	if (IS_ERR(xen_obj))
		return ERR_CAST(xen_obj);
	xen_obj->sgt_imported = sgt;
	return &xen_obj->base;
}

int xendrm_gem_set_ext_sg_table(struct drm_gem_object *gem_obj,
	struct sg_table *sgt)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);
	int ret;

	DRM_ERROR("%s\n", __FUNCTION__);
	xen_obj->pages = drm_malloc_ab(xen_obj->num_pages,
		sizeof(struct page *));
	if (!xen_obj->pages)
		return -ENOMEM;
	ret = drm_prime_sg_to_page_addr_arrays(sgt, xen_obj->pages, NULL,
		xen_obj->num_pages);
	if (ret < 0) {
		drm_free_large(xen_obj->pages);
		xen_obj->pages = NULL;
		return ret;
	}
	xen_obj->be_alloc = true;
	return 0;
}

static struct xen_fb *xendrm_gem_fb_alloc(struct drm_device *dev,
	const struct drm_mode_fb_cmd2 *mode_cmd,
	struct xen_gem_object *xen_obj,
	const struct drm_framebuffer_funcs *funcs)
{
	struct xen_fb *xen_fb;
	int ret;

	xen_fb = kzalloc(sizeof(*xen_fb), GFP_KERNEL);
	if (!xen_fb)
		return ERR_PTR(-ENOMEM);

	drm_helper_mode_fill_fb_struct(&xen_fb->fb, mode_cmd);
	xen_fb->xen_obj = xen_obj;
	ret = drm_framebuffer_init(dev, &xen_fb->fb, funcs);
	if (ret < 0) {
		DRM_ERROR("Failed to initialize framebuffer: %d\n", ret);
		kfree(xen_fb);
		return ERR_PTR(ret);
	}
	return xen_fb;
}

struct drm_framebuffer *xendrm_gem_fb_create_with_funcs(struct drm_device *dev,
	struct drm_file *file_priv, const struct drm_mode_fb_cmd2 *mode_cmd,
	const struct drm_framebuffer_funcs *funcs)
{
	struct xen_fb *xen_fb;
	struct xen_gem_object *xen_obj;
	struct drm_gem_object *gem_obj;
	unsigned int hsub;
	unsigned int vsub;
	unsigned int min_size;
	int ret;

	/* we do not support formats that require more than 1 plane */
	if (drm_format_num_planes(mode_cmd->pixel_format) != 1) {
		DRM_ERROR("Unsupported pixel format 0x%04x\n",
			mode_cmd->pixel_format);
		return ERR_PTR(-EINVAL);
	}
	hsub = drm_format_horz_chroma_subsampling(mode_cmd->pixel_format);
	vsub = drm_format_vert_chroma_subsampling(mode_cmd->pixel_format);

	gem_obj = drm_gem_object_lookup(file_priv, mode_cmd->handles[0]);
	if (!gem_obj) {
		DRM_ERROR("Failed to lookup GEM object\n");
		return ERR_PTR(-ENXIO);
	}

	min_size = (mode_cmd->height - 1) * mode_cmd->pitches[0] +
		mode_cmd->width *
		drm_format_plane_cpp(mode_cmd->pixel_format, 0) +
		mode_cmd->offsets[0];
	if (gem_obj->size < min_size) {
		drm_gem_object_unreference_unlocked(gem_obj);
		return ERR_PTR(-EINVAL);
	}
	xen_obj = to_xen_gem_obj(gem_obj);

	xen_fb = xendrm_gem_fb_alloc(dev, mode_cmd, xen_obj, funcs);
	if (IS_ERR(xen_fb)) {
		ret = PTR_ERR(xen_fb);
		goto fail;
	}
	return &xen_fb->fb;

fail:
	drm_gem_object_unreference_unlocked(gem_obj);
	return ERR_PTR(ret);
}

void xendrm_gem_fb_destroy(struct drm_framebuffer *fb)
{
	struct xen_fb *xen_fb = to_xen_fb(fb);

	if (xen_fb->xen_obj)
		drm_gem_object_unreference_unlocked(&xen_fb->xen_obj->base);
	drm_framebuffer_cleanup(fb);
	kfree(xen_fb);
}

int xendrm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct drm_gem_object *gem_obj = vma->vm_private_data;
	struct xen_gem_object *xen_obj;
	pgoff_t pgoff;
	int ret;

	/* Make sure we don't parallel update on a fault, nor move or remove
	 * something from beneath our feet
	 */
	mutex_lock(&gem_obj->dev->struct_mutex);

	xen_obj = to_xen_gem_obj(vma->vm_private_data);

	if (!xen_obj->pages) {
		ret = VM_FAULT_SIGBUS;
		DRM_ERROR("%s ret %d No pages \n", __FUNCTION__, ret);
		goto fail;
	}

	/* we don't use vmf->pgoff since that has the fake offset */
	pgoff = ((unsigned long)vmf->virtual_address -
		vma->vm_start) >> PAGE_SHIFT;

	ret = vm_insert_page(vma, (unsigned long)vmf->virtual_address,
		xen_obj->pages[pgoff]);

fail:
	mutex_unlock(&gem_obj->dev->struct_mutex);
	DRM_ERROR("%s ret %d\n", __FUNCTION__, ret);
	switch (ret) {
	case 0:
	case -ERESTARTSYS:
	case -EINTR:
	case -EBUSY:
		/*
		 * EBUSY is ok: this just means that another thread
		 * already did the job.
		 */
		return VM_FAULT_NOPAGE;
	case -ENOMEM:
		return VM_FAULT_OOM;
	default:
		return VM_FAULT_SIGBUS;
	}
}

int xendrm_gem_dumb_map_offset(struct drm_file *file_priv,
	struct drm_device *dev, uint32_t handle, uint64_t *offset)
{
	struct drm_gem_object *gem_obj;
	struct xen_gem_object *xen_obj;
	int ret = 0;

	DRM_ERROR("%s\n", __FUNCTION__);
	gem_obj = drm_gem_object_lookup(file_priv, handle);
	if (!gem_obj) {
		DRM_ERROR("Failed to lookup GEM object\n");
		return -ENOENT;
	}
	DRM_ERROR("%s handle %d\n", __FUNCTION__, handle);
	xen_obj = to_xen_gem_obj(gem_obj);
	/* do not allow mapping of the imported buffers */
	if (xen_obj->base.import_attach) {
		ret = -EINVAL;
	} else {
		ret = drm_gem_create_mmap_offset(gem_obj);
		if (ret < 0)
			*offset = 0;
		else
			*offset = drm_vma_node_offset_addr(&gem_obj->vma_node);
		DRM_ERROR("%s gem_obj->size %zu ret %d\n", __FUNCTION__, gem_obj->size, ret);
	}
	drm_gem_object_unreference_unlocked(gem_obj);
	DRM_ERROR("%s ret %d\n", __FUNCTION__, ret);
	return ret;
}

static int xendrm_gem_mmap_obj(struct xen_gem_object *xen_obj,
	struct vm_area_struct *vma)
{
	DRM_ERROR("%s\n", __FUNCTION__);
	/*
	 * Clear the VM_PFNMAP flag that was set by drm_gem_mmap(), and set the
	 * vm_pgoff (used as a fake buffer offset by DRM) to 0 as we want to map
	 * the whole buffer.
	 */
	vma->vm_flags |= VM_MIXEDMAP;
	vma->vm_flags &= ~VM_PFNMAP;

	/*
	 * Shunt off cached objs to shmem file so they have their own
	 * address_space (so unmap_mapping_range does what we want,
	 * in particular in the case of mmap'd dmabufs)
	 */
	fput(vma->vm_file);

	vma->vm_pgoff = 0;
	vma->vm_file = get_file(xen_obj->base.filp);
	/* this is the only way to mmap for unprivileged domain */
	vma->vm_page_prot = PAGE_SHARED;
	return 0;
}

int xendrm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct xen_gem_object *xen_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	DRM_ERROR("%s\n", __FUNCTION__);
	ret = drm_gem_mmap(filp, vma);
	if (ret)
		return ret;
	gem_obj = vma->vm_private_data;
	xen_obj = to_xen_gem_obj(gem_obj);
	return xendrm_gem_mmap_obj(xen_obj, vma);
}

void *xendrm_gem_prime_vmap(struct drm_gem_object *gem_obj)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);

	DRM_ERROR("%s\n", __FUNCTION__);
	if (!xen_obj->pages)
		return NULL;
	return vmap(xen_obj->pages, xen_obj->num_pages,
		GFP_KERNEL, PAGE_SHARED);
}

void xendrm_gem_prime_vunmap(struct drm_gem_object *gem_obj, void *vaddr)
{
	DRM_ERROR("%s\n", __FUNCTION__);
	vunmap(vaddr);
}

int xendrm_gem_prime_mmap(struct drm_gem_object *gem_obj,
	struct vm_area_struct *vma)
{
	struct xen_gem_object *xen_obj;
	int ret;

	DRM_ERROR("%s\n", __FUNCTION__);
	ret = drm_gem_mmap_obj(gem_obj, gem_obj->size, vma);
	if (ret < 0)
		return ret;
	xen_obj = to_xen_gem_obj(gem_obj);
	return xendrm_gem_mmap_obj(xen_obj, vma);
}

bool xendrm_gem_is_still_used(struct drm_gem_object *gem_obj)
{
	struct xen_gem_object *xen_obj;
	int i;

	xen_obj = to_xen_gem_obj(gem_obj);
	if (!xen_obj->pages)
		return false;
	for (i = 0; i < xen_obj->num_pages; i++)
		if (xendrm_check_if_bad_page(xen_obj->pages[i]))
			return true;
	return false;
}
