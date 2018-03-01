// SPDX-License-Identifier: GPL-2.0 OR MIT

/*
 *  Xen zero-copy helper DRM device
 *
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#include <drm/drmP.h>
#include <drm/drm_gem.h>

#include <linux/dma-buf.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <xen/grant_table.h>

#include <drm/xen_zcopy_drm.h>

struct xen_gem_object {
	struct drm_gem_object base;
	uint32_t dumb_handle;

	int otherend_id;

	uint32_t num_pages;
	grant_ref_t *grefs;
	/* these are the pages for allocated Xen GEM object */
	struct page **pages;
	/* this will be set if we have imported a PRIME buffer */
	struct sg_table *sgt;
};

struct xen_drv_info {
	struct drm_device *drm_dev;
};

static inline struct xen_gem_object *to_xen_gem_obj(
		struct drm_gem_object *gem_obj)
{
	return container_of(gem_obj, struct xen_gem_object, base);
}

static int gem_create_with_handle(struct xen_gem_object *xen_obj,
		struct drm_file *filp, struct drm_device *dev, int size)
{
	struct drm_gem_object *gem_obj;
	int ret;

	drm_gem_private_object_init(dev, &xen_obj->base, size);
	gem_obj = &xen_obj->base;
	ret = drm_gem_handle_create(filp, gem_obj, &xen_obj->dumb_handle);
	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_unreference_unlocked(gem_obj);
	return ret;
}

static int gem_create_obj(struct xen_gem_object *xen_obj,
		struct drm_device *dev, struct drm_file *filp, int size)
{
	struct drm_gem_object *gem_obj;
	int ret;

	ret = gem_create_with_handle(xen_obj, filp, dev, size);
	if (ret < 0)
		goto fail;

	gem_obj = drm_gem_object_lookup(filp, xen_obj->dumb_handle);
	if (!gem_obj) {
		DRM_ERROR("Lookup for handle %d failed\n",
				xen_obj->dumb_handle);
		ret = -EINVAL;
		goto fail_destroy;
	}

	drm_gem_object_unreference_unlocked(gem_obj);
	return 0;

fail_destroy:
	drm_gem_dumb_destroy(filp, dev, xen_obj->dumb_handle);
fail:
	DRM_ERROR("Failed to create dumb buffer: %d\n", ret);
	xen_obj->dumb_handle = 0;
	return ret;
}

static int gem_init_obj(struct xen_gem_object *xen_obj,
	struct drm_device *dev, int size)
{
	struct drm_gem_object *gem_obj = &xen_obj->base;
	int ret;

	ret = drm_gem_object_init(dev, gem_obj, size);
	if (ret < 0)
		return ret;

	ret = drm_gem_create_mmap_offset(gem_obj);
	if (ret < 0) {
		drm_gem_object_release(gem_obj);
		return ret;
	}

	return 0;
}

static void gem_free_object(struct drm_gem_object *gem_obj)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);

	DRM_DEBUG("Freeing dumb with handle %d\n", xen_obj->dumb_handle);
	if (xen_obj->grefs)
		if (xen_obj->sgt) {
			if (xen_obj->base.import_attach)
				drm_prime_gem_destroy(&xen_obj->base,
						xen_obj->sgt);
		}

	drm_gem_object_release(gem_obj);
	kfree(xen_obj);
}

static struct sg_table *gem_prime_get_sg_table(
		struct drm_gem_object *gem_obj)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);
	struct sg_table *sgt = NULL;

	if (unlikely(!xen_obj->pages))
		return NULL;

	sgt = drm_prime_pages_to_sg(xen_obj->pages, xen_obj->num_pages);

	if (unlikely(!sgt))
		DRM_ERROR("Failed to export sgt\n");
	else
		DRM_DEBUG("Exporting %scontiguous buffer nents %d\n",
				sgt->nents == 1 ? "" : "non-", sgt->nents);
	return sgt;
}

struct drm_gem_object *gem_prime_import_sg_table(struct drm_device *dev,
		struct dma_buf_attachment *attach, struct sg_table *sgt)
{
	struct xen_gem_object *xen_obj;
	int ret;

	xen_obj = kzalloc(sizeof(*xen_obj), GFP_KERNEL);
	if (!xen_obj)
		return ERR_PTR(-ENOMEM);

	ret = gem_init_obj(xen_obj, dev, attach->dmabuf->size);
	if (ret < 0)
		goto fail;

	xen_obj->sgt = sgt;
	xen_obj->num_pages = DIV_ROUND_UP(attach->dmabuf->size, PAGE_SIZE);
	DRM_DEBUG("Imported buffer of size %zu with nents %u\n",
			attach->dmabuf->size, sgt->nents);
	return &xen_obj->base;

fail:
	kfree(xen_obj);
	return ERR_PTR(ret);
}

static int do_ioctl_from_refs(struct drm_device *dev,
		struct drm_xen_zcopy_dumb_from_refs *req,
		struct drm_file *filp)
{
	struct xen_gem_object *xen_obj;
	int ret;

	xen_obj = kzalloc(sizeof(*xen_obj), GFP_KERNEL);
	if (!xen_obj)
		return -ENOMEM;

	xen_obj->num_pages = req->num_grefs;
	xen_obj->otherend_id = req->otherend_id;
	xen_obj->grefs = kcalloc(xen_obj->num_pages, sizeof(grant_ref_t),
			GFP_KERNEL);
	if (!xen_obj->grefs) {
		ret = -ENOMEM;
		goto fail;
	}

	if (copy_from_user(xen_obj->grefs, req->grefs,
			xen_obj->num_pages * sizeof(grant_ref_t))) {
		ret = -EINVAL;
		goto fail;
	}

	/* do nothing with grefs at the moment */

	ret = gem_create_obj(xen_obj, dev, filp,
			round_up(req->dumb.size, PAGE_SIZE));
	if (ret < 0)
		goto fail;

	req->dumb.handle = xen_obj->dumb_handle;

	return 0;

fail:
	kfree(xen_obj->grefs);
	xen_obj->grefs = NULL;
	return ret;
}

static int ioctl_from_refs(struct drm_device *dev,
		void *data, struct drm_file *filp)
{
	struct drm_xen_zcopy_dumb_from_refs *req =
			(struct drm_xen_zcopy_dumb_from_refs *)data;
	struct drm_mode_create_dumb *args = &req->dumb;
	uint32_t cpp, stride, size;

	if (!req->num_grefs || !req->grefs)
		return -EINVAL;

	if (!args->width || !args->height || !args->bpp)
		return -EINVAL;

	cpp = DIV_ROUND_UP(args->bpp, 8);
	if (!cpp || cpp > 0xffffffffU / args->width)
		return -EINVAL;

	stride = cpp * args->width;
	if (args->height > 0xffffffffU / stride)
		return -EINVAL;

	size = args->height * stride;
	if (PAGE_ALIGN(size) == 0)
		return -EINVAL;

	args->pitch = DIV_ROUND_UP(args->width * args->bpp, 8);
	args->size = args->pitch * args->height;
	args->handle = 0;
	if (req->num_grefs < DIV_ROUND_UP(args->size, PAGE_SIZE)) {
		DRM_ERROR("Provided %d pages, need %d\n", req->num_grefs,
				(int)DIV_ROUND_UP(args->size, PAGE_SIZE));
		return -EINVAL;
	}

	return do_ioctl_from_refs(dev, req, filp);
}

static int ioctl_to_refs(struct drm_device *dev,
		void *data, struct drm_file *filp)
{
	struct xen_gem_object *xen_obj;
	struct drm_gem_object *gem_obj;
	struct drm_xen_zcopy_dumb_to_refs *req =
			(struct drm_xen_zcopy_dumb_to_refs *)data;
	int ret;

	if (!req->num_grefs || !req->grefs)
		return -EINVAL;

	gem_obj = drm_gem_object_lookup(filp, req->handle);
	if (!gem_obj) {
		DRM_ERROR("Lookup for handle %d failed\n", req->handle);
		return -EINVAL;
	}

	drm_gem_object_unreference_unlocked(gem_obj);
	xen_obj = to_xen_gem_obj(gem_obj);

	if (xen_obj->num_pages != req->num_grefs) {
		DRM_ERROR("Provided %d pages, need %d\n", req->num_grefs,
				xen_obj->num_pages);
		return -EINVAL;
	}

	xen_obj->otherend_id = req->otherend_id;
	xen_obj->grefs = kcalloc(xen_obj->num_pages, sizeof(grant_ref_t),
			GFP_KERNEL);
	if (!xen_obj->grefs) {
		ret = -ENOMEM;
		goto fail;
	}

	/* do nothing with grefs at the moment */

	if (copy_to_user(req->grefs, xen_obj->grefs,
			xen_obj->num_pages * sizeof(grant_ref_t))) {
		ret = -EINVAL;
		goto fail;
	}

	return 0;

fail:
	kfree(xen_obj->grefs);
	xen_obj->grefs = NULL;
	return ret;
}

static const struct drm_ioctl_desc xen_drm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(XEN_ZCOPY_DUMB_FROM_REFS,
		ioctl_from_refs,
		DRM_AUTH | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(XEN_ZCOPY_DUMB_TO_REFS,
		ioctl_to_refs,
		DRM_AUTH | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
};

static const struct file_operations xen_drm_fops = {
	.owner          = THIS_MODULE,
	.open           = drm_open,
	.release        = drm_release,
	.unlocked_ioctl = drm_ioctl,
};

static struct drm_driver xen_drm_driver = {
	.driver_features           = DRIVER_GEM | DRIVER_PRIME,
	.prime_handle_to_fd        = drm_gem_prime_handle_to_fd,
	.gem_prime_export          = drm_gem_prime_export,
	.gem_prime_get_sg_table    = gem_prime_get_sg_table,
	.prime_fd_to_handle        = drm_gem_prime_fd_to_handle,
	.gem_prime_import          = drm_gem_prime_import,
	.gem_prime_import_sg_table = gem_prime_import_sg_table,
	.gem_free_object_unlocked  = gem_free_object,
	.fops                      = &xen_drm_fops,
	.ioctls                    = xen_drm_ioctls,
	.num_ioctls                = ARRAY_SIZE(xen_drm_ioctls),
	.name                      = XENDRM_ZCOPY_DRIVER_NAME,
	.desc                      = "Xen PV DRM zero copy",
	.date                      = "20180221",
	.major                     = 1,
	.minor                     = 0,
};

static int xen_drm_drv_remove(struct platform_device *pdev)
{
	struct xen_drv_info *drv_info = platform_get_drvdata(pdev);

	if (drv_info && drv_info->drm_dev) {
		drm_dev_unregister(drv_info->drm_dev);
		drm_dev_unref(drv_info->drm_dev);
	}
	return 0;
}

static int xen_drm_drv_probe(struct platform_device *pdev)
{
	struct xen_drv_info *drv_info;
	int ret;

	DRM_INFO("Creating %s\n", xen_drm_driver.desc);
	drv_info = kzalloc(sizeof(*drv_info), GFP_KERNEL);
	if (!drv_info)
		return -ENOMEM;

	/*
	 * The device is not spawn from a device tree, so arch_setup_dma_ops
	 * is not called, thus leaving the device with dummy DMA ops.
	 * This makes the device return error on PRIME buffer import, which
	 * is not correct: to fix this call of_dma_configure() with a NULL
	 * node to set default DMA ops.
	 */
	of_dma_configure(&pdev->dev, NULL);

	drv_info->drm_dev = drm_dev_alloc(&xen_drm_driver, &pdev->dev);
	if (!drv_info->drm_dev)
		return -ENOMEM;

	ret = drm_dev_register(drv_info->drm_dev, 0);
	if (ret < 0)
		goto fail;

	drv_info->drm_dev->dev_private = drv_info;
	platform_set_drvdata(pdev, drv_info);

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n",
			xen_drm_driver.name, xen_drm_driver.major,
			xen_drm_driver.minor, xen_drm_driver.patchlevel,
			xen_drm_driver.date, drv_info->drm_dev->primary->index);
	return 0;

fail:
	drm_dev_unref(drv_info->drm_dev);
	kfree(drv_info);
	return ret;
}

static struct platform_driver xen_drm_zcopy_platform_drv_info = {
	.probe		= xen_drm_drv_probe,
	.remove		= xen_drm_drv_remove,
	.driver		= {
		.name	= XENDRM_ZCOPY_DRIVER_NAME,
	},
};

struct platform_device_info xen_drm_zcopy_dev_info = {
	.name = XENDRM_ZCOPY_DRIVER_NAME,
	.id = 0,
	.num_res = 0,
	.dma_mask = DMA_BIT_MASK(32),
};

static struct platform_device *xen_pdev;

static int __init xen_drv_init(void)
{
	int ret;

	if (!xen_domain())
		return -ENODEV;

	xen_pdev = platform_device_register_full(&xen_drm_zcopy_dev_info);
	if (!xen_pdev) {
		DRM_ERROR("Failed to register " XENDRM_ZCOPY_DRIVER_NAME " device\n");
		return -ENODEV;
	}

	ret = platform_driver_register(&xen_drm_zcopy_platform_drv_info);
	if (ret != 0) {
		DRM_ERROR("Failed to register " XENDRM_ZCOPY_DRIVER_NAME " driver: %d\n", ret);
		platform_device_unregister(xen_pdev);
		return ret;
	}

	return 0;
}

static void __exit xen_drv_cleanup(void)
{
	if (xen_pdev)
		platform_device_unregister(xen_pdev);
	platform_driver_unregister(&xen_drm_zcopy_platform_drv_info);
}

module_init(xen_drv_init);
module_exit(xen_drv_cleanup);

MODULE_DESCRIPTION("Xen zero-copy helper DRM device");
MODULE_LICENSE("GPL");
