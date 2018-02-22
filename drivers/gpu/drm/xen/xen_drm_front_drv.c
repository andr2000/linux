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
#include <drm/drm_atomic_helper.h>

#include "xen_drm_front.h"
#include "xen_drm_front_cfg.h"
#include "xen_drm_front_drv.h"
#include "xen_drm_front_gem.h"
#include "xen_drm_front_kms.h"

#include "xen_drm_front_backport.h"

static int dumb_create(struct drm_file *filp,
		struct drm_device *dev, struct drm_mode_create_dumb *args)
{
	struct xen_drm_front_drm_info *drm_info = dev->dev_private;
	struct drm_gem_object *obj;
	int ret;

	ret = drm_info->gem_ops->dumb_create(filp, dev, args);
	if (ret)
		goto fail;

	obj = drm_gem_object_lookup(filp, args->handle);
	if (!obj) {
		ret = -ENOENT;
		goto fail_destroy;
	}

	drm_gem_object_unreference_unlocked(obj);

	/*
	 * In case of CONFIG_DRM_XEN_FRONTEND_CMA gem_obj is constructed
	 * via DRM CMA helpers and doesn't have ->pages allocated
	 * (xendrm_gem_get_pages will return NULL), but instead can provide
	 * sg table
	 */
	if (drm_info->gem_ops->get_pages(obj))
		ret = drm_info->front_ops->dbuf_create_from_pages(
				drm_info->front_info,
				xen_drm_front_dbuf_to_cookie(obj),
				args->width, args->height, args->bpp,
				args->size,
				drm_info->gem_ops->get_pages(obj));
	else
		ret = drm_info->front_ops->dbuf_create_from_sgt(
				drm_info->front_info,
				xen_drm_front_dbuf_to_cookie(obj),
				args->width, args->height, args->bpp,
				args->size,
				drm_info->gem_ops->prime_get_sg_table(obj));
	if (ret)
		goto fail_destroy;

	return 0;

fail_destroy:
	drm_gem_dumb_destroy(filp, dev, args->handle);
fail:
	DRM_ERROR("Failed to create dumb buffer: %d\n", ret);
	return ret;
}

static void free_object(struct drm_gem_object *obj)
{
	struct xen_drm_front_drm_info *drm_info = obj->dev->dev_private;

	drm_info->front_ops->dbuf_destroy(drm_info->front_info,
			xen_drm_front_dbuf_to_cookie(obj));
	drm_info->gem_ops->free_object_unlocked(obj);
}

static void on_frame_done(struct platform_device *pdev,
		int conn_idx, uint64_t fb_cookie)
{
	struct xen_drm_front_drm_info *drm_info = platform_get_drvdata(pdev);

	if (unlikely(conn_idx >= drm_info->cfg->num_connectors))
		return;

	xen_drm_front_kms_on_frame_done(&drm_info->pipeline[conn_idx],
			fb_cookie);
}

static void lastclose(struct drm_device *dev)
{
	struct xen_drm_front_drm_info *drm_info = dev->dev_private;

	drm_info->front_ops->drm_last_close(drm_info->front_info);
}

static int gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct drm_file *file_priv = filp->private_data;
	struct drm_device *dev = file_priv->minor->dev;
	struct xen_drm_front_drm_info *drm_info = dev->dev_private;

	return drm_info->gem_ops->mmap(filp, vma);
}

static struct sg_table *prime_get_sg_table(struct drm_gem_object *obj)
{
	struct xen_drm_front_drm_info *drm_info;

	drm_info = obj->dev->dev_private;
	return drm_info->gem_ops->prime_get_sg_table(obj);
}

static struct drm_gem_object *prime_import_sg_table(struct drm_device *dev,
		struct dma_buf_attachment *attach, struct sg_table *sgt)
{
	struct xen_drm_front_drm_info *drm_info;

	drm_info = dev->dev_private;
	return drm_info->gem_ops->prime_import_sg_table(dev, attach, sgt);
}

static void *prime_vmap(struct drm_gem_object *obj)
{
	struct xen_drm_front_drm_info *drm_info;

	drm_info = obj->dev->dev_private;
	return drm_info->gem_ops->prime_vmap(obj);
}

static void prime_vunmap(struct drm_gem_object *obj, void *vaddr)
{
	struct xen_drm_front_drm_info *drm_info;

	drm_info = obj->dev->dev_private;
	drm_info->gem_ops->prime_vunmap(obj, vaddr);
}

static int prime_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	struct xen_drm_front_drm_info *drm_info;

	drm_info = obj->dev->dev_private;
	return drm_info->gem_ops->prime_mmap(obj, vma);
}

static const struct file_operations xendrm_fops = {
	.owner          = THIS_MODULE,
	.open           = drm_open,
	.release        = drm_release,
	.unlocked_ioctl = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = drm_compat_ioctl,
#endif
	.poll           = drm_poll,
	.read           = drm_read,
	.llseek         = no_llseek,
	.mmap           = gem_mmap,
};

static const struct vm_operations_struct xen_drm_vm_ops = {
	.open           = drm_gem_vm_open,
	.close          = drm_gem_vm_close,
};

struct drm_driver xen_drm_driver = {
	.driver_features           = DRIVER_GEM | DRIVER_MODESET |
				     DRIVER_PRIME | DRIVER_ATOMIC,
	.lastclose                 = lastclose,
	.gem_free_object_unlocked  = free_object,
	.gem_vm_ops                = &xen_drm_vm_ops,
	.prime_handle_to_fd        = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle        = drm_gem_prime_fd_to_handle,
	.gem_prime_import          = drm_gem_prime_import,
	.gem_prime_export          = drm_gem_prime_export,
	.gem_prime_get_sg_table    = prime_get_sg_table,
	.gem_prime_import_sg_table = prime_import_sg_table,
	.gem_prime_vmap            = prime_vmap,
	.gem_prime_vunmap          = prime_vunmap,
	.gem_prime_mmap            = prime_mmap,
	.dumb_create               = dumb_create,
	.fops                      = &xendrm_fops,
	.name                      = "xendrm-du",
	.desc                      = "Xen PV DRM Display Unit",
	.date                      = "20180221",
	.major                     = 1,
	.minor                     = 0,
#if LINUX_VERSION_CODE < PV_DRM_LINUX_VERSION
	.get_vblank_counter        = drm_vblank_no_hw_counter,
	.enable_vblank             = xen_drm_front_enable_vblank,
	.disable_vblank            = xen_drm_front_disable_vblank,
	.dumb_map_offset           = xen_drm_front_dumb_map_offset,
	.dumb_destroy              = drm_gem_dumb_destroy,
#endif
};

int xen_drm_front_drv_probe(struct platform_device *pdev,
		struct xen_drm_front_ops *front_ops)
{
	struct xen_drm_front_cfg *cfg = dev_get_platdata(&pdev->dev);
	struct xen_drm_front_drm_info *drm_info;
	struct drm_device *dev;
	int ret;

	DRM_INFO("Creating %s\n", xen_drm_driver.desc);

	drm_info = devm_kzalloc(&pdev->dev, sizeof(*drm_info), GFP_KERNEL);
	if (!drm_info)
		return -ENOMEM;

	drm_info->front_ops = front_ops;
	drm_info->front_ops->on_frame_done = on_frame_done;
	drm_info->gem_ops = xen_drm_front_gem_get_ops();
	drm_info->front_info = cfg->front_info;

	dev = drm_dev_alloc(&xen_drm_driver, &pdev->dev);
	if (!dev)
		return -ENOMEM;

	drm_info->drm_dev = dev;

	drm_info->cfg = cfg;
	dev->dev_private = drm_info;
	platform_set_drvdata(pdev, drm_info);

	ret = drm_vblank_init(dev, cfg->num_connectors);
	if (ret) {
		DRM_ERROR("Failed to initialize vblank, ret %d\n", ret);
		return ret;
	}

	ret = xen_drm_front_kms_init(drm_info);
	if (ret) {
		DRM_ERROR("Failed to initialize DRM/KMS, ret %d\n", ret);
		goto fail_modeset;
	}

	dev->irq_enabled = 1;

	ret = drm_dev_register(dev, 0);
	if (ret)
		goto fail_register;

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n",
			xen_drm_driver.name, xen_drm_driver.major,
			xen_drm_driver.minor, xen_drm_driver.patchlevel,
			xen_drm_driver.date, dev->primary->index);

	return 0;

fail_register:
	drm_dev_unregister(dev);
fail_modeset:
	drm_mode_config_cleanup(dev);
	return ret;
}

int xen_drm_front_drv_remove(struct platform_device *pdev)
{
	struct xen_drm_front_drm_info *drm_info = platform_get_drvdata(pdev);
	struct drm_device *dev = drm_info->drm_dev;

	if (dev) {
		drm_dev_unregister(dev);
		drm_atomic_helper_shutdown(dev);
		drm_mode_config_cleanup(dev);
		drm_dev_unref(dev);
	}
	return 0;
}

bool xen_drm_front_drv_is_used(struct platform_device *pdev)
{
	struct xen_drm_front_drm_info *drm_info = platform_get_drvdata(pdev);
	struct drm_device *dev;

	if (!drm_info)
		return false;

	dev = drm_info->drm_dev;
	if (!dev)
		return false;

	/*
	 * FIXME: the code below must be protected by drm_global_mutex,
	 * but it is not accessible to us. Anyways there is a race condition,
	 * but we will re-try.
	 */
	return dev->open_count != 0;
}
