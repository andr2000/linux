// SPDX-License-Identifier: GPL-2.0 OR MIT

/*
 *  Xen para-virtual DRM device
 *
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_cma_helper.h>

#include "xen_drm_front.h"
#include "xen_drm_front_cfg.h"
#include "xen_drm_front_drv.h"
#include "xen_drm_front_gem.h"
#include "xen_drm_front_kms.h"

static int dumb_create(struct drm_file *filp, struct drm_device *dev,
		struct drm_mode_create_dumb *args)
{
	struct xen_drm_front_drm_info *drm_info = dev->dev_private;
	struct drm_gem_object *obj;
	int ret;

	ret = xen_drm_front_gem_dumb_create(filp, dev, args);
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
	if (xen_drm_front_gem_get_pages(obj))
		ret = xen_drm_front_dbuf_create_from_pages(
				drm_info->front_info,
				xen_drm_front_dbuf_to_cookie(obj),
				args->width, args->height, args->bpp,
				args->size,
				xen_drm_front_gem_get_pages(obj));
	else
		ret = xen_drm_front_dbuf_create_from_sgt(
				drm_info->front_info,
				xen_drm_front_dbuf_to_cookie(obj),
				args->width, args->height, args->bpp,
				args->size,
				xen_drm_front_gem_get_sg_table(obj));
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

	xen_drm_front_dbuf_destroy(drm_info->front_info,
			xen_drm_front_dbuf_to_cookie(obj));
	xen_drm_front_gem_free_object(obj);
}

void xen_drm_front_on_frame_done(struct platform_device *pdev,
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

	xen_drm_front_unload(drm_info->front_info);
}

static const struct file_operations xen_drm_fops = {
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
#ifdef CONFIG_DRM_XEN_FRONTEND_CMA
	.mmap           = drm_gem_cma_mmap,
#else
	.mmap           = xen_drm_front_gem_mmap,
#endif
};

static const struct vm_operations_struct xen_drm_vm_ops = {
	.open           = drm_gem_vm_open,
	.close          = drm_gem_vm_close,
};

struct drm_driver xen_drm_driver = {
	.driver_features           = DRIVER_GEM | DRIVER_MODESET |
				     DRIVER_PRIME | DRIVER_ATOMIC,
	.lastclose                 = lastclose,
	.gem_vm_ops                = &xen_drm_vm_ops,
	.gem_free_object_unlocked  = free_object,
	.prime_handle_to_fd        = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle        = drm_gem_prime_fd_to_handle,
	.gem_prime_import          = drm_gem_prime_import,
	.gem_prime_export          = drm_gem_prime_export,
	.gem_prime_import_sg_table = xen_drm_front_gem_import_sg_table,
	.gem_prime_get_sg_table    = xen_drm_front_gem_get_sg_table,
	.dumb_create               = dumb_create,
	.fops                      = &xen_drm_fops,
	.name                      = "xendrm-du",
	.desc                      = "Xen PV DRM Display Unit",
	.date                      = "20180221",
	.major                     = 1,
	.minor                     = 0,

#ifdef CONFIG_DRM_XEN_FRONTEND_CMA
	.gem_prime_vmap            = drm_gem_cma_prime_vmap,
	.gem_prime_vunmap          = drm_gem_cma_prime_vunmap,
	.gem_prime_mmap            = drm_gem_cma_prime_mmap,
#else
	.gem_prime_vmap            = xen_drm_front_gem_prime_vmap,
	.gem_prime_vunmap          = xen_drm_front_gem_prime_vunmap,
	.gem_prime_mmap            = xen_drm_front_gem_prime_mmap,
#endif
};

int xen_drm_front_drv_probe(struct platform_device *pdev)
{
	struct xen_drm_front_cfg *cfg = dev_get_platdata(&pdev->dev);
	struct xen_drm_front_drm_info *drm_info;
	struct drm_device *dev;
	int ret;

	DRM_INFO("Creating %s\n", xen_drm_driver.desc);

	drm_info = devm_kzalloc(&pdev->dev, sizeof(*drm_info), GFP_KERNEL);
	if (!drm_info)
		return -ENOMEM;

	drm_info->front_info = cfg->front_info;

	dev = drm_dev_alloc(&xen_drm_driver, &pdev->dev);
	if (!dev)
		return -ENOMEM;

	drm_info->drm_dev = dev;

	drm_info->cfg = cfg;
	dev->dev_private = drm_info;
	platform_set_drvdata(pdev, drm_info);

	ret = xen_drm_front_kms_init(drm_info);
	if (ret) {
		DRM_ERROR("Failed to initialize DRM/KMS, ret %d\n", ret);
		goto fail_modeset;
	}

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
	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);
	return ret;
}

int xen_drm_front_drv_remove(struct platform_device *pdev)
{
	struct xen_drm_front_drm_info *drm_info = platform_get_drvdata(pdev);
	struct drm_device *dev = drm_info->drm_dev;

	if (dev) {
		drm_kms_helper_poll_fini(dev);
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
