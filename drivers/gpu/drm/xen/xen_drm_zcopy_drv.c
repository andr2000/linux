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

#include <drm/xen_zcopy_drm.h>

struct xen_drv_info {
	struct drm_device *drm_dev;
};

static void gem_free_object(struct drm_gem_object *gem_obj)
{
}

static struct sg_table *gem_prime_get_sg_table(
		struct drm_gem_object *gem_obj)
{
	return NULL;
}

struct drm_gem_object *gem_prime_import_sg_table(struct drm_device *dev,
		struct dma_buf_attachment *attach, struct sg_table *sgt)
{
	return NULL;
}

static int ioctl_from_refs(struct drm_device *dev,
		void *data, struct drm_file *filp)
{
	return -EINVAL;
}

static int ioctl_to_refs(struct drm_device *dev,
		void *data, struct drm_file *filp)
{
	return -EINVAL;
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
