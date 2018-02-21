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

#include <drm/drmP.h>

#include <linux/of_device.h>

#include <xen/platform_pci.h>
#include <xen/xen.h>
#include <xen/xenbus.h>

#include <xen/interface/io/displif.h>

#include "xen_drm_front.h"
#include "xen_drm_front_drv.h"
#include "xen_drm_front_evtchnl.h"
#include "xen_drm_front_shbuf.h"

static int be_mode_set(struct xen_drm_front_drm_pipeline *pipeline, uint32_t x,
		uint32_t y, uint32_t width, uint32_t height, uint32_t bpp,
		uint64_t fb_cookie)

{
	return 0;
}

static int be_dbuf_create_int(struct xen_drm_front_info *front_info,
		uint64_t dbuf_cookie, uint32_t width, uint32_t height,
		uint32_t bpp, uint64_t size, struct page **pages,
		struct sg_table *sgt)
{
	return 0;
}

static int be_dbuf_create_from_sgt(struct xen_drm_front_info *front_info,
		uint64_t dbuf_cookie, uint32_t width, uint32_t height,
		uint32_t bpp, uint64_t size, struct sg_table *sgt)
{
	return be_dbuf_create_int(front_info, dbuf_cookie, width, height,
			bpp, size, NULL, sgt);
}

static int be_dbuf_create_from_pages(struct xen_drm_front_info *front_info,
		uint64_t dbuf_cookie, uint32_t width, uint32_t height,
		uint32_t bpp, uint64_t size, struct page **pages)
{
	return be_dbuf_create_int(front_info, dbuf_cookie, width, height,
			bpp, size, pages, NULL);
}

static int be_dbuf_destroy(struct xen_drm_front_info *front_info,
		uint64_t dbuf_cookie)
{
	return 0;
}

static int be_fb_attach(struct xen_drm_front_info *front_info,
		uint64_t dbuf_cookie, uint64_t fb_cookie, uint32_t width,
		uint32_t height, uint32_t pixel_format)
{
	return 0;
}

static int be_fb_detach(struct xen_drm_front_info *front_info,
		uint64_t fb_cookie)
{
	return 0;
}

static int be_page_flip(struct xen_drm_front_info *front_info, int conn_idx,
		uint64_t fb_cookie)
{
	return 0;
}

static void xen_drm_drv_unload(struct xen_drm_front_info *front_info)
{
	if (front_info->xb_dev->state != XenbusStateReconfiguring)
		return;

	DRM_DEBUG("Can try removing driver now\n");
	xenbus_switch_state(front_info->xb_dev, XenbusStateInitialising);
}

static struct xen_drm_front_ops front_ops = {
	.mode_set = be_mode_set,
	.dbuf_create_from_pages = be_dbuf_create_from_pages,
	.dbuf_create_from_sgt = be_dbuf_create_from_sgt,
	.dbuf_destroy = be_dbuf_destroy,
	.fb_attach = be_fb_attach,
	.fb_detach = be_fb_detach,
	.page_flip = be_page_flip,
	.drm_last_close = xen_drm_drv_unload,
};

static int xen_drm_drv_probe(struct platform_device *pdev)
{
	/*
	 * The device is not spawn from a device tree, so arch_setup_dma_ops
	 * is not called, thus leaving the device with dummy DMA ops.
	 * This makes the device return error on PRIME buffer import, which
	 * is not correct: to fix this call of_dma_configure() with a NULL
	 * node to set default DMA ops.
	 */
	of_dma_configure(&pdev->dev, NULL);
	return xen_drm_front_drv_probe(pdev, &front_ops);
}

static int xen_drm_drv_remove(struct platform_device *pdev)
{
	return xen_drm_front_drv_remove(pdev);
}

struct platform_device_info xen_drm_front_platform_info = {
	.name = XENDISPL_DRIVER_NAME,
	.id = 0,
	.num_res = 0,
	.dma_mask = DMA_BIT_MASK(32),
};

static struct platform_driver xen_drm_front_front_info = {
	.probe		= xen_drm_drv_probe,
	.remove		= xen_drm_drv_remove,
	.driver		= {
		.name	= XENDISPL_DRIVER_NAME,
	},
};

static void xen_drm_drv_deinit(struct xen_drm_front_info *front_info)
{
	if (!front_info->drm_pdrv_registered)
		return;

	if (front_info->drm_pdev)
		platform_device_unregister(front_info->drm_pdev);

	platform_driver_unregister(&xen_drm_front_front_info);
	front_info->drm_pdrv_registered = false;
	front_info->drm_pdev = NULL;
}

static int xen_drm_drv_init(struct xen_drm_front_info *front_info)
{
	int ret;

	ret = platform_driver_register(&xen_drm_front_front_info);
	if (ret < 0)
		return ret;

	front_info->drm_pdrv_registered = true;
	/* pass card configuration via platform data */
	xen_drm_front_platform_info.data = &front_info->cfg;
	xen_drm_front_platform_info.size_data = sizeof(front_info->cfg);

	front_info->drm_pdev = platform_device_register_full(
			&xen_drm_front_platform_info);
	if (IS_ERR_OR_NULL(front_info->drm_pdev)) {
		DRM_ERROR("Failed to register " XENDISPL_DRIVER_NAME " PV DRM driver\n");
		front_info->drm_pdev = NULL;
		xen_drm_drv_deinit(front_info);
		return -ENODEV;
	}

	return 0;
}

static void xen_drv_remove_internal(struct xen_drm_front_info *front_info)
{
	xen_drm_drv_deinit(front_info);
	xen_drm_front_evtchnl_free_all(front_info);
}

static int backend_on_initwait(struct xen_drm_front_info *front_info)
{
	struct xen_drm_front_cfg *cfg = &front_info->cfg;
	int ret;

	cfg->front_info = front_info;
	ret = xen_drm_front_cfg_card(front_info, cfg);
	if (ret < 0)
		return ret;

	DRM_INFO("Have %d conector(s)\n", cfg->num_connectors);
	/* Create event channels for all connectors and publish */
	ret = xen_drm_front_evtchnl_create_all(front_info, &front_ops);
	if (ret < 0)
		return ret;

	return xen_drm_front_evtchnl_publish_all(front_info);
}

static int backend_on_connected(struct xen_drm_front_info *front_info)
{
	xen_drm_front_evtchnl_set_state(front_info, EVTCHNL_STATE_CONNECTED);
	return xen_drm_drv_init(front_info);
}

static void backend_on_disconnected(struct xen_drm_front_info *front_info)
{
	bool removed = true;

	if (front_info->drm_pdev) {
		if (xen_drm_front_drv_is_used(front_info->drm_pdev)) {
			DRM_WARN("DRM driver still in use, deferring removal\n");
			removed = false;
		} else
			xen_drv_remove_internal(front_info);
	}

	xen_drm_front_evtchnl_set_state(front_info, EVTCHNL_STATE_DISCONNECTED);

	if (removed)
		xenbus_switch_state(front_info->xb_dev, XenbusStateInitialising);
	else
		xenbus_switch_state(front_info->xb_dev, XenbusStateReconfiguring);
}

static void backend_on_changed(struct xenbus_device *xb_dev,
		enum xenbus_state backend_state)
{
	struct xen_drm_front_info *front_info = dev_get_drvdata(&xb_dev->dev);
	int ret;

	DRM_DEBUG("Backend state is %s, front is %s\n",
			xenbus_strstate(backend_state),
			xenbus_strstate(xb_dev->state));

	switch (backend_state) {
	case XenbusStateReconfiguring:
		/* fall through */
	case XenbusStateReconfigured:
		/* fall through */
	case XenbusStateInitialised:
		break;

	case XenbusStateInitialising:
		/* recovering after backend unexpected closure */
		backend_on_disconnected(front_info);
		break;

	case XenbusStateInitWait:
		/* recovering after backend unexpected closure */
		backend_on_disconnected(front_info);
		if (xb_dev->state != XenbusStateInitialising)
			break;

		ret = backend_on_initwait(front_info);
		if (ret < 0)
			xenbus_dev_fatal(xb_dev, ret, "initializing frontend");
		else
			xenbus_switch_state(xb_dev, XenbusStateInitialised);
		break;

	case XenbusStateConnected:
		if (xb_dev->state != XenbusStateInitialised)
			break;

		ret = backend_on_connected(front_info);
		if (ret < 0)
			xenbus_dev_fatal(xb_dev, ret, "initializing DRM driver");
		else
			xenbus_switch_state(xb_dev, XenbusStateConnected);
		break;

	case XenbusStateClosing:
		/*
		 * in this state backend starts freeing resources,
		 * so let it go into closed state, so we can also
		 * remove ours
		 */
		break;

	case XenbusStateUnknown:
		/* fall through */
	case XenbusStateClosed:
		if (xb_dev->state == XenbusStateClosed)
			break;

		backend_on_disconnected(front_info);
		break;
	}
}

static int xen_drv_probe(struct xenbus_device *xb_dev,
		const struct xenbus_device_id *id)
{
	struct xen_drm_front_info *front_info;

	front_info = devm_kzalloc(&xb_dev->dev,
			sizeof(*front_info), GFP_KERNEL);
	if (!front_info) {
		xenbus_dev_fatal(xb_dev, -ENOMEM, "allocating device memory");
		return -ENOMEM;
	}

	front_info->xb_dev = xb_dev;
	spin_lock_init(&front_info->io_lock);
	front_info->drm_pdrv_registered = false;
	dev_set_drvdata(&xb_dev->dev, front_info);
	return xenbus_switch_state(xb_dev, XenbusStateInitialising);
}

static int xen_drv_remove(struct xenbus_device *dev)
{
	struct xen_drm_front_info *front_info = dev_get_drvdata(&dev->dev);
	int to = 100;

	xenbus_switch_state(dev, XenbusStateClosing);

	/*
	 * On driver removal it is disconnected from XenBus,
	 * so no backend state change events come via .otherend_changed
	 * callback. This prevents us from exiting gracefully, e.g.
	 * signaling the backend to free event channels, waiting for its
	 * state to change to XenbusStateClosed and cleaning at our end.
	 * Normally when front driver removed backend will finally go into
	 * XenbusStateInitWait state.
	 *
	 * Workaround: read backend's state manually and wait with time-out.
	 */
	while ((xenbus_read_unsigned(front_info->xb_dev->otherend,
			"state", XenbusStateUnknown) != XenbusStateInitWait) &&
			to--)
		msleep(10);

	if (!to)
		DRM_ERROR("Backend state is %s while removing driver\n",
			xenbus_strstate(xenbus_read_unsigned(
					front_info->xb_dev->otherend,
					"state", XenbusStateUnknown)));

	xen_drv_remove_internal(front_info);
	xenbus_frontend_closed(dev);
	return 0;
}

static const struct xenbus_device_id xen_drv_ids[] = {
	{ XENDISPL_DRIVER_NAME },
	{ "" }
};

static struct xenbus_driver xen_driver = {
	.ids = xen_drv_ids,
	.probe = xen_drv_probe,
	.remove = xen_drv_remove,
	.otherend_changed = backend_on_changed,
};

static int __init xen_drv_init(void)
{
	/* At the moment we only support case with XEN_PAGE_SIZE == PAGE_SIZE */
	BUILD_BUG_ON(XEN_PAGE_SIZE != PAGE_SIZE);

	if (!xen_domain())
		return -ENODEV;

	if (xen_initial_domain()) {
		DRM_ERROR(XENDISPL_DRIVER_NAME " cannot run in initial domain\n");
		return -ENODEV;
	}

	if (!xen_has_pv_devices())
		return -ENODEV;

	DRM_INFO("Registering XEN PV " XENDISPL_DRIVER_NAME "\n");
	return xenbus_register_frontend(&xen_driver);
}

static void __exit xen_drv_cleanup(void)
{
	DRM_INFO("Unregistering XEN PV " XENDISPL_DRIVER_NAME "\n");
	xenbus_unregister_driver(&xen_driver);
}

module_init(xen_drv_init);
module_exit(xen_drv_cleanup);

MODULE_DESCRIPTION("Xen para-virtualized display device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:"XENDISPL_DRIVER_NAME);
