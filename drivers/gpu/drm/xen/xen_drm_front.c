// SPDX-License-Identifier: GPL-2.0 OR MIT

/*
 *  Xen para-virtual DRM device
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

/* timeout in ms to wait for backend to respond */
#define VDRM_WAIT_BACK_MS	3000

struct xen_drm_front_dbuf {
	struct list_head list;
	uint64_t dbuf_cookie;
	uint64_t fb_cookie;
	struct xen_drm_front_shbuf *shbuf;
};

static int dbuf_add_to_list(struct xen_drm_front_info *front_info,
		struct xen_drm_front_shbuf *shbuf, uint64_t dbuf_cookie)
{
	struct xen_drm_front_dbuf *dbuf;

	dbuf = kzalloc(sizeof(*dbuf), GFP_KERNEL);
	if (!dbuf)
		return -ENOMEM;

	dbuf->dbuf_cookie = dbuf_cookie;
	dbuf->shbuf = shbuf;
	list_add(&dbuf->list, &front_info->dbuf_list);
	return 0;
}

static struct xen_drm_front_dbuf *dbuf_get(struct list_head *dbuf_list,
		uint64_t dbuf_cookie)
{
	struct xen_drm_front_dbuf *buf, *q;

	list_for_each_entry_safe(buf, q, dbuf_list, list)
		if (buf->dbuf_cookie == dbuf_cookie)
			return buf;

	return NULL;
}

static void dbuf_flush_fb(struct list_head *dbuf_list, uint64_t fb_cookie)
{
	struct xen_drm_front_dbuf *buf, *q;

	list_for_each_entry_safe(buf, q, dbuf_list, list)
		if (buf->fb_cookie == fb_cookie)
			xen_drm_front_shbuf_flush(buf->shbuf);
}

static void dbuf_free(struct list_head *dbuf_list, uint64_t dbuf_cookie)
{
	struct xen_drm_front_dbuf *buf, *q;

	list_for_each_entry_safe(buf, q, dbuf_list, list)
		if (buf->dbuf_cookie == dbuf_cookie) {
			list_del(&buf->list);
			xen_drm_front_shbuf_unmap(buf->shbuf);
			xen_drm_front_shbuf_free(buf->shbuf);
			kfree(buf);
			break;
		}
}

static void dbuf_free_all(struct list_head *dbuf_list)
{
	struct xen_drm_front_dbuf *buf, *q;

	list_for_each_entry_safe(buf, q, dbuf_list, list) {
		list_del(&buf->list);
		xen_drm_front_shbuf_unmap(buf->shbuf);
		xen_drm_front_shbuf_free(buf->shbuf);
		kfree(buf);
	}
}

static struct xendispl_req *be_prepare_req(
		struct xen_drm_front_evtchnl *evtchnl, uint8_t operation)
{
	struct xendispl_req *req;

	req = RING_GET_REQUEST(&evtchnl->u.req.ring,
			evtchnl->u.req.ring.req_prod_pvt);
	req->operation = operation;
	req->id = evtchnl->evt_next_id++;
	evtchnl->evt_id = req->id;
	return req;
}

static int be_stream_do_io(struct xen_drm_front_evtchnl *evtchnl,
		struct xendispl_req *req)
{
	reinit_completion(&evtchnl->u.req.completion);
	if (unlikely(evtchnl->state != EVTCHNL_STATE_CONNECTED))
		return -EIO;

	xen_drm_front_evtchnl_flush(evtchnl);
	return 0;
}

static int be_stream_wait_io(struct xen_drm_front_evtchnl *evtchnl)
{
	if (wait_for_completion_timeout(&evtchnl->u.req.completion,
			msecs_to_jiffies(VDRM_WAIT_BACK_MS)) <= 0)
		return -ETIMEDOUT;

	return evtchnl->u.req.resp_status;
}

static int be_mode_set(struct xen_drm_front_drm_pipeline *pipeline, uint32_t x,
		uint32_t y, uint32_t width, uint32_t height, uint32_t bpp,
		uint64_t fb_cookie)

{
	struct xen_drm_front_evtchnl *evtchnl;
	struct xen_drm_front_info *front_info;
	struct xendispl_req *req;
	unsigned long flags;
	int ret;

	front_info = pipeline->drm_info->front_info;
	evtchnl = &front_info->evt_pairs[pipeline->index].req;
	if (unlikely(!evtchnl))
		return -EIO;

	mutex_lock(&front_info->req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENDISPL_OP_SET_CONFIG);
	req->op.set_config.x = x;
	req->op.set_config.y = y;
	req->op.set_config.width = width;
	req->op.set_config.height = height;
	req->op.set_config.bpp = bpp;
	req->op.set_config.fb_cookie = fb_cookie;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	mutex_unlock(&front_info->req_io_lock);
	return ret;
}

static int be_dbuf_create_int(struct xen_drm_front_info *front_info,
		uint64_t dbuf_cookie, uint32_t width, uint32_t height,
		uint32_t bpp, uint64_t size, struct page **pages,
		struct sg_table *sgt)
{
	struct xen_drm_front_evtchnl *evtchnl;
	struct xen_drm_front_shbuf *shbuf;
	struct xendispl_req *req;
	struct xen_drm_front_shbuf_cfg buf_cfg;
	unsigned long flags;
	int ret;

	evtchnl = &front_info->evt_pairs[GENERIC_OP_EVT_CHNL].req;
	if (unlikely(!evtchnl))
		return -EIO;

	memset(&buf_cfg, 0, sizeof(buf_cfg));
	buf_cfg.xb_dev = front_info->xb_dev;
	buf_cfg.pages = pages;
	buf_cfg.size = size;
	buf_cfg.sgt = sgt;
	buf_cfg.be_alloc = front_info->cfg.be_alloc;

	shbuf = xen_drm_front_shbuf_alloc(&buf_cfg);
	if (!shbuf)
		return -ENOMEM;

	ret = dbuf_add_to_list(front_info, shbuf, dbuf_cookie);
	if (ret < 0) {
		xen_drm_front_shbuf_free(shbuf);
		return ret;
	}

	mutex_lock(&front_info->req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENDISPL_OP_DBUF_CREATE);
	req->op.dbuf_create.gref_directory =
			xen_drm_front_shbuf_get_dir_start(shbuf);
	req->op.dbuf_create.buffer_sz = size;
	req->op.dbuf_create.dbuf_cookie = dbuf_cookie;
	req->op.dbuf_create.width = width;
	req->op.dbuf_create.height = height;
	req->op.dbuf_create.bpp = bpp;
	if (buf_cfg.be_alloc)
		req->op.dbuf_create.flags |= XENDISPL_DBUF_FLG_REQ_ALLOC;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret < 0)
		goto fail;

	ret = be_stream_wait_io(evtchnl);
	if (ret < 0)
		goto fail;

	ret = xen_drm_front_shbuf_map(shbuf);
	if (ret < 0)
		goto fail;

	mutex_unlock(&front_info->req_io_lock);
	return 0;

fail:
	mutex_unlock(&front_info->req_io_lock);
	dbuf_free(&front_info->dbuf_list, dbuf_cookie);
	return ret;
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
	struct xen_drm_front_evtchnl *evtchnl;
	struct xendispl_req *req;
	unsigned long flags;
	bool be_alloc;
	int ret;

	evtchnl = &front_info->evt_pairs[GENERIC_OP_EVT_CHNL].req;
	if (unlikely(!evtchnl))
		return -EIO;

	be_alloc = front_info->cfg.be_alloc;

	/*
	 * for the backend allocated buffer release references now, so backend
	 * can free the buffer
	 */
	if (be_alloc)
		dbuf_free(&front_info->dbuf_list, dbuf_cookie);

	mutex_lock(&front_info->req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENDISPL_OP_DBUF_DESTROY);
	req->op.dbuf_destroy.dbuf_cookie = dbuf_cookie;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	/*
	 * do this regardless of communication status with the backend:
	 * if we cannot remove remote resources remove what we can locally
	 */
	if (!be_alloc)
		dbuf_free(&front_info->dbuf_list, dbuf_cookie);

	mutex_unlock(&front_info->req_io_lock);
	return ret;
}

static int be_fb_attach(struct xen_drm_front_info *front_info,
		uint64_t dbuf_cookie, uint64_t fb_cookie, uint32_t width,
		uint32_t height, uint32_t pixel_format)
{
	struct xen_drm_front_evtchnl *evtchnl;
	struct xen_drm_front_dbuf *buf;
	struct xendispl_req *req;
	unsigned long flags;
	int ret;

	evtchnl = &front_info->evt_pairs[GENERIC_OP_EVT_CHNL].req;
	if (unlikely(!evtchnl))
		return -EIO;

	buf = dbuf_get(&front_info->dbuf_list, dbuf_cookie);
	if (!buf)
		return -EINVAL;

	buf->fb_cookie = fb_cookie;

	mutex_lock(&front_info->req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENDISPL_OP_FB_ATTACH);
	req->op.fb_attach.dbuf_cookie = dbuf_cookie;
	req->op.fb_attach.fb_cookie = fb_cookie;
	req->op.fb_attach.width = width;
	req->op.fb_attach.height = height;
	req->op.fb_attach.pixel_format = pixel_format;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	mutex_unlock(&front_info->req_io_lock);
	return ret;
}

static int be_fb_detach(struct xen_drm_front_info *front_info,
		uint64_t fb_cookie)
{
	struct xen_drm_front_evtchnl *evtchnl;
	struct xendispl_req *req;
	unsigned long flags;
	int ret;

	evtchnl = &front_info->evt_pairs[GENERIC_OP_EVT_CHNL].req;
	if (unlikely(!evtchnl))
		return -EIO;

	mutex_lock(&front_info->req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENDISPL_OP_FB_DETACH);
	req->op.fb_detach.fb_cookie = fb_cookie;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	mutex_unlock(&front_info->req_io_lock);
	return ret;
}

static int be_page_flip(struct xen_drm_front_info *front_info, int conn_idx,
		uint64_t fb_cookie)
{
	struct xen_drm_front_evtchnl *evtchnl;
	struct xendispl_req *req;
	unsigned long flags;
	int ret;

	if (unlikely(conn_idx >= front_info->num_evt_pairs))
		return -EINVAL;

	dbuf_flush_fb(&front_info->dbuf_list, fb_cookie);
	evtchnl = &front_info->evt_pairs[conn_idx].req;

	mutex_lock(&front_info->req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENDISPL_OP_PG_FLIP);
	req->op.pg_flip.fb_cookie = fb_cookie;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	mutex_unlock(&front_info->req_io_lock);
	return ret;
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
	dbuf_free_all(&front_info->dbuf_list);
}

static int displback_initwait(struct xen_drm_front_info *front_info)
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

static int displback_connect(struct xen_drm_front_info *front_info)
{
	xen_drm_front_evtchnl_set_state(front_info, EVTCHNL_STATE_CONNECTED);
	return xen_drm_drv_init(front_info);
}

static void displback_disconnect(struct xen_drm_front_info *front_info)
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

static void displback_changed(struct xenbus_device *xb_dev,
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
		displback_disconnect(front_info);
		break;

	case XenbusStateInitWait:
		/* recovering after backend unexpected closure */
		displback_disconnect(front_info);
		if (xb_dev->state != XenbusStateInitialising)
			break;

		ret = displback_initwait(front_info);
		if (ret < 0)
			xenbus_dev_fatal(xb_dev, ret, "initializing frontend");
		else
			xenbus_switch_state(xb_dev, XenbusStateInitialised);
		break;

	case XenbusStateConnected:
		if (xb_dev->state != XenbusStateInitialised)
			break;

		ret = displback_connect(front_info);
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

		displback_disconnect(front_info);
		break;
	}
}

static int xen_drv_probe(struct xenbus_device *xb_dev,
		const struct xenbus_device_id *id)
{
	struct xen_drm_front_info *front_info;

	front_info = devm_kzalloc(&xb_dev->dev,
			sizeof(*front_info), GFP_KERNEL);
	if (!front_info)
		return -ENOMEM;

	front_info->xb_dev = xb_dev;
	spin_lock_init(&front_info->io_lock);
	mutex_init(&front_info->req_io_lock);
	INIT_LIST_HEAD(&front_info->dbuf_list);
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
	.otherend_changed = displback_changed,
};

static int __init xen_drv_init(void)
{
	/* At the moment we only support case with XEN_PAGE_SIZE == PAGE_SIZE */
	if (XEN_PAGE_SIZE != PAGE_SIZE) {
		DRM_ERROR(XENDISPL_DRIVER_NAME ": different kernel and Xen page sizes are not supported: XEN_PAGE_SIZE (%lu) != PAGE_SIZE (%lu)\n",
				XEN_PAGE_SIZE, PAGE_SIZE);
		return -ENODEV;
	}

	if (!xen_domain())
		return -ENODEV;

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
