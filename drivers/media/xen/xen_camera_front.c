// SPDX-License-Identifier: GPL-2.0 OR MIT

/*
 * Xen para-virtual camera device
 *
 * Copyright (C) 2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>

#include <xen/platform_pci.h>
#include <xen/xen.h>
#include <xen/xenbus.h>

#include "xen_camera_front.h"
#include "xen_camera_front_evtchnl.h"
#include "xen_camera_front_shbuf.h"
#include "xen_camera_front_v4l2.h"

struct xen_camera_front_cbuf {
	struct list_head list;
	/* This is V4L2 buffer index. */
	u8 index;
	struct xen_camera_front_shbuf *shbuf;
};

static int cbuf_add_to_list(struct xen_camera_front_info *front_info,
			    struct xen_camera_front_shbuf *shbuf,
			    u8 index)
{
	struct xen_camera_front_cbuf *cbuf;

	cbuf = kzalloc(sizeof(*cbuf), GFP_KERNEL);
	if (!cbuf)
		return -ENOMEM;

	cbuf->index = index;
	cbuf->shbuf = shbuf;
	list_add(&cbuf->list, &front_info->cbuf_list);
	return 0;
}

static struct xen_camera_front_cbuf *cbuf_get(struct list_head *cbuf_list,
					      u8 index)
{
	struct xen_camera_front_cbuf *buf, *q;

	list_for_each_entry_safe(buf, q, cbuf_list, list)
		if (buf->index == index)
			return buf;

	return NULL;
}

static void cbuf_free(struct list_head *cbuf_list, u8 index)
{
	struct xen_camera_front_cbuf *buf, *q;

	list_for_each_entry_safe(buf, q, cbuf_list, list)
		if (buf->index == index) {
			list_del(&buf->list);
			xen_camera_front_shbuf_unmap(buf->shbuf);
			xen_camera_front_shbuf_free(buf->shbuf);
			kfree(buf);
			break;
		}
}

static void cbuf_free_all(struct list_head *cbuf_list)
{
	struct xen_camera_front_cbuf *buf, *q;

	list_for_each_entry_safe(buf, q, cbuf_list, list) {
		list_del(&buf->list);
		xen_camera_front_shbuf_unmap(buf->shbuf);
		xen_camera_front_shbuf_free(buf->shbuf);
		kfree(buf);
	}
}

static struct xencamera_req *
be_prepare_req(struct xen_camera_front_evtchnl *evtchnl, u8 operation)
{
	struct xencamera_req *req;

	req = RING_GET_REQUEST(&evtchnl->u.req.ring,
			       evtchnl->u.req.ring.req_prod_pvt);
	req->operation = operation;
	req->id = evtchnl->evt_next_id++;
	evtchnl->evt_id = req->id;
	return req;
}

static int be_stream_do_io(struct xen_camera_front_evtchnl *evtchnl,
			   struct xencamera_req *req)
{
	reinit_completion(&evtchnl->u.req.completion);
	if (unlikely(evtchnl->state != EVTCHNL_STATE_CONNECTED))
		return -EIO;

	xen_camera_front_evtchnl_flush(evtchnl);
	return 0;
}

static int be_stream_wait_io(struct xen_camera_front_evtchnl *evtchnl)
{
	if (wait_for_completion_timeout(&evtchnl->u.req.completion,
			msecs_to_jiffies(XEN_CAMERA_FRONT_WAIT_BACK_MS)) <= 0)
		return -ETIMEDOUT;

	return evtchnl->u.req.resp_status;
}

int xen_camera_front_set_config(struct xen_camera_front_info *front_info,
				struct xencamera_config *cfg,
				struct xencamera_config *resp)
{
	struct xen_camera_front_evtchnl *evtchnl;
	struct xencamera_req *req;
	unsigned long flags;
	int ret;

	evtchnl = &front_info->evt_pair.req;
	if (unlikely(!evtchnl))
		return -EIO;

	mutex_lock(&evtchnl->u.req.req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENCAMERA_OP_CONFIG_SET);
	req->req.config = *cfg;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	*resp = evtchnl->u.req.resp.resp.config;

	mutex_unlock(&evtchnl->u.req.req_io_lock);
	return ret;
}

int xen_camera_front_get_config(struct xen_camera_front_info *front_info,
				struct xencamera_config *resp)
{
	struct xen_camera_front_evtchnl *evtchnl;
	struct xencamera_req *req;
	unsigned long flags;
	int ret;

	evtchnl = &front_info->evt_pair.req;
	if (unlikely(!evtchnl))
		return -EIO;

	mutex_lock(&evtchnl->u.req.req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENCAMERA_OP_CONFIG_GET);

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	*resp = evtchnl->u.req.resp.resp.config;

	mutex_unlock(&evtchnl->u.req.req_io_lock);
	return ret;
}

int xen_camera_front_set_control(struct xen_camera_front_info *front_info,
				 int v4l2_cid, s64 value)
{
	struct xen_camera_front_evtchnl *evtchnl;
	struct xencamera_req *req;
	unsigned long flags;
	int ret, xen_type;

	evtchnl = &front_info->evt_pair.req;
	if (unlikely(!evtchnl))
		return -EIO;

	xen_type = xen_camera_front_v4l2_to_xen_type(v4l2_cid);
	if (xen_type < 0)
		return xen_type;

	mutex_lock(&evtchnl->u.req.req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENCAMERA_OP_CTRL_SET);

	req->req.ctrl_value.type = xen_type;
	req->req.ctrl_value.value = value;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	mutex_unlock(&evtchnl->u.req.req_io_lock);
	return ret;
}

int xen_camera_front_get_control(struct xen_camera_front_info *front_info,
				 int v4l2_cid, s64 *value)
{
	struct xen_camera_front_evtchnl *evtchnl;
	struct xencamera_req *req;
	unsigned long flags;
	int ret, xen_type;

	evtchnl = &front_info->evt_pair.req;
	if (unlikely(!evtchnl))
		return -EIO;

	xen_type = xen_camera_front_v4l2_to_xen_type(v4l2_cid);
	if (xen_type < 0)
		return xen_type;

	mutex_lock(&evtchnl->u.req.req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENCAMERA_OP_CTRL_GET);

	req->req.get_ctrl.type = xen_type;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	*value = evtchnl->u.req.resp.resp.ctrl_value.value;

	mutex_unlock(&evtchnl->u.req.req_io_lock);
	return ret;
}

static int be_enum_control(struct xen_camera_front_info *front_info, int index,
			   struct xencamera_ctrl_enum_resp *resp)
{
	struct xen_camera_front_evtchnl *evtchnl;
	struct xencamera_req *req;
	unsigned long flags;
	int ret;

	evtchnl = &front_info->evt_pair.req;
	if (unlikely(!evtchnl))
		return -EIO;

	mutex_lock(&evtchnl->u.req.req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENCAMERA_OP_CTRL_ENUM);
	req->req.index.index = index;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	*resp = evtchnl->u.req.resp.resp.ctrl_enum;

	mutex_unlock(&evtchnl->u.req.req_io_lock);
	return ret;
}

static int be_enum_controls(struct xen_camera_front_info *front_info)
{
	struct xen_camera_front_cfg_card *cfg = &front_info->cfg;
	struct xencamera_ctrl_enum_resp resp;
	int i, ret;

	cfg->num_controls = 0;
	for (i = 0; i < XENCAMERA_MAX_CTRL; i++) {
		ret = be_enum_control(front_info, i, &resp);
		/*
		 * We enumerate assigned controls here until EINVAL is
		 * returned by the backend meaning that the requested control
		 * with that index is not supported/assigned to the frontend.
		 */
		if (ret == -EINVAL)
			break;

		if (ret < 0)
			return ret;

		ret = xen_camera_front_v4l2_to_v4l2_cid(resp.type);
		if (ret < 0)
			return -EINVAL;

		cfg->ctrl[i].v4l2_cid = ret;
		cfg->ctrl[i].flags = resp.flags;
		cfg->ctrl[i].minimum = resp.min;
		cfg->ctrl[i].maximum = resp.max;
		cfg->ctrl[i].default_value = resp.def_val;
		cfg->ctrl[i].step = resp.step;

		dev_info(&front_info->xb_dev->dev, "Control CID %x\n", ret);

		cfg->num_controls++;
	}

	dev_info(&front_info->xb_dev->dev, "Assigned %d control(s)\n",
		 cfg->num_controls);
	return 0;
}

int xen_camera_front_buf_request(struct xen_camera_front_info *front_info,
				 int num_bufs)
{
	struct xen_camera_front_evtchnl *evtchnl;
	struct xencamera_req *req;
	unsigned long flags;
	int ret;

	evtchnl = &front_info->evt_pair.req;
	if (unlikely(!evtchnl))
		return -EIO;

	mutex_lock(&evtchnl->u.req.req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENCAMERA_OP_BUF_REQUEST);

	req->req.buf_request.num_bufs = num_bufs;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	num_bufs = evtchnl->u.req.resp.resp.buf_request.num_bufs;

	mutex_unlock(&evtchnl->u.req.req_io_lock);
	if (ret < 0)
		return ret;

	return num_bufs;
}

int xen_camera_front_buf_create(struct xen_camera_front_info *front_info,
				u8 index, u64 size, struct sg_table *sgt)
{
	struct xen_camera_front_evtchnl *evtchnl;
	struct xen_camera_front_shbuf *shbuf;
	struct xencamera_req *req;
	struct xen_camera_front_shbuf_cfg buf_cfg;
	unsigned long flags;
	int ret;

	evtchnl = &front_info->evt_pair.req;
	if (unlikely(!evtchnl))
		return -EIO;

	memset(&buf_cfg, 0, sizeof(buf_cfg));
	buf_cfg.xb_dev = front_info->xb_dev;
	buf_cfg.sgt = sgt;
	buf_cfg.size = size;
	buf_cfg.be_alloc = front_info->cfg.be_alloc;

	shbuf = xen_camera_front_shbuf_alloc(&buf_cfg);
	if (IS_ERR(shbuf))
		return PTR_ERR(shbuf);

	ret = cbuf_add_to_list(front_info, shbuf, index);
	if (ret < 0) {
		xen_camera_front_shbuf_free(shbuf);
		return ret;
	}

	mutex_lock(&evtchnl->u.req.req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENCAMERA_OP_BUF_CREATE);
	req->req.buf_create.gref_directory =
		xen_camera_front_shbuf_get_dir_start(shbuf);
	req->req.buf_create.index = index;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret < 0)
		goto fail;

	ret = be_stream_wait_io(evtchnl);
	if (ret < 0)
		goto fail;

	ret = xen_camera_front_shbuf_map(shbuf);
	if (ret < 0)
		goto fail;

	mutex_unlock(&evtchnl->u.req.req_io_lock);
	return 0;

fail:
	mutex_unlock(&evtchnl->u.req.req_io_lock);
	cbuf_free(&front_info->cbuf_list, index);
	return ret;
}

int xen_camera_front_buf_destroy(struct xen_camera_front_info *front_info,
				 u8 index)
{
	struct xen_camera_front_evtchnl *evtchnl;
	struct xencamera_req *req;
	unsigned long flags;
	bool be_alloc;
	int ret;

	evtchnl = &front_info->evt_pair.req;
	if (unlikely(!evtchnl))
		return -EIO;

	be_alloc = front_info->cfg.be_alloc;

	/*
	 * For the backend allocated buffer release references now, so backend
	 * can free the buffer.
	 */
	if (be_alloc)
		cbuf_free(&front_info->cbuf_list, index);

	mutex_lock(&evtchnl->u.req.req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENCAMERA_OP_BUF_DESTROY);
	req->req.index.index = index;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	/*
	 * Do this regardless of communication status with the backend:
	 * if we cannot remove remote resources remove what we can locally.
	 */
	if (!be_alloc)
		cbuf_free(&front_info->cbuf_list, index);

	mutex_unlock(&evtchnl->u.req.req_io_lock);
	return ret;
}

static int buf_queue_helper(struct xen_camera_front_info *front_info,
			    int index, u8 op)
{
	struct xen_camera_front_evtchnl *evtchnl;
	struct xencamera_req *req;
	unsigned long flags;
	int ret;

	evtchnl = &front_info->evt_pair.req;
	if (unlikely(!evtchnl))
		return -EIO;

	mutex_lock(&evtchnl->u.req.req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, op);

	req->req.index.index = index;

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	mutex_unlock(&evtchnl->u.req.req_io_lock);
	return ret;
}

int xen_camera_front_buf_queue(struct xen_camera_front_info *front_info,
			       int index)
{
	return buf_queue_helper(front_info, index, XENCAMERA_OP_BUF_QUEUE);
}

int xen_camera_front_buf_dequeue(struct xen_camera_front_info *front_info,
				 int index)
{
	return buf_queue_helper(front_info, index, XENCAMERA_OP_BUF_DEQUEUE);
}

int xen_camera_front_get_buf_layout(struct xen_camera_front_info *front_info,
				    struct xencamera_buf_get_layout_resp *resp)
{
	struct xen_camera_front_evtchnl *evtchnl;
	struct xencamera_req *req;
	unsigned long flags;
	int ret;

	evtchnl = &front_info->evt_pair.req;
	if (unlikely(!evtchnl))
		return -EIO;

	mutex_lock(&evtchnl->u.req.req_io_lock);

	spin_lock_irqsave(&front_info->io_lock, flags);
	req = be_prepare_req(evtchnl, XENCAMERA_OP_BUF_GET_LAYOUT);

	ret = be_stream_do_io(evtchnl, req);
	spin_unlock_irqrestore(&front_info->io_lock, flags);

	if (ret == 0)
		ret = be_stream_wait_io(evtchnl);

	*resp = evtchnl->u.req.resp.resp.buf_layout;

	mutex_unlock(&evtchnl->u.req.req_io_lock);
	return ret;
}

static void xen_camera_drv_fini(struct xen_camera_front_info *front_info)
{
	xen_camera_front_v4l2_fini(front_info);
	xen_camera_front_evtchnl_free_all(front_info);
}

static int cameraback_initwait(struct xen_camera_front_info *front_info)
{
	int ret;

	ret = xen_camera_front_cfg_init(front_info);
	if (ret < 0)
		return ret;

	/* Create all event channels and publish. */
	ret = xen_camera_front_evtchnl_create_all(front_info);
	if (ret < 0)
		return ret;

	return xen_camera_front_evtchnl_publish_all(front_info);
}

static int cameraback_connect(struct xen_camera_front_info *front_info)
{
	int ret;

	xen_camera_front_evtchnl_pair_set_connected(&front_info->evt_pair,
						    true);
	/*
	 * Event channels are all set now, so we can read detailed
	 * configuration for each assigned control.
	 */
	ret = be_enum_controls(front_info);
	if (ret < 0)
		return ret;

	return xen_camera_front_v4l2_init(front_info);
}

static void cameraback_disconnect(struct xen_camera_front_info *front_info)
{
	xen_camera_drv_fini(front_info);
	xenbus_switch_state(front_info->xb_dev, XenbusStateInitialising);
}

static void cameraback_changed(struct xenbus_device *xb_dev,
			    enum xenbus_state backend_state)
{
	struct xen_camera_front_info *front_info = dev_get_drvdata(&xb_dev->dev);
	int ret;

	dev_dbg(&xb_dev->dev, "Backend state is %s, front is %s\n",
		xenbus_strstate(backend_state),
		xenbus_strstate(xb_dev->state));

	switch (backend_state) {
	case XenbusStateReconfiguring:
		/* fall through */
	case XenbusStateReconfigured:
		/* fall through */
	case XenbusStateInitialised:
		/* fall through */
		break;

	case XenbusStateInitialising:
		/* Recovering after backend unexpected closure. */
		cameraback_disconnect(front_info);
		break;

	case XenbusStateInitWait:
		/* Recovering after backend unexpected closure. */
		cameraback_disconnect(front_info);

		ret = cameraback_initwait(front_info);
		if (ret < 0)
			xenbus_dev_fatal(xb_dev, ret, "initializing frontend");
		else
			xenbus_switch_state(xb_dev, XenbusStateInitialised);

		break;

	case XenbusStateConnected:
		if (xb_dev->state != XenbusStateInitialised)
			break;

		ret = cameraback_connect(front_info);
		if (ret < 0)
			xenbus_dev_fatal(xb_dev, ret, "initializing frontend");
		else
			xenbus_switch_state(xb_dev, XenbusStateConnected);
		break;

	case XenbusStateClosing:
		/*
		 * In this state backend starts freeing resources,
		 * so let it go into closed state first, so we can also
		 * remove ours.
		 */
		break;

	case XenbusStateUnknown:
		/* fall through */
	case XenbusStateClosed:
		if (xb_dev->state == XenbusStateClosed)
			break;

		cameraback_disconnect(front_info);
		break;

	default:
		break;
	}
}

static int xen_drv_probe(struct xenbus_device *xb_dev,
			 const struct xenbus_device_id *id)
{
	struct device *dev = &xb_dev->dev;
	struct xen_camera_front_info *front_info;
	int ret;

	/*
	 * The device is not spawn from a device tree, so arch_setup_dma_ops
	 * is not called, thus leaving the device with dummy DMA ops.
	 * This makes the device return error on PRIME buffer import, which
	 * is not correct: to fix this call of_dma_configure() with a NULL
	 * node to set default DMA ops.
	 */
	dev->coherent_dma_mask = DMA_BIT_MASK(32);
	ret = of_dma_configure(dev, NULL, true);
	if (ret < 0) {
		xenbus_dev_fatal(xb_dev, ret, "setting up DMA ops");
		return ret;
	}

	front_info = devm_kzalloc(&xb_dev->dev,
				  sizeof(*front_info), GFP_KERNEL);
	if (!front_info)
		return -ENOMEM;

	front_info->xb_dev = xb_dev;
	spin_lock_init(&front_info->io_lock);
	INIT_LIST_HEAD(&front_info->cbuf_list);
	dev_set_drvdata(&xb_dev->dev, front_info);

	return xenbus_switch_state(xb_dev, XenbusStateInitialising);
}

static int xen_drv_remove(struct xenbus_device *dev)
{
	struct xen_camera_front_info *front_info = dev_get_drvdata(&dev->dev);
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
	while ((xenbus_read_unsigned(front_info->xb_dev->otherend, "state",
				     XenbusStateUnknown) != XenbusStateInitWait) &&
				     --to)
		msleep(10);

	if (!to) {
		unsigned int state;

		state = xenbus_read_unsigned(front_info->xb_dev->otherend,
					     "state", XenbusStateUnknown);
		pr_err("Backend state is %s while removing driver\n",
		       xenbus_strstate(state));
	}

	xen_camera_drv_fini(front_info);
	xenbus_frontend_closed(dev);
	return 0;
}

static const struct xenbus_device_id xen_drv_ids[] = {
	{ XENCAMERA_DRIVER_NAME },
	{ "" }
};

static struct xenbus_driver xen_driver = {
	.ids = xen_drv_ids,
	.probe = xen_drv_probe,
	.remove = xen_drv_remove,
	.otherend_changed = cameraback_changed,
};

static int __init xen_drv_init(void)
{
	if (!xen_domain())
		return -ENODEV;

	if (!xen_has_pv_devices())
		return -ENODEV;

	pr_info("Initialising Xen " XENCAMERA_DRIVER_NAME " frontend driver\n");
	return xenbus_register_frontend(&xen_driver);
}

static void __exit xen_drv_fini(void)
{
	pr_info("Unregistering Xen " XENCAMERA_DRIVER_NAME " frontend driver\n");
	xenbus_unregister_driver(&xen_driver);
}

module_init(xen_drv_init);
module_exit(xen_drv_fini);

MODULE_DESCRIPTION("Xen virtual camera device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:" XENCAMERA_DRIVER_NAME);
