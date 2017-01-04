/*
 * Xen para-virtual input device
 *
 * Copyright (C) 2005 Anthony Liguori <aliguori@us.ibm.com>
 * Copyright (C) 2006-2008 Red Hat, Inc., Markus Armbruster <armbru@redhat.com>
 *
 *  Based on linux/drivers/input/mouse/sermouse.c
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>

#include <asm/xen/hypervisor.h>

#include <xen/xen.h>
#include <xen/events.h>
#include <xen/page.h>
#include <xen/grant_table.h>
#include <xen/interface/grant_table.h>
#include <xen/interface/io/fbif.h>
#include <xen/interface/io/kbdif.h>
#include <xen/xenbus.h>
#include <xen/platform_pci.h>

struct xenkbd_ring_info {
	struct xenkbd_page *page;
	int gref;
	int irq;
};

struct xenkbd_mt_info {
	struct xenkbd_ring_info ring;
	struct input_dev *dev;
	int cur_slot;
};

struct xenkbd_info {
	struct input_dev *kbd;
	struct input_dev *ptr;
	struct xenkbd_ring_info ring;
	struct xenbus_device *xbdev;
	char phys[32];
	int mt_num_devices;
	/* array of multi-touch devices */
	struct xenkbd_mt_info *mt_devs;
};

static int xenkbd_remove(struct xenbus_device *);
static int xenkbd_connect_backend(struct xenbus_device *, struct xenkbd_info *);
static void xenkbd_disconnect_backend(struct xenkbd_info *);

/*
 * Note: if you need to send out events, see xenfb_do_update() for how
 * to do that.
 */

static struct xenkbd_page *xenkbd_ring_alloc(struct xenkbd_ring_info *ring)
{
	ring->irq = -1;
	ring->gref = -1;
	ring->page = (void *)__get_free_page(GFP_KERNEL | __GFP_ZERO);
	return ring->page;
}

static int xenkbd_ring_setup(struct xenbus_device *dev, void *dev_id,
			     irq_handler_t handler,
			     struct xenkbd_ring_info *ring,
			     const char *path)
{
	int ret, evtchn;
	struct xenbus_transaction xbt;

	ret = gnttab_grant_foreign_access(dev->otherend_id,
	                                  virt_to_gfn(ring->page), 0);
	if (ret < 0)
		return ret;
	ring->gref = ret;

	ret = xenbus_alloc_evtchn(dev, &evtchn);
	if (ret)
		goto error_grant;
	ret = bind_evtchn_to_irqhandler(evtchn, handler,
					0, dev->devicetype, dev_id);
	if (ret < 0) {
		xenbus_dev_fatal(dev, ret, "bind_evtchn_to_irqhandler");
		goto error_evtchan;
	}
	ring->irq = ret;

 again:
	ret = xenbus_transaction_start(&xbt);
	if (ret) {
		xenbus_dev_fatal(dev, ret, "starting transaction");
		goto error_irqh;
	}
	ret = xenbus_printf(xbt, path, XENKBD_FIELD_RING_REF,
			    "%lu", virt_to_gfn(ring->page));
	if (ret)
		goto error_xenbus;
	ret = xenbus_printf(xbt, path, XENKBD_FIELD_RING_GREF,
			    "%u", ring->gref);
	if (ret)
		goto error_xenbus;
	ret = xenbus_printf(xbt, path, XENKBD_FIELD_EVT_CHANNEL, "%u",
			    evtchn);
	if (ret)
		goto error_xenbus;
	ret = xenbus_transaction_end(xbt, 0);
	if (ret) {
		if (ret == -EAGAIN)
			goto again;
		xenbus_dev_fatal(dev, ret, "completing transaction");
		goto error_irqh;
	}

	return 0;

 error_xenbus:
	xenbus_transaction_end(xbt, 1);
	xenbus_dev_fatal(dev, ret, "writing xenstore");
 error_irqh:
	unbind_from_irqhandler(ring->irq, dev_id);
	ring->irq = -1;
 error_evtchan:
	xenbus_free_evtchn(dev, evtchn);
 error_grant:
	gnttab_end_foreign_access(ring->gref, 0, 0UL);
	ring->gref = -1;
	return ret;
}

static inline void xenkbd_ring_reset(struct xenkbd_ring_info *ring)
{
	memset(ring->page, 0, PAGE_SIZE);
}

static inline void xenkbd_ring_free(struct xenkbd_ring_info *ring)
{
	free_page((unsigned long)ring->page);
}

static void xenkbd_ring_cleanup(void *dev_id, struct xenkbd_ring_info *ring)
{
	if (ring->irq >= 0)
		unbind_from_irqhandler(ring->irq, dev_id);
	ring->irq = -1;
	if (ring->gref >= 0)
		gnttab_end_foreign_access(ring->gref, 0, 0UL);
	ring->gref = -1;
}

static irqreturn_t input_handler(int rq, void *dev_id)
{
	struct xenkbd_info *info = dev_id;
	struct xenkbd_page *page = info->ring.page;
	__u32 cons, prod;

	prod = page->in_prod;
	if (prod == page->in_cons)
		return IRQ_HANDLED;
	rmb();			/* ensure we see ring contents up to prod */
	for (cons = page->in_cons; cons != prod; cons++) {
		union xenkbd_in_event *event;
		struct input_dev *dev;
		event = &XENKBD_IN_RING_REF(page, cons);

		dev = info->ptr;
		switch (event->type) {
		case XENKBD_TYPE_MOTION:
			input_report_rel(dev, REL_X, event->motion.rel_x);
			input_report_rel(dev, REL_Y, event->motion.rel_y);
			if (event->motion.rel_z)
				input_report_rel(dev, REL_WHEEL,
						 -event->motion.rel_z);
			break;
		case XENKBD_TYPE_KEY:
			dev = NULL;
			if (test_bit(event->key.keycode, info->kbd->keybit))
				dev = info->kbd;
			if (test_bit(event->key.keycode, info->ptr->keybit))
				dev = info->ptr;
			if (dev)
				input_report_key(dev, event->key.keycode,
						 event->key.pressed);
			else
				pr_warning("unhandled keycode 0x%x\n",
					   event->key.keycode);
			break;
		case XENKBD_TYPE_POS:
			input_report_abs(dev, ABS_X, event->pos.abs_x);
			input_report_abs(dev, ABS_Y, event->pos.abs_y);
			if (event->pos.rel_z)
				input_report_rel(dev, REL_WHEEL,
						 -event->pos.rel_z);
			break;
		}
		if (dev)
			input_sync(dev);
	}
	mb();			/* ensure we got ring contents */
	page->in_cons = cons;
	notify_remote_via_irq(info->ring.irq);

	return IRQ_HANDLED;
}

static irqreturn_t mt_input_handler(int rq, void *dev_id)
{
	struct xenkbd_mt_info *info = dev_id;
	struct xenkbd_page *page = info->ring.page;
	__u32 cons, prod;

	prod = page->in_prod;
	if (prod == page->in_cons)
		return IRQ_HANDLED;
	rmb();			/* ensure we see ring contents up to prod */
	for (cons = page->in_cons; cons != prod; cons++) {
		struct input_dev *input = info->dev;
		struct xenkbd_mtouch *event =
			(struct xenkbd_mtouch *)&XENKBD_IN_RING_REF(page, cons);

		if (unlikely(event->type != XENKBD_TYPE_MTOUCH))
			continue;
		switch (event->event_type) {
		case XENKBD_MT_EV_DOWN:
			input_mt_report_slot_state(input, MT_TOOL_FINGER,
						   true);
			input_event(input, EV_ABS, ABS_MT_POSITION_X,
				    event->u.pos.abs_x);
			input_event(input, EV_ABS, ABS_MT_POSITION_Y,
				    event->u.pos.abs_y);
			input_event(input, EV_ABS, ABS_X,
				    event->u.pos.abs_x);
			input_event(input, EV_ABS, ABS_Y,
				    event->u.pos.abs_y);
			break;
		case XENKBD_MT_EV_UP:
			input_mt_report_slot_state(input, MT_TOOL_FINGER,
						   false);
			break;
		case XENKBD_MT_EV_MOTION:
			input_event(input, EV_ABS, ABS_MT_POSITION_X,
				    event->u.pos.abs_x);
			input_event(input, EV_ABS, ABS_MT_POSITION_Y,
				    event->u.pos.abs_y);
			input_event(input, EV_ABS, ABS_X,
				    event->u.pos.abs_x);
			input_event(input, EV_ABS, ABS_Y,
				    event->u.pos.abs_y);
			break;
		case XENKBD_MT_EV_SYN:
			input_mt_sync_frame(input);
			input_sync(input);
			break;
		case XENKBD_MT_EV_SHAPE:
			input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR,
				    event->u.shape.major);
			input_event(input, EV_ABS, ABS_MT_TOUCH_MINOR,
				    event->u.shape.minor);
			break;
		case XENKBD_MT_EV_ORIENT:
			input_event(input, EV_ABS, ABS_MT_ORIENTATION,
				    event->u.orientation);
			break;
		default:
			break;
		}
	}
	mb();			/* ensure we got ring contents */
	page->in_cons = cons;
	notify_remote_via_irq(info->ring.irq);

	return IRQ_HANDLED;
}

static int xenkbd_mt_get_num_devices(struct xenkbd_info *info)
{
	char **mt_nodes = NULL;
	int num_mt_devs;

	mt_nodes = xenbus_directory(XBT_NIL, info->xbdev->nodename,
				    XENKBD_PATH_MTOUCH, &num_mt_devs);
	if (IS_ERR(mt_nodes))
		num_mt_devs = 0;
	kfree(mt_nodes);
	return num_mt_devs;
}

static int xenkbd_mt_update_config(struct xenkbd_info *info)
{
	int num_contacts, width, height;
	int i, ret;
	char *node;

	ret = 0;
	for (i = 0; i < info->mt_num_devices; i++) {
		struct input_dev *dev;

		node = kasprintf(GFP_KERNEL, XENKBD_PATH_MTOUCH "/%d/"
				 XENKBD_FIELD_NUM_CONTACTS, i);
		if (!node)
			return -ENOMEM;
		num_contacts = xenbus_read_unsigned(info->xbdev->nodename,
						    node, 1);
		kfree(node);

		node = kasprintf(GFP_KERNEL, XENKBD_PATH_MTOUCH "/%d/"
				 XENKBD_FIELD_WIDTH, i);
		if (!node)
			return -ENOMEM;
		width = xenbus_read_unsigned(info->xbdev->nodename,
					     node, XENFB_WIDTH);
		kfree(node);

		node = kasprintf(GFP_KERNEL, XENKBD_FIELD_HEIGHT "/%d/"
				 XENKBD_FIELD_HEIGHT, i);
		if (!node)
			return -ENOMEM;
		height = xenbus_read_unsigned(info->xbdev->nodename,
					      node, XENFB_HEIGHT);
		kfree(node);

		dev = info->mt_devs[i].dev;

		input_set_abs_params(dev, ABS_X, 0, width, 0, 0);
		input_set_abs_params(dev, ABS_MT_POSITION_X, 0, width, 0, 0);

		input_set_abs_params(dev, ABS_Y, 0, height, 0, 0);
		input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, height, 0, 0);

		ret = input_mt_init_slots(dev, num_contacts, 0);
		if (ret < 0)
			break;
	}
	return ret;
}

static int xenkbd_mt_rings_setup(struct xenkbd_info *info)
{
	char *node;
	int i, ret;

	for (i = 0; i < info->mt_num_devices; i++) {
		node = kasprintf(GFP_KERNEL, "%s/" XENKBD_PATH_MTOUCH "/%d",
				 info->xbdev->nodename, i);
		if (!node)
			return -ENOMEM;
		ret = xenkbd_ring_setup(info->xbdev, &info->mt_devs[i],
					mt_input_handler,
					&info->mt_devs[i].ring,
					node);
		kfree(node);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static struct input_dev *xenkbd_mt_input_dev_register(struct xenkbd_info *info,
						      int index)
{
	struct input_dev *touch;
	int ret;

	touch = input_allocate_device();
	if (!touch)
		return NULL;
	touch->name = devm_kasprintf(&touch->dev, GFP_KERNEL,
				     "Xen Virtual Multi-touch %d", index);
	touch->phys = info->phys;
	touch->id.bustype = BUS_PCI;
	touch->id.vendor = 0x5853;
	touch->id.product = 0xfffd - index;

	__set_bit(EV_ABS, touch->evbit);
	__set_bit(EV_KEY, touch->evbit);
	__set_bit(BTN_TOUCH, touch->keybit);

	/*
	 * width, height and number of slots will be set on back's
	 * XenbusStateConnected
	 */
	input_set_abs_params(touch, ABS_X, 0, XENFB_WIDTH, 0, 0);
	input_set_abs_params(touch, ABS_Y, 0, XENFB_HEIGHT, 0, 0);
	input_set_abs_params(touch, ABS_PRESSURE, 0, 255, 0, 0);

	input_set_abs_params(touch, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(touch, ABS_MT_POSITION_X,
			     0, XENFB_WIDTH, 0, 0);
	input_set_abs_params(touch, ABS_MT_POSITION_Y,
			     0, XENFB_HEIGHT, 0, 0);
	input_set_abs_params(touch, ABS_MT_PRESSURE, 0, 255, 0, 0);

	ret = input_register_device(touch);
	if (ret < 0) {
		input_free_device(touch);
		xenbus_dev_fatal(info->xbdev, ret,
				 "input_register_device(touch)");
		return NULL;
	}
	return touch;
}

static int xenkbd_mt_probe(struct xenkbd_info *info)
{
	int num_devices, i;

	num_devices = xenkbd_mt_get_num_devices(info);
	if (!num_devices) {
		pr_warning("xenkbd: no multi-touch devices configured\n");
		return 0;
	}
	info->mt_devs = kcalloc(num_devices, sizeof(*info->mt_devs),
				GFP_KERNEL);
	if (!info->mt_devs) {
		xenbus_dev_fatal(info->xbdev, -ENOMEM,
				 "allocating device memory");
		return -ENOMEM;
	}
	info->mt_num_devices = num_devices;
	for (i = 0; i < num_devices; i++) {
		if (!xenkbd_ring_alloc(&info->mt_devs[i].ring))
			return -ENOMEM;
		info->mt_devs[i].cur_slot = -1;
	}
	for (i = 0; i < num_devices; i++) {
		struct input_dev *touch;

		touch = xenkbd_mt_input_dev_register(info, i);
		if (!touch)
			return -EFAULT;
		info->mt_devs[i].dev = touch;
	}
	return 0;
}

static int xenkbd_probe(struct xenbus_device *dev,
			const struct xenbus_device_id *id)
{
	int ret, i;
	unsigned int abs, mtouch;
	struct xenkbd_info *info;
	struct input_dev *kbd, *ptr;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		xenbus_dev_fatal(dev, -ENOMEM, "allocating info structure");
		return -ENOMEM;
	}
	dev_set_drvdata(&dev->dev, info);
	info->xbdev = dev;
	snprintf(info->phys, sizeof(info->phys), "xenbus/%s", dev->nodename);

	if (!xenkbd_ring_alloc(&info->ring))
		goto error_nomem;

	abs = xenbus_read_unsigned(dev->otherend,
				   XENKBD_FIELD_FEAT_ABS_POINTER, 0);
	if (abs) {
		ret = xenbus_write(XBT_NIL, dev->nodename,
				   XENKBD_FIELD_REQ_ABS_POINTER, "1");
		if (ret) {
			pr_warning("xenkbd: can't request abs-pointer");
			abs = 0;
		}
	}

	mtouch = xenbus_read_unsigned(dev->otherend,
				      XENKBD_FIELD_FEAT_MTOUCH, 0);
	if (mtouch) {
		ret = xenbus_write(XBT_NIL, dev->nodename,
				   XENKBD_FIELD_REQ_MTOUCH, "1");
		if (ret)
			pr_warning("xenkbd: can't request multi-touch");
	}

	/* keyboard */
	kbd = input_allocate_device();
	if (!kbd)
		goto error_nomem;
	kbd->name = "Xen Virtual Keyboard";
	kbd->phys = info->phys;
	kbd->id.bustype = BUS_PCI;
	kbd->id.vendor = 0x5853;
	kbd->id.product = 0xffff;

	__set_bit(EV_KEY, kbd->evbit);
	for (i = KEY_ESC; i < KEY_UNKNOWN; i++)
		__set_bit(i, kbd->keybit);
	for (i = KEY_OK; i < KEY_MAX; i++)
		__set_bit(i, kbd->keybit);

	ret = input_register_device(kbd);
	if (ret) {
		input_free_device(kbd);
		xenbus_dev_fatal(dev, ret, "input_register_device(kbd)");
		goto error;
	}
	info->kbd = kbd;

	/* pointing device */
	ptr = input_allocate_device();
	if (!ptr)
		goto error_nomem;
	ptr->name = "Xen Virtual Pointer";
	ptr->phys = info->phys;
	ptr->id.bustype = BUS_PCI;
	ptr->id.vendor = 0x5853;
	ptr->id.product = 0xfffe;

	if (abs) {
		__set_bit(EV_ABS, ptr->evbit);
		input_set_abs_params(ptr, ABS_X, 0, XENFB_WIDTH, 0, 0);
		input_set_abs_params(ptr, ABS_Y, 0, XENFB_HEIGHT, 0, 0);
	} else {
		input_set_capability(ptr, EV_REL, REL_X);
		input_set_capability(ptr, EV_REL, REL_Y);
	}
	input_set_capability(ptr, EV_REL, REL_WHEEL);

	__set_bit(EV_KEY, ptr->evbit);
	for (i = BTN_LEFT; i <= BTN_TASK; i++)
		__set_bit(i, ptr->keybit);

	ret = input_register_device(ptr);
	if (ret) {
		input_free_device(ptr);
		xenbus_dev_fatal(dev, ret, "input_register_device(ptr)");
		goto error;
	}
	info->ptr = ptr;

	ret = xenkbd_mt_probe(info);
	if (ret < 0)
		goto error;

	ret = xenkbd_connect_backend(dev, info);
	if (ret < 0)
		goto error;

	return 0;

 error_nomem:
	ret = -ENOMEM;
	xenbus_dev_fatal(dev, ret, "allocating device memory");
 error:
	xenkbd_remove(dev);
	return ret;
}

static int xenkbd_resume(struct xenbus_device *dev)
{
	struct xenkbd_info *info = dev_get_drvdata(&dev->dev);
	int i;

	xenkbd_disconnect_backend(info);
	xenkbd_ring_reset(&info->ring);
	for (i = 0; i < info->mt_num_devices; i++)
		xenkbd_ring_reset(&info->mt_devs[i].ring);
	return xenkbd_connect_backend(dev, info);
}

static int xenkbd_remove(struct xenbus_device *dev)
{
	struct xenkbd_info *info = dev_get_drvdata(&dev->dev);
	int i;

	xenkbd_disconnect_backend(info);
	if (info->kbd)
		input_unregister_device(info->kbd);
	if (info->ptr)
		input_unregister_device(info->ptr);
	xenkbd_ring_free(&info->ring);
	for (i = 0; i < info->mt_num_devices; i++) {
		if (info->mt_devs[i].dev)
			input_unregister_device(info->mt_devs[i].dev);
		xenkbd_ring_free(&info->mt_devs[i].ring);
	}
	kfree(info->mt_devs);
	kfree(info);
	return 0;
}

static int xenkbd_connect_backend(struct xenbus_device *dev,
				  struct xenkbd_info *info)
{
	int ret;

	ret = xenkbd_ring_setup(dev, info, input_handler, &info->ring,
				dev->nodename);
	if (ret < 0)
		return ret;

	ret = xenkbd_mt_rings_setup(info);

	if (ret < 0)
		return ret;

	xenbus_switch_state(dev, XenbusStateInitialised);
	return 0;
}

static void xenkbd_disconnect_backend(struct xenkbd_info *info)
{
	int i;

	xenkbd_ring_cleanup(info, &info->ring);
	for (i = 0; i < info->mt_num_devices; i++)
		xenkbd_ring_cleanup(&info->mt_devs[i],
				    &info->mt_devs[i].ring);
}

static void xenkbd_backend_changed(struct xenbus_device *dev,
				   enum xenbus_state backend_state)
{
	struct xenkbd_info *info = dev_get_drvdata(&dev->dev);
	int ret, val;

	switch (backend_state) {
	case XenbusStateInitialising:
	case XenbusStateInitialised:
	case XenbusStateReconfiguring:
	case XenbusStateReconfigured:
	case XenbusStateUnknown:
		break;

	case XenbusStateInitWait:
InitWait:
		if (xenbus_read_unsigned(info->xbdev->otherend,
					 XENKBD_FIELD_FEAT_ABS_POINTER, 0)) {
			ret = xenbus_write(XBT_NIL, info->xbdev->nodename,
					   XENKBD_FIELD_REQ_ABS_POINTER, "1");
			if (ret)
				pr_warning("xenkbd: can't request abs-pointer");
		}

		if (xenbus_read_unsigned(info->xbdev->otherend,
					 XENKBD_FIELD_FEAT_MTOUCH, 0)) {
			ret = xenbus_write(XBT_NIL, dev->nodename,
					   XENKBD_FIELD_REQ_MTOUCH, "1");
			if (ret)
				pr_warning("xenkbd: can't request multi-touch");
		}

		xenbus_switch_state(dev, XenbusStateConnected);
		break;

	case XenbusStateConnected:
		/*
		 * Work around xenbus race condition: If backend goes
		 * through InitWait to Connected fast enough, we can
		 * get Connected twice here.
		 */
		if (dev->state != XenbusStateConnected)
			goto InitWait; /* no InitWait seen yet, fudge it */

		/* Set input abs params to match backend screen res */
		if (xenbus_scanf(XBT_NIL, info->xbdev->otherend,
				 XENKBD_FIELD_WIDTH, "%d", &val) > 0)
			input_set_abs_params(info->ptr, ABS_X, 0, val, 0, 0);

		if (xenbus_scanf(XBT_NIL, info->xbdev->otherend,
				 XENKBD_FIELD_HEIGHT, "%d", &val) > 0)
			input_set_abs_params(info->ptr, ABS_Y, 0, val, 0, 0);

		ret = xenkbd_mt_update_config(info);
		if (ret < 0)
			xenbus_dev_fatal(dev, ret, "mtouch configure");

		break;

	case XenbusStateClosed:
		if (dev->state == XenbusStateClosed)
			break;
		/* Missed the backend's CLOSING state -- fallthrough */
	case XenbusStateClosing:
		xenbus_frontend_closed(dev);
		break;
	}
}

static const struct xenbus_device_id xenkbd_ids[] = {
	{ XENKBD_DRIVER_NAME },
	{ "" }
};

static struct xenbus_driver xenkbd_driver = {
	.ids = xenkbd_ids,
	.probe = xenkbd_probe,
	.remove = xenkbd_remove,
	.resume = xenkbd_resume,
	.otherend_changed = xenkbd_backend_changed,
};

static int __init xenkbd_init(void)
{
	if (!xen_domain())
		return -ENODEV;

	/* Nothing to do if running in dom0. */
	if (xen_initial_domain())
		return -ENODEV;

	if (!xen_has_pv_devices())
		return -ENODEV;

	return xenbus_register_frontend(&xenkbd_driver);
}

static void __exit xenkbd_cleanup(void)
{
	xenbus_unregister_driver(&xenkbd_driver);
}

module_init(xenkbd_init);
module_exit(xenkbd_cleanup);

MODULE_DESCRIPTION("Xen virtual keyboard/pointer device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:vkbd");
