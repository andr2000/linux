/*
 * Xen para-virtual sound device
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
 * Copyright (C) 2016-2017 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#include <xen/events.h>
#include <xen/grant_table.h>
#include <xen/xen.h>
#include <xen/xenbus.h>

#include "xen_front.h"
#include "xen_front_alsa.h"
#include "xen_front_cfg.h"
#include "xen_front_evtchnl.h"

static irqreturn_t evtchnl_interrupt_req(int irq, void *dev_id)
{
	struct xen_front_evtchnl_info *channel = dev_id;
	struct drv_info *drv_info = channel->drv_info;
	struct xensnd_resp *resp;
	RING_IDX i, rp;
	unsigned long flags;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	if (unlikely(channel->state != EVTCHNL_STATE_CONNECTED))
		goto out;

again:
	rp = channel->u.req.ring.sring->rsp_prod;
	/* ensure we see queued responses up to rp */
	rmb();

	for (i = channel->u.req.ring.rsp_cons; i != rp; i++) {
		resp = RING_GET_RESPONSE(&channel->u.req.ring, i);
		if (resp->id != channel->evt_id)
			continue;
		switch (resp->operation) {
		case XENSND_OP_OPEN:
			/* fall through */
		case XENSND_OP_CLOSE:
			/* fall through */
		case XENSND_OP_READ:
			/* fall through */
		case XENSND_OP_WRITE:
			/* fall through */
		case XENSND_OP_TRIGGER:
			channel->u.req.resp_status = resp->status;
			complete(&channel->u.req.completion);
			break;

		default:
			dev_err(&drv_info->xb_dev->dev,
				"Operation %d is not supported\n",
				resp->operation);
			break;
		}
	}

	channel->u.req.ring.rsp_cons = i;
	if (i != channel->u.req.ring.req_prod_pvt) {
		int more_to_do;

		RING_FINAL_CHECK_FOR_RESPONSES(&channel->u.req.ring,
			more_to_do);
		if (more_to_do)
			goto again;
	} else
		channel->u.req.ring.sring->rsp_event = i + 1;

out:
	spin_unlock_irqrestore(&drv_info->io_lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t evtchnl_interrupt_evt(int irq, void *dev_id)
{
	struct xen_front_evtchnl_info *channel = dev_id;
	struct drv_info *drv_info = channel->drv_info;
	struct xensnd_event_page *page = channel->u.evt.page;
	uint32_t cons, prod;
	unsigned long flags;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	if (unlikely(channel->state != EVTCHNL_STATE_CONNECTED))
		goto out;

	prod = page->in_prod;
	/* ensure we see ring contents up to prod */
	virt_rmb();
	if (prod == page->in_cons)
		goto out;

	for (cons = page->in_cons; cons != prod; cons++) {
		struct xensnd_evt *event;

		event = &XENSND_IN_RING_REF(page, cons);
		if (unlikely(event->id != channel->evt_id++))
			continue;

		switch (event->type) {
		case XENSND_EVT_CUR_POS:
			spin_unlock_irqrestore(&drv_info->io_lock, flags);
			xen_front_alsa_handle_cur_pos(channel,
				event->op.cur_pos.position);
			spin_lock_irqsave(&drv_info->io_lock, flags);
			break;
		}
	}
	page->in_cons = cons;
	/* ensure ring contents */
	virt_wmb();
out:
	spin_unlock_irqrestore(&drv_info->io_lock, flags);
	return IRQ_HANDLED;
}

void xen_front_evtchnl_flush(struct xen_front_evtchnl_info *channel)
{
	int notify;

	channel->u.req.ring.req_prod_pvt++;
	RING_PUSH_REQUESTS_AND_CHECK_NOTIFY(&channel->u.req.ring, notify);
	if (notify)
		notify_remote_via_irq(channel->irq);
}

static void evtchnl_free(struct drv_info *drv_info,
		struct xen_front_evtchnl_info *channel)
{
	unsigned long page = 0;

	if (channel->type == EVTCHNL_TYPE_REQ)
		page = (unsigned long)channel->u.req.ring.sring;
	else if (channel->type == EVTCHNL_TYPE_EVT)
		page = (unsigned long)channel->u.evt.page;

	if (!page)
		return;

	channel->state = EVTCHNL_STATE_DISCONNECTED;
	if (channel->type == EVTCHNL_TYPE_REQ) {
		/* release all who still waits for response if any */
		channel->u.req.resp_status = -EIO;
		complete_all(&channel->u.req.completion);
	}

	if (channel->irq)
		unbind_from_irqhandler(channel->irq, channel);

	if (channel->port)
		xenbus_free_evtchn(drv_info->xb_dev, channel->port);

	/* end access and free the page */
	if (channel->gref != GRANT_INVALID_REF)
		gnttab_end_foreign_access(channel->gref, 0, page);

	if (channel->type == EVTCHNL_TYPE_REQ)
		channel->u.req.ring.sring = NULL;
	else
		channel->u.evt.page = NULL;

	memset(channel, 0, sizeof(*channel));
}

void xen_front_evtchnl_free_all(struct drv_info *drv_info)
{
	int i;

	if (!drv_info->evt_pairs)
		return;

	for (i = 0; i < drv_info->num_evt_pairs; i++) {
		evtchnl_free(drv_info, &drv_info->evt_pairs[i].req);
		evtchnl_free(drv_info, &drv_info->evt_pairs[i].evt);
	}
	kfree(drv_info->evt_pairs);
	drv_info->evt_pairs = NULL;
}

static int evtchnl_alloc(struct drv_info *drv_info, int index,
	struct xen_front_evtchnl_info *channel,
	enum xen_front_evtchnl_type type)
{
	struct xenbus_device *xb_dev = drv_info->xb_dev;
	unsigned long page;
	grant_ref_t gref;
	irq_handler_t handler;
	char *handler_name = NULL;
	int ret;

	memset(channel, 0, sizeof(*channel));
	channel->type = type;
	channel->index = index;
	channel->drv_info = drv_info;
	channel->state = EVTCHNL_STATE_DISCONNECTED;
	channel->gref = GRANT_INVALID_REF;
	page = get_zeroed_page(GFP_NOIO | __GFP_HIGH);
	if (!page) {
		ret = -ENOMEM;
		goto fail;
	}

	handler_name = kasprintf(GFP_KERNEL, "%s-%s", XENSND_DRIVER_NAME,
		type == EVTCHNL_TYPE_REQ ? XENSND_FIELD_RING_REF :
			XENSND_FIELD_EVT_RING_REF);
	if (!handler_name) {
		ret = -ENOMEM;
		goto fail;
	}

	if (type == EVTCHNL_TYPE_REQ) {
		struct xen_sndif_sring *sring = (struct xen_sndif_sring *)page;

		init_completion(&channel->u.req.completion);
		SHARED_RING_INIT(sring);
		FRONT_RING_INIT(&channel->u.req.ring, sring, XEN_PAGE_SIZE);

		ret = xenbus_grant_ring(xb_dev, sring, 1, &gref);
		if (ret < 0)
			goto fail;

		handler = evtchnl_interrupt_req;
	} else {
		channel->u.evt.page = (struct xensnd_event_page *)page;
		ret = gnttab_grant_foreign_access(xb_dev->otherend_id,
			virt_to_gfn((void *)page), 0);
		if (ret < 0)
			goto fail;

		gref = ret;
		handler = evtchnl_interrupt_evt;
	}
	channel->gref = gref;

	ret = xenbus_alloc_evtchn(xb_dev, &channel->port);
	if (ret < 0)
		goto fail;

	ret = bind_evtchn_to_irq(channel->port);
	if (ret < 0) {
		dev_err(&xb_dev->dev,
			"Failed to bind IRQ for domid %d port %d: %d\n",
			drv_info->xb_dev->otherend_id, channel->port, ret);
		goto fail;
	}

	channel->irq = ret;

	ret = request_threaded_irq(channel->irq, NULL, handler,
		IRQF_ONESHOT, handler_name, channel);
	if (ret < 0) {
		dev_err(&xb_dev->dev, "Failed to request IRQ %d: %d\n",
			channel->irq, ret);
		goto fail;
	}

	kfree(handler_name);
	return 0;

fail:
	kfree(handler_name);
	dev_err(&xb_dev->dev, "Failed to allocate ring: %d\n", ret);
	return ret;
}

int xen_front_evtchnl_create_all(struct drv_info *drv_info, int num_streams)
{
	struct xen_front_cfg_card *cfg_card;
	int d, ret = 0;

	drv_info->evt_pairs = kcalloc(num_streams,
		sizeof(struct xen_front_evtchnl_pair_info), GFP_KERNEL);
	if (!drv_info->evt_pairs)
		return -ENOMEM;

	cfg_card = &drv_info->cfg_plat_data.cfg_card;
	/* iterate over devices and their streams and create event channels */
	for (d = 0; d < cfg_card->num_pcm_instances; d++) {
		struct xen_front_cfg_pcm_instance *pcm_instance;
		int s, index;

		pcm_instance = &cfg_card->pcm_instances[d];

		for (s = 0; s < pcm_instance->num_streams_pb; s++) {
			index = pcm_instance->streams_pb[s].index;

			ret = evtchnl_alloc(drv_info, index,
				&drv_info->evt_pairs[index].req,
				EVTCHNL_TYPE_REQ);
			if (ret < 0) {
				dev_err(&drv_info->xb_dev->dev,
					"Error allocating control channel\n");
				goto fail;
			}

			ret = evtchnl_alloc(drv_info, index,
				&drv_info->evt_pairs[index].evt,
				EVTCHNL_TYPE_EVT);
			if (ret < 0) {
				dev_err(&drv_info->xb_dev->dev,
					"Error allocating in-event channel\n");
				goto fail;
			}
		}

		for (s = 0; s < pcm_instance->num_streams_cap; s++) {
			index = pcm_instance->streams_cap[s].index;

			ret = evtchnl_alloc(drv_info, index,
				&drv_info->evt_pairs[index].req,
				EVTCHNL_TYPE_REQ);
			if (ret < 0) {
				dev_err(&drv_info->xb_dev->dev,
					"Error allocating control channel\n");
				goto fail;
			}

			ret = evtchnl_alloc(drv_info, index,
				&drv_info->evt_pairs[index].evt,
				EVTCHNL_TYPE_EVT);
			if (ret < 0) {
				dev_err(&drv_info->xb_dev->dev,
					"Error allocating in-event channel\n");
				goto fail;
			}
		}
	}
	if (ret < 0)
		goto fail;

	drv_info->num_evt_pairs = num_streams;
	return 0;

fail:
	xen_front_evtchnl_free_all(drv_info);
	return ret;
}

static int evtchnl_publish(struct xenbus_transaction xbt,
	struct xen_front_evtchnl_info *channel,
	const char *path, const char *node_ring,
	const char *node_chnl)
{
	struct xenbus_device *xb_dev = channel->drv_info->xb_dev;
	int ret;

	/* write control channel ring reference */
	ret = xenbus_printf(xbt, path, node_ring, "%u", channel->gref);
	if (ret < 0) {
		dev_err(&xb_dev->dev, "Error writing ring-ref: %d\n", ret);
		return ret;
	}

	/* write event channel ring reference */
	ret = xenbus_printf(xbt, path, node_chnl, "%u", channel->port);
	if (ret < 0) {
		dev_err(&xb_dev->dev, "Error writing event channel: %d\n", ret);
		return ret;
	}

	return 0;
}

int xen_front_evtchnl_publish_all(struct drv_info *drv_info)
{
	struct xenbus_transaction xbt;
	struct xen_front_cfg_card *cfg_card;
	int ret, d;

	cfg_card = &drv_info->cfg_plat_data.cfg_card;

again:
	ret = xenbus_transaction_start(&xbt);
	if (ret < 0) {
		xenbus_dev_fatal(drv_info->xb_dev, ret, "starting transaction");
		return ret;
	}

	for (d = 0; d < cfg_card->num_pcm_instances; d++) {
		struct xen_front_cfg_pcm_instance *pcm_instance;
		int s, index;

		pcm_instance = &cfg_card->pcm_instances[d];

		for (s = 0; s < pcm_instance->num_streams_pb; s++) {
			index = pcm_instance->streams_pb[s].index;

			ret = evtchnl_publish(xbt,
				&drv_info->evt_pairs[index].req,
				pcm_instance->streams_pb[s].xenstore_path,
				XENSND_FIELD_RING_REF,
				XENSND_FIELD_EVT_CHNL);
			if (ret < 0)
				goto fail;

			ret = evtchnl_publish(xbt,
				&drv_info->evt_pairs[index].evt,
				pcm_instance->streams_pb[s].xenstore_path,
				XENSND_FIELD_EVT_RING_REF,
				XENSND_FIELD_EVT_EVT_CHNL);
			if (ret < 0)
				goto fail;
		}

		for (s = 0; s < pcm_instance->num_streams_cap; s++) {
			index = pcm_instance->streams_cap[s].index;

			ret = evtchnl_publish(xbt,
				&drv_info->evt_pairs[index].req,
				pcm_instance->streams_cap[s].xenstore_path,
				XENSND_FIELD_RING_REF,
				XENSND_FIELD_EVT_CHNL);
			if (ret < 0)
				goto fail;

			ret = evtchnl_publish(xbt,
				&drv_info->evt_pairs[index].evt,
				pcm_instance->streams_cap[s].xenstore_path,
				XENSND_FIELD_EVT_RING_REF,
				XENSND_FIELD_EVT_EVT_CHNL);
			if (ret < 0)
				goto fail;
		}
	}
	ret = xenbus_transaction_end(xbt, 0);
	if (ret < 0) {
		if (ret == -EAGAIN)
			goto again;
		xenbus_dev_fatal(drv_info->xb_dev, ret,
			"completing transaction");
		goto fail_to_end;
	}
	return 0;
fail:
	xenbus_transaction_end(xbt, 1);
fail_to_end:
	xenbus_dev_fatal(drv_info->xb_dev, ret, "writing XenStore");
	return ret;
}
