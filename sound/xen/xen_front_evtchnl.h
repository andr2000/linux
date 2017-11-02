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

#ifndef __XEN_FRONT_EVTCHNL_H
#define __XEN_FRONT_EVTCHNL_H

#include <xen/interface/io/sndif.h>

struct drv_info;

/*
 * FIXME: usage of grant reference 0 as invalid grant reference:
 * grant reference 0 is valid, but never exposed to a PV driver,
 * because of the fact it is already in use/reserved by the PV console.
 */
#define GRANT_INVALID_REF	0

/* timeout in ms to wait for backend to respond */
#define VSND_WAIT_BACK_MS	3000

enum xen_front_evtchnl_state {
	EVTCHNL_STATE_DISCONNECTED,
	EVTCHNL_STATE_CONNECTED,
};

enum xen_front_evtchnl_type {
	EVTCHNL_TYPE_REQ,
	EVTCHNL_TYPE_EVT,
};

struct xen_front_evtchnl_info {
	struct drv_info *drv_info;
	int gref;
	int port;
	int irq;
	int index;
	/* state of the event channel */
	enum xen_front_evtchnl_state state;
	enum xen_front_evtchnl_type type;
	/* either response id or incoming event id */
	uint16_t evt_id;
	/* next request id or next expected event id */
	uint16_t evt_next_id;
	union {
		struct {
			struct xen_sndif_front_ring ring;
			struct completion completion;
			/* latest response status */
			int resp_status;
		} req;
		struct {
			struct xensnd_event_page *page;
			/* this is needed to handle XENSND_EVT_CUR_POS event */
			struct snd_pcm_substream *substream;
		} evt;
	} u;
};

struct xen_front_evtchnl_pair_info {
	struct xen_front_evtchnl_info req;
	struct xen_front_evtchnl_info evt;
};

int xen_front_evtchnl_create_all(struct drv_info *drv_info, int num_streams);
void xen_front_evtchnl_free_all(struct drv_info *drv_info);
int xen_front_evtchnl_publish_all(struct drv_info *drv_info);

void xen_front_evtchnl_flush(struct xen_front_evtchnl_info *channel);

#endif /* __XEN_FRONT_EVTCHNL_H */
