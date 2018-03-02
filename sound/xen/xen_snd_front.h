/* SPDX-License-Identifier: GPL-2.0 OR MIT */

/*
 * Xen para-virtual sound device
 *
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_SND_FRONT_H
#define __XEN_SND_FRONT_H

#include "xen_snd_front_cfg.h"

struct xen_snd_front_evtchnl;
struct xen_snd_front_evtchnl_pair;
struct xen_snd_front_shbuf;
struct xensnd_query_hw_param;

struct xen_snd_front_info {
	struct xenbus_device *xb_dev;
	spinlock_t io_lock;
	/* serializer for backend IO: request/response */
	struct mutex req_io_lock;
	bool snd_pdrv_registered;
	struct platform_device *snd_pdev;
	struct xen_snd_front_ops *front_ops;
	int num_evt_pairs;
	struct xen_snd_front_evtchnl_pair *evt_pairs;
	struct xen_snd_front_cfg cfg;
};

struct xen_snd_front_ops {
	int (*query_hw_param)(struct xen_snd_front_evtchnl *evtchnl,
			struct xensnd_query_hw_param *hw_param_req,
			struct xensnd_query_hw_param *hw_param_resp);
	int (*prepare)(struct xen_snd_front_evtchnl *evtchnl,
			struct xen_snd_front_shbuf *sh_buf,
			uint8_t format, unsigned int channels,
			unsigned int rate, uint32_t buffer_sz,
			uint32_t period_sz);
	int (*close)(struct xen_snd_front_evtchnl *evtchnl);
	int (*write)(struct xen_snd_front_evtchnl *evtchnl,
			unsigned long pos, unsigned long count);
	int (*read)(struct xen_snd_front_evtchnl *evtchnl,
			unsigned long pos, unsigned long count);
	int (*trigger)(struct xen_snd_front_evtchnl *evtchnl,
			int type);
	void (*handle_cur_pos)(struct xen_snd_front_evtchnl *evtchnl,
			uint64_t pos_bytes);
};

#endif /* __XEN_SND_FRONT_H */
