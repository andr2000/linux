/* SPDX-License-Identifier: GPL-2.0 OR MIT */

/*
 *  Xen para-virtual DRM device
 *
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_DRM_FRONT_H_
#define __XEN_DRM_FRONT_H_

#include <linux/scatterlist.h>

#include "xen_drm_front_cfg.h"

/* timeout in ms to wait for backend to respond */
#define XEN_DRM_FRONT_WAIT_BACK_MS	3000

#ifndef GRANT_INVALID_REF
/*
 * Note on usage of grant reference 0 as invalid grant reference:
 * grant reference 0 is valid, but never exposed to a PV driver,
 * because of the fact it is already in use/reserved by the PV console.
 */
#define GRANT_INVALID_REF	0
#endif

struct xen_drm_front_drm_pipeline;

struct xen_drm_front_info {
	struct xenbus_device *xb_dev;
	/* to protect data between backend IO code and interrupt handler */
	spinlock_t io_lock;
	/* serializer for backend IO: request/response */
	struct mutex req_io_lock;
	bool drm_pdrv_registered;
	/* virtual DRM platform device */
	struct platform_device *drm_pdev;

	int num_evt_pairs;
	struct xen_drm_front_evtchnl_pair *evt_pairs;
	struct xen_drm_front_cfg cfg;

	/* display buffers */
	struct list_head dbuf_list;
};

int xen_drm_front_mode_set(struct xen_drm_front_drm_pipeline *pipeline,
		uint32_t x, uint32_t y, uint32_t width, uint32_t height,
		uint32_t bpp, uint64_t fb_cookie);

int xen_drm_front_dbuf_create_from_sgt(struct xen_drm_front_info *front_info,
		uint64_t dbuf_cookie, uint32_t width, uint32_t height,
		uint32_t bpp, uint64_t size, struct sg_table *sgt);

int xen_drm_front_dbuf_create_from_pages(struct xen_drm_front_info *front_info,
		uint64_t dbuf_cookie, uint32_t width, uint32_t height,
		uint32_t bpp, uint64_t size, struct page **pages);

int xen_drm_front_dbuf_destroy(struct xen_drm_front_info *front_info,
		uint64_t dbuf_cookie);

int xen_drm_front_fb_attach(struct xen_drm_front_info *front_info,
		uint64_t dbuf_cookie, uint64_t fb_cookie, uint32_t width,
		uint32_t height, uint32_t pixel_format);

int xen_drm_front_fb_detach(struct xen_drm_front_info *front_info,
		uint64_t fb_cookie);

int xen_drm_front_page_flip(struct xen_drm_front_info *front_info,
		int conn_idx, uint64_t fb_cookie);

void xen_drm_front_unload(struct xen_drm_front_info *front_info);

#endif /* __XEN_DRM_FRONT_H_ */
