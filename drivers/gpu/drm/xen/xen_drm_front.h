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

#ifndef GRANT_INVALID_REF
/*
 * Note on usage of grant reference 0 as invalid grant reference:
 * grant reference 0 is valid, but never exposed to a PV driver,
 * because of the fact it is already in use/reserved by the PV console.
 */
#define GRANT_INVALID_REF	0
#endif

struct xen_drm_front_drm_pipeline;

/*
 *******************************************************************************
 * Para-virtualized DRM/KMS frontend driver
 *******************************************************************************
 * This frontend driver implements Xen para-virtualized display
 * according to the display protocol described at
 * include/xen/interface/io/displif.h
 *
 *******************************************************************************
 * Driver modes of operation in terms of display buffers used
 *******************************************************************************
 * Depending on the requirements for the para-virtualized environment, namely
 * requirements dictated by the accompanying DRM/(v)GPU drivers running in both
 * host and guest environments, number of operating modes of para-virtualized
 * display driver are supported:
 *  - display buffers can be allocated by either frontend driver or backend
 *  - display buffers can be allocated to be contiguous in memory or not
 *
 * Note! Frontend driver itself has no dependency on contiguous memory for
 *       its operation.
 *
 *******************************************************************************
 * 1. Buffers allocated by the frontend driver.
 *******************************************************************************
 *
 * The below modes of operation are configured at compile-time via
 * frontend driver's kernel configuration.
 *
 * 1.1. Front driver configured to use GEM CMA helpers
 *      This use-case is useful when used with accompanying DRM/vGPU driver in
 *      guest domain which was designed to only work with contiguous buffers,
 *      e.g. DRM driver based on GEM CMA helpers: such drivers can only import
 *      contiguous PRIME buffers, thus requiring frontend driver to provide
 *      such. In order to implement this mode of operation para-virtualized
 *      frontend driver can be configured to use GEM CMA helpers.
 *
 * 1.2. Front driver doesn't use GEM CMA
 *      If accompanying drivers can cope with non-contiguous memory then, to
 *      lower pressure on CMA subsystem of the kernel, driver can allocate
 *      buffers from system memory.
 *
 * Note! If used with accompanying DRM/(v)GPU drivers this mode of operation
 *   may require IOMMU support on the platform, so accompanying DRM/vGPU
 *   hardware can still reach display buffer memory while importing PRIME
 *   buffers from the frontend driver.
 *
 *******************************************************************************
 * 2. Buffers allocated by the backend
 *******************************************************************************
 *
 * This mode of operation is run-time configured via guest domain configuration
 * through XenStore entries.
 *
 * For systems which do not provide IOMMU support, but having specific
 * requirements for display buffers it is possible to allocate such buffers
 * at backend side and share those with the frontend.
 * For example, if host domain is 1:1 mapped and has DRM/GPU hardware expecting
 * physically contiguous memory, this allows implementing zero-copying
 * use-cases.
 *
 *******************************************************************************
 * Driver limitations
 *******************************************************************************
 * 1. Configuration options 1.1 (contiguous display buffers) and 2 (backend
 *    allocated buffers) are not supported at the same time.
 *
 * 2. Only primary plane without additional properties is supported.
 *
 * 3. Only one video mode supported which is configured via XenStore.
 *
 * 4. All CRTCs operate at fixed frequency of 60Hz.
 *
 ******************************************************************************/

struct xen_drm_front_ops {
	int (*mode_set)(struct xen_drm_front_drm_pipeline *pipeline,
			uint32_t x, uint32_t y, uint32_t width, uint32_t height,
			uint32_t bpp, uint64_t fb_cookie);
	int (*dbuf_create_from_pages)(struct xen_drm_front_info *front_info,
			uint64_t dbuf_cookie, uint32_t width, uint32_t height,
			uint32_t bpp, uint64_t size, struct page **pages);
	int (*dbuf_create_from_sgt)(struct xen_drm_front_info *front_info,
			uint64_t dbuf_cookie, uint32_t width, uint32_t height,
			uint32_t bpp, uint64_t size, struct sg_table *sgt);
	int (*dbuf_destroy)(struct xen_drm_front_info *front_info,
			uint64_t dbuf_cookie);
	int (*fb_attach)(struct xen_drm_front_info *front_info,
			uint64_t dbuf_cookie, uint64_t fb_cookie,
			uint32_t width, uint32_t height, uint32_t pixel_format);
	int (*fb_detach)(struct xen_drm_front_info *front_info,
			uint64_t fb_cookie);
	int (*page_flip)(struct xen_drm_front_info *front_info,
			int conn_idx, uint64_t fb_cookie);
	/* CAUTION! this is called with a spin_lock held! */
	void (*on_frame_done)(struct platform_device *pdev,
			int conn_idx, uint64_t fb_cookie);
	void (*drm_last_close)(struct xen_drm_front_info *front_info);
};

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

#endif /* __XEN_DRM_FRONT_H_ */
