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
 * Copyright (C) 2017 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <Oleksandr_Andrushchenko@epam.com>
 */

#ifndef __XEN_DRM_SHBUF_H_
#define __XEN_DRM_SHBUF_H_

#include <linux/kernel.h>
#include <linux/scatterlist.h>

#include <xen/grant_table.h>

#define GRANT_INVALID_REF	0

struct xdrv_shared_buffer_info {
	struct list_head list;
	uint64_t dumb_cookie;
	uint64_t fb_cookie;
	/* number of references granted for the backend use:
	 * for internal buffers this holds grefs for the
	 * page directory and pages of the buffer
	 * for external buffer this only has grefs for the page
	 * directory as buffer grefs will be provided by the backend
	 */
	int num_grefs;
	grant_ref_t *grefs;
	unsigned char *vdirectory;
	struct sg_table *sgt;

	/* external buffer handling */
	struct xenbus_device *xb_dev;
	/* set if this buffer was allocated by the backend */
	bool ext_buffer;
	/* ballooned pages */
	int ext_num_pages;
	struct page **ext_pages;
	/* Xen map handle */
	grant_handle_t *ext_map_handle;
};

grant_ref_t xdrv_shbuf_get_dir_start(struct xdrv_shared_buffer_info *buf);
struct xdrv_shared_buffer_info *xdrv_shbuf_alloc(struct xenbus_device *xb_dev,
	struct list_head *dumb_buf_list, uint64_t dumb_cookie,
	struct sg_table *sgt, unsigned int buffer_size, bool ext_buffer);
int xdrv_shbuf_ext_map(struct xdrv_shared_buffer_info *buf);
struct sg_table *xdrv_shbuf_get_sg_table(struct xdrv_shared_buffer_info *buf);
struct xdrv_shared_buffer_info *xdrv_shbuf_get_by_dumb_cookie(
	struct list_head *dumb_buf_list, uint64_t dumb_cookie);
void xdrv_shbuf_flush_fb(struct list_head *dumb_buf_list, uint64_t fb_cookie);
void xdrv_shbuf_free_by_dumb_cookie(struct list_head *dumb_buf_list,
	uint64_t dumb_cookie);
void xdrv_shbuf_free_all(struct list_head *dumb_buf_list);

#endif /* __XEN_DRM_SHBUF_H_ */
