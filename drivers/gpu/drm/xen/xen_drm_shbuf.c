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

#include <linux/errno.h>

#include <asm/xen/hypervisor.h>
#include <xen/xen.h>
#include <xen/xenbus.h>
#include <xen/interface/io/ring.h>
#include <xen/interface/io/displif.h>

#include "xen_drm.h"
#include "xen_drm_shbuf.h"

grant_ref_t xdrv_shbuf_get_dir_start(struct xdrv_shared_buffer_info *buf)
{
	if (!buf->grefs)
		return GRANT_INVALID_REF;
	return buf->grefs[0];
}

struct xdrv_shared_buffer_info *xdrv_shbuf_get_by_dumb_cookie(
	struct list_head *dumb_buf_list, uint64_t dumb_cookie)
{
	struct xdrv_shared_buffer_info *buf, *q;

	list_for_each_entry_safe(buf, q, dumb_buf_list, list) {
		if (buf->dumb_cookie == dumb_cookie)
			return buf;
	}
	return NULL;
}

void xdrv_shbuf_flush_fb(struct list_head *dumb_buf_list, uint64_t fb_cookie)
{
#ifdef CONFIG_X86
	struct xdrv_shared_buffer_info *buf, *q;

	list_for_each_entry_safe(buf, q, dumb_buf_list, list) {
		if (buf->fb_cookie == fb_cookie) {
			struct scatterlist *sg;
			unsigned int count;

			for_each_sg(buf->sgt->sgl, sg, buf->sgt->nents, count)
				clflush_cache_range(sg_virt(sg), sg->length);
			break;
		}
	}
#endif
}

static void xdrv_shbuf_free(struct xdrv_shared_buffer_info *buf)
{
	int i;

	if (buf->grefs) {
		/* [0] entry is used for page directory, so skip it and use
		 * the one which is used for the buffer which is expected
		 * to be released at this time
		 */
		if (unlikely(gnttab_query_foreign_access(buf->grefs[1]) &&
				buf->num_grefs)) {
			int try = 5;

			/* reference is not yet updated by the Xen, we can
			 * end access but it will make the removal deferred,
			 * so give it a chance
			 */
			do {
				DRM_WARN("Grant refs are not released yet\n");
				msleep(20);
				if (!gnttab_query_foreign_access(buf->grefs[1]))
					break;
			} while (--try);
		}
		for (i = 0; i < buf->num_grefs; i++)
			if (buf->grefs[i] != GRANT_INVALID_REF)
				gnttab_end_foreign_access(buf->grefs[i],
					0, 0UL);
		kfree(buf->grefs);
	}
	kfree(buf->vdirectory);
	sg_free_table(buf->sgt);
	kfree(buf);
}

void xdrv_shbuf_free_by_dumb_cookie(struct list_head *dumb_buf_list,
	uint64_t dumb_cookie)
{
	struct xdrv_shared_buffer_info *buf, *q;

	list_for_each_entry_safe(buf, q, dumb_buf_list, list) {
		if (buf->dumb_cookie == dumb_cookie) {
			list_del(&buf->list);
			xdrv_shbuf_free(buf);
			break;
		}
	}
}

void xdrv_shbuf_free_all(struct list_head *dumb_buf_list)
{
	struct xdrv_shared_buffer_info *buf, *q;

	list_for_each_entry_safe(buf, q, dumb_buf_list, list) {
		list_del(&buf->list);
		xdrv_shbuf_free(buf);
	}
}

/* number of grefs a page can hold with respect to the
 * xendispl_page_directory header
 */
#define XENDRM_NUM_GREFS_PER_PAGE ((XEN_PAGE_SIZE - \
	offsetof(struct xendispl_page_directory, gref)) / \
	sizeof(grant_ref_t))

static void xdrv_shbuf_fill_page_dir(struct xdrv_shared_buffer_info *buf,
	int num_pages_dir)
{
	struct xendispl_page_directory *page_dir;
	unsigned char *ptr;
	int i, cur_gref, grefs_left, to_copy;

	ptr = buf->vdirectory;
	grefs_left = buf->num_grefs - num_pages_dir;
	/* skip grefs at start, they are for pages granted for the directory */
	cur_gref = num_pages_dir;
	for (i = 0; i < num_pages_dir; i++) {
		page_dir = (struct xendispl_page_directory *)ptr;
		if (grefs_left <= XENDRM_NUM_GREFS_PER_PAGE) {
			to_copy = grefs_left;
			page_dir->gref_dir_next_page = GRANT_INVALID_REF;
		} else {
			to_copy = XENDRM_NUM_GREFS_PER_PAGE;
			page_dir->gref_dir_next_page = buf->grefs[i + 1];
		}
		memcpy(&page_dir->gref, &buf->grefs[cur_gref],
			to_copy * sizeof(grant_ref_t));
		ptr += XEN_PAGE_SIZE;
		grefs_left -= to_copy;
		cur_gref += to_copy;
	}
}

static int xdrv_shbuf_grant_refs(struct xenbus_device *xb_dev,
	struct xdrv_shared_buffer_info *buf,
	int num_pages_dir, int num_pages_vbuffer, int num_grefs)
{
	grant_ref_t priv_gref_head;
	int ret, i, j, cur_ref;
	int otherend_id;
	int count;
	struct scatterlist *sg;

	ret = gnttab_alloc_grant_references(num_grefs, &priv_gref_head);
	if (ret < 0) {
		DRM_ERROR("Cannot allocate grant references\n");
		return ret;
	}
	buf->num_grefs = num_grefs;
	otherend_id = xb_dev->otherend_id;
	j = 0;
	for (i = 0; i < num_pages_dir; i++) {
		cur_ref = gnttab_claim_grant_reference(&priv_gref_head);
		if (cur_ref < 0)
			return cur_ref;
		gnttab_grant_foreign_access_ref(cur_ref, otherend_id,
			xen_page_to_gfn(virt_to_page(buf->vdirectory +
				XEN_PAGE_SIZE * i)), 0);
		buf->grefs[j++] = cur_ref;
	}
	for_each_sg(buf->sgt->sgl, sg, buf->sgt->nents, count) {
		struct page *page;
		int len;

		len = sg->length;
		page = sg_page(sg);
		while (len > 0) {
			cur_ref = gnttab_claim_grant_reference(&priv_gref_head);
			if (cur_ref < 0)
				return cur_ref;
			gnttab_grant_foreign_access_ref(cur_ref, otherend_id,
				xen_page_to_gfn(page), 0);
			buf->grefs[j++] = cur_ref;
			len -= PAGE_SIZE;
			page++;
			num_pages_vbuffer--;
		}
	}
	WARN_ON(num_pages_vbuffer != 0);
	gnttab_free_grant_references(priv_gref_head);
	return 0;
}

static int xdrv_shbuf_alloc_buffers(struct xdrv_shared_buffer_info *buf,
	int num_pages_dir, int num_pages_vbuffer, int num_grefs)
{
	buf->grefs = kcalloc(num_grefs, sizeof(*buf->grefs), GFP_KERNEL);
	if (!buf->grefs)
		return -ENOMEM;
	buf->vdirectory = kcalloc(num_pages_dir, XEN_PAGE_SIZE, GFP_KERNEL);
	if (!buf->vdirectory) {
		kfree(buf->grefs);
		buf->grefs = NULL;
		return -ENOMEM;
	}
	buf->vbuffer_sz = num_pages_vbuffer * XEN_PAGE_SIZE;
	return 0;
}

struct xdrv_shared_buffer_info *xdrv_shbuf_alloc(struct xenbus_device *xb_dev,
	struct list_head *dumb_buf_list, uint64_t dumb_cookie,
	struct sg_table *sgt, unsigned int buffer_size)
{
	struct xdrv_shared_buffer_info *buf;
	int num_pages_vbuffer, num_pages_dir, num_grefs;

	if (!sgt)
		return NULL;
	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return NULL;
	buf->sgt = sgt;
	buf->dumb_cookie = dumb_cookie;
	num_pages_vbuffer = DIV_ROUND_UP(buffer_size, XEN_PAGE_SIZE);
	/* number of pages the directory itself consumes */
	num_pages_dir = DIV_ROUND_UP(num_pages_vbuffer,
		XENDRM_NUM_GREFS_PER_PAGE);
	num_grefs = num_pages_vbuffer + num_pages_dir;

	if (xdrv_shbuf_alloc_buffers(buf, num_pages_dir,
			num_pages_vbuffer, num_grefs) < 0)
		goto fail;
	if (xdrv_shbuf_grant_refs(xb_dev, buf,
			num_pages_dir, num_pages_vbuffer, num_grefs) < 0)
		goto fail;
	xdrv_shbuf_fill_page_dir(buf, num_pages_dir);
	list_add(&buf->list, dumb_buf_list);
	return buf;
fail:
	xdrv_shbuf_free(buf);
	return NULL;
}
