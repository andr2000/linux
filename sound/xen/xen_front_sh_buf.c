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

#include <xen/xen.h>
#include <xen/xenbus.h>

#include "xen_front_sh_buf.h"

void xen_front_sh_buf_free(struct xen_front_sh_buf_info *buf)
{
	int i;

	if (buf->grefs) {
		for (i = 0; i < buf->num_grefs; i++)
			if (buf->grefs[i] != GRANT_INVALID_REF)
				gnttab_end_foreign_access(buf->grefs[i],
						0, 0UL);
		kfree(buf->grefs);
	}
	kfree(buf->vdirectory);
	free_pages_exact(buf->vbuffer, buf->vbuffer_sz);
	xen_front_sh_buf_clear(buf);
}

/*
 * number of grant references a page can hold with respect to the
 * xensnd_page_directory header
 */
#define XENSND_NUM_GREFS_PER_PAGE ((XEN_PAGE_SIZE - \
	offsetof(struct xensnd_page_directory, gref)) / \
	sizeof(grant_ref_t))

static void sh_buf_fill_page_dir(struct xen_front_sh_buf_info *buf,
	int num_pages_dir)
{
	struct xensnd_page_directory *page_dir;
	unsigned char *ptr;
	int i, cur_gref, grefs_left, to_copy;

	ptr = buf->vdirectory;
	grefs_left = buf->num_grefs - num_pages_dir;
	/*
	 * skip grant references at the beginning, they are for pages granted
	 * for the page directory itself
	 */
	cur_gref = num_pages_dir;
	for (i = 0; i < num_pages_dir; i++) {
		page_dir = (struct xensnd_page_directory *)ptr;
		if (grefs_left <= XENSND_NUM_GREFS_PER_PAGE) {
			to_copy = grefs_left;
			page_dir->gref_dir_next_page = GRANT_INVALID_REF;
		} else {
			to_copy = XENSND_NUM_GREFS_PER_PAGE;
			page_dir->gref_dir_next_page = buf->grefs[i + 1];
		}
		memcpy(&page_dir->gref, &buf->grefs[cur_gref],
			to_copy * sizeof(grant_ref_t));
		ptr += XEN_PAGE_SIZE;
		grefs_left -= to_copy;
		cur_gref += to_copy;
	}
}

static int sh_buf_grant_refs(struct xenbus_device *xb_dev,
	struct xen_front_sh_buf_info *buf,
	int num_pages_dir, int num_pages_vbuffer, int num_grefs)
{
	grant_ref_t priv_gref_head;
	int ret, i, j, cur_ref;
	int otherend_id;

	ret = gnttab_alloc_grant_references(num_grefs, &priv_gref_head);
	if (ret)
		return ret;

	buf->num_grefs = num_grefs;
	otherend_id = xb_dev->otherend_id;
	j = 0;

	for (i = 0; i < num_pages_dir; i++) {
		cur_ref = gnttab_claim_grant_reference(&priv_gref_head);
		if (cur_ref < 0) {
			ret = cur_ref;
			goto fail;
		}

		gnttab_grant_foreign_access_ref(cur_ref, otherend_id,
			xen_page_to_gfn(virt_to_page(buf->vdirectory +
				XEN_PAGE_SIZE * i)), 0);
		buf->grefs[j++] = cur_ref;
	}

	for (i = 0; i < num_pages_vbuffer; i++) {
		cur_ref = gnttab_claim_grant_reference(&priv_gref_head);
		if (cur_ref < 0) {
			ret = cur_ref;
			goto fail;
		}

		gnttab_grant_foreign_access_ref(cur_ref, otherend_id,
			xen_page_to_gfn(virt_to_page(buf->vbuffer +
				XEN_PAGE_SIZE * i)), 0);
		buf->grefs[j++] = cur_ref;
	}

	gnttab_free_grant_references(priv_gref_head);
	sh_buf_fill_page_dir(buf, num_pages_dir);
	return 0;

fail:
	gnttab_free_grant_references(priv_gref_head);
	return ret;
}

static int sh_buf_alloc_int_buffers(struct xen_front_sh_buf_info *buf,
		int num_pages_dir, int num_pages_vbuffer, int num_grefs)
{
	buf->grefs = kcalloc(num_grefs, sizeof(*buf->grefs), GFP_KERNEL);
	if (!buf->grefs)
		return -ENOMEM;

	buf->vdirectory = kcalloc(num_pages_dir, XEN_PAGE_SIZE, GFP_KERNEL);
	if (!buf->vdirectory)
		goto fail;

	buf->vbuffer_sz = num_pages_vbuffer * XEN_PAGE_SIZE;
	buf->vbuffer = alloc_pages_exact(buf->vbuffer_sz, GFP_KERNEL);
	if (!buf->vbuffer)
		goto fail;
	return 0;

fail:
	kfree(buf->grefs);
	buf->grefs = NULL;
	kfree(buf->vdirectory);
	buf->vdirectory = NULL;
	return -ENOMEM;
}

int xen_front_sh_buf_alloc(struct xenbus_device *xb_dev,
	struct xen_front_sh_buf_info *buf, unsigned int buffer_sz)
{
	int num_pages_vbuffer, num_pages_dir, num_grefs;
	int ret;

	xen_front_sh_buf_clear(buf);

	num_pages_vbuffer = DIV_ROUND_UP(buffer_sz, XEN_PAGE_SIZE);
	/* number of pages the page directory consumes itself */
	num_pages_dir = DIV_ROUND_UP(num_pages_vbuffer,
		XENSND_NUM_GREFS_PER_PAGE);
	num_grefs = num_pages_vbuffer + num_pages_dir;

	ret = sh_buf_alloc_int_buffers(buf, num_pages_dir,
		num_pages_vbuffer, num_grefs);
	if (ret < 0)
		return ret;

	ret = sh_buf_grant_refs(xb_dev, buf,
		num_pages_dir, num_pages_vbuffer, num_grefs);
	if (ret < 0)
		return ret;

	sh_buf_fill_page_dir(buf, num_pages_dir);
	return 0;
}
