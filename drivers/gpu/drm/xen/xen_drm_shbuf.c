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
#include <linux/mm.h>

#include <asm/xen/hypervisor.h>
#include <xen/xen.h>
#include <xen/balloon.h>
#include <xen/xenbus.h>
#include <xen/interface/io/ring.h>
#include <xen/interface/io/displif.h>

#include "xen_drm.h"
#include "xen_drm_shbuf.h"

static int xdrv_shbuf_unmap(struct xdrv_shared_buffer_info *buf);

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
		if (buf->ext_buffer)
			xdrv_shbuf_unmap(buf);
		else
			for (i = 0; i < buf->num_grefs; i++)
				if (buf->grefs[i] != GRANT_INVALID_REF)
					gnttab_end_foreign_access(buf->grefs[i],
						0, 0UL);
	}
	kfree(buf->grefs);
	buf->grefs = NULL;
	kfree(buf->vdirectory);
	if (buf->ext_buffer) {
		free_xenballooned_pages(buf->num_pages, buf->pages);
		kfree(buf->pages);
		buf->pages = NULL;
		kfree(buf->map_info);
		buf->map_info = NULL;
	}
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

static int xdrv_shbuf_grant_refs(struct xdrv_shared_buffer_info *buf,
	int num_pages_dir, int num_pages_vbuffer, int num_grefs)
{
	grant_ref_t priv_gref_head;
	int ret, i, j, cur_ref;
	int otherend_id;
	struct sg_page_iter sg_iter;

	ret = gnttab_alloc_grant_references(num_grefs, &priv_gref_head);
	if (ret < 0) {
		DRM_ERROR("Cannot allocate grant references\n");
		return ret;
	}
	buf->num_grefs = num_grefs;
	otherend_id = buf->xb_dev->otherend_id;
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
	for_each_sg_page(buf->sgt->sgl, &sg_iter, buf->sgt->nents, 0) {
		struct page *page;

		page = sg_page_iter_page(&sg_iter);
		cur_ref = gnttab_claim_grant_reference(&priv_gref_head);
		if (cur_ref < 0)
			return cur_ref;
		gnttab_grant_foreign_access_ref(cur_ref,
			otherend_id, xen_page_to_gfn(page), 0);
		buf->grefs[j++] = cur_ref;
		num_pages_vbuffer--;
	}
	WARN_ON(num_pages_vbuffer != 0);
	gnttab_free_grant_references(priv_gref_head);
	return 0;
}

static int xdrv_shbuf_alloc_storage(struct xdrv_shared_buffer_info *buf,
	int num_pages_dir, int num_grefs)
{
	int ret;

	buf->grefs = kcalloc(num_grefs, sizeof(*buf->grefs), GFP_KERNEL);
	if (!buf->grefs)
		return -ENOMEM;
	buf->vdirectory = kcalloc(num_pages_dir, XEN_PAGE_SIZE, GFP_KERNEL);
	if (!buf->vdirectory)
		return -ENOMEM;
	buf->vbuffer_sz = buf->num_pages * XEN_PAGE_SIZE;
	if (buf->ext_buffer) {
		buf->pages = kcalloc(buf->num_pages, sizeof(*buf->pages),
			GFP_KERNEL);
		if (!buf->pages)
			return -ENOMEM;
		buf->map_info = kcalloc(buf->num_pages,
			sizeof(*buf->map_info), GFP_KERNEL);
		if (!buf->map_info)
			return -ENOMEM;
		ret = alloc_xenballooned_pages(buf->num_pages, buf->pages);
		if (ret < 0) {
			DRM_ERROR("Cannot allocate %d ballooned pages: %d\n",
				buf->num_pages, ret);
			return -ENOMEM;
		}
	}
	return 0;
}

struct xdrv_shared_buffer_info *xdrv_shbuf_alloc(struct xenbus_device *xb_dev,
	struct list_head *dumb_buf_list, uint64_t dumb_cookie,
	struct sg_table *sgt, unsigned int buffer_size, bool ext_buffer)
{
	struct xdrv_shared_buffer_info *buf;
	int num_pages_vbuffer, num_pages_dir, num_grefs;

	if (!sgt)
		return NULL;
	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return NULL;
	buf->xb_dev = xb_dev;
	buf->ext_buffer = ext_buffer;
	buf->sgt = sgt;
	buf->dumb_cookie = dumb_cookie;
	num_pages_vbuffer = DIV_ROUND_UP(buffer_size, XEN_PAGE_SIZE);
	buf->num_pages = num_pages_vbuffer;
	/* number of pages the directory itself consumes */
	num_pages_dir = DIV_ROUND_UP(num_pages_vbuffer,
		XENDRM_NUM_GREFS_PER_PAGE);
	num_grefs = num_pages_vbuffer + num_pages_dir;

	if (xdrv_shbuf_alloc_storage(buf, num_pages_dir, num_grefs) < 0)
		goto fail;
	if (!ext_buffer) {
		if (xdrv_shbuf_grant_refs(buf, num_pages_dir,
				num_pages_vbuffer, num_grefs) < 0)
			goto fail;
		xdrv_shbuf_fill_page_dir(buf, num_pages_dir);
	}
	list_add(&buf->list, dumb_buf_list);
	return buf;
fail:
	xdrv_shbuf_free(buf);
	return NULL;
}

#define xen_page_to_vaddr(page) \
	((phys_addr_t)pfn_to_kaddr(page_to_xen_pfn(page)))

struct sg_table *xdrv_shbuf_map(struct xdrv_shared_buffer_info *buf)
{
	struct gnttab_map_grant_ref *map_ops = NULL;
	int ret, i;

	map_ops = kcalloc(buf->num_pages, sizeof(*map_ops), GFP_KERNEL);
	if (!map_ops)
		return NULL;
	for (i = 0; i < buf->num_pages; i++) {
		phys_addr_t addr;

		/* Map the grant entry for access by host CPUs. */
		addr = xen_page_to_vaddr(buf->pages[i]);
		gnttab_set_map_op(&map_ops[i], addr, GNTMAP_host_map,
			buf->grefs[i], buf->xb_dev->otherend_id);
	}
	ret = gnttab_map_refs(map_ops, NULL, buf->pages, buf->num_pages);
	BUG_ON(ret);
	for (i = 0; i < buf->num_pages; i++) {
		buf->map_info[i].handle = map_ops[i].handle;
		if (unlikely(map_ops[i].status != GNTST_okay))
			DRM_ERROR("Failed to map page %d with ref %d: %d\n",
				i, buf->grefs[i], map_ops[i].status);
	}
	kfree(map_ops);
	buf->sgt = drm_prime_pages_to_sg(buf->pages, buf->num_pages);
	return buf->sgt;
}

static int xdrv_shbuf_unmap(struct xdrv_shared_buffer_info *buf)
{
	struct gnttab_unmap_grant_ref *unmap_ops;
	int i;

	if (!buf->pages || !buf->map_info)
		return 0;

	unmap_ops = kcalloc(buf->num_pages, sizeof(*unmap_ops), GFP_KERNEL);
	if (!unmap_ops)
		return -ENOMEM;
	for (i = 0; i < buf->num_pages; i++) {
		phys_addr_t addr;

		/* Map the grant entry for access by I/O devices.
		 * Map the grant entry for access by host CPUs.
		 * If <host_addr> or <dev_bus_addr> is zero, that
		 * field is ignored. If non-zero, they must refer to
		 * a device/host mapping that is tracked by <handle>
		 */
		addr = xen_page_to_vaddr(buf->pages[i]);
		gnttab_set_unmap_op(&unmap_ops[i], addr, GNTMAP_host_map,
			buf->map_info[i].handle);
	}
	BUG_ON(gnttab_unmap_refs(unmap_ops, NULL, buf->pages, buf->num_pages));
	for (i = 0; i < buf->num_pages; i++) {
		if (unlikely(unmap_ops[i].status != GNTST_okay))
			DRM_ERROR("Failed to unmap page %d with ref %d: %d\n",
				i, buf->grefs[i], unmap_ops[i].status);
	}
	kfree(unmap_ops);
	return 0;
}
