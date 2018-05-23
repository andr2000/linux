/* SPDX-License-Identifier: GPL-2.0 OR MIT */

/*
 * Xen memory reservation functionality
 */

#ifndef _XEN_MEM_RESERVATION_H
#define _XEN_MEM_RESERVATION_H

void xen_mem_reservation_scrub_page(struct page *page);

void xen_mem_reservation_va_mapping_update(unsigned long count,
					   struct page **pages,
					   xen_pfn_t *frames);

void xen_mem_reservation_va_mapping_reset(unsigned long count,
					  struct page **pages);

int xen_mem_reservation_increase(int count, xen_pfn_t *frames);

int xen_mem_reservation_decrease(int count, xen_pfn_t *frames);

#endif
