/* SPDX-License-Identifier: GPL-2.0 OR MIT */

/*
 * Xen memory reservation functionality
 */

#ifndef _XENMEM_RESERVATION_H
#define _XENMEM_RESERVATION_H

void xenmem_reservation_scrub_page(struct page *page);

void xenmem_reservation_va_mapping_update(unsigned long count,
					  struct page **pages,
					  xen_pfn_t *frames);

void xenmem_reservation_va_mapping_reset(unsigned long count,
					 struct page **pages);

int xenmem_reservation_increase(int count, xen_pfn_t *frames);

int xenmem_reservation_decrease(int count, xen_pfn_t *frames);

#endif
