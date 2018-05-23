// SPDX-License-Identifier: GPL-2.0 OR MIT

/******************************************************************************
 * Xen memory reservation utilities.
 *
 * Copyright (c) 2003, B Dragovic
 * Copyright (c) 2003-2004, M Williamson, K Fraser
 * Copyright (c) 2005 Dan M. Smith, IBM Corporation
 * Copyright (c) 2010 Daniel Kiper
 * Copyright (c) 2018, Oleksandr Andrushchenko, EPAM Systems Inc.
 */

#include <linux/kernel.h>
#include <linux/slab.h>

#include <asm/tlb.h>
#include <asm/xen/hypercall.h>

#include <xen/interface/memory.h>
#include <xen/page.h>

/*
 * Use one extent per PAGE_SIZE to avoid to break down the page into
 * multiple frame.
 */
#define EXTENT_ORDER (fls(XEN_PFN_PER_PAGE) - 1)

void xenmem_reservation_scrub_page(struct page *page)
{
#ifdef CONFIG_XEN_SCRUB_PAGES
	clear_highpage(page);
#endif
}
EXPORT_SYMBOL(xenmem_reservation_scrub_page);

void xenmem_reservation_va_mapping_update(unsigned long count,
					  struct page **pages,
					  xen_pfn_t *frames)
{
#ifdef CONFIG_XEN_HAVE_PVMMU
	int i;

	for (i = 0; i < count; i++) {
		struct page *page;

		page = pages[i];
		BUG_ON(page == NULL);

		/*
		 * We don't support PV MMU when Linux and Xen is using
		 * different page granularity.
		 */
		BUILD_BUG_ON(XEN_PAGE_SIZE != PAGE_SIZE);

		if (!xen_feature(XENFEAT_auto_translated_physmap)) {
			unsigned long pfn = page_to_pfn(page);

			set_phys_to_machine(pfn, frames[i]);

			/* Link back into the page tables if not highmem. */
			if (!PageHighMem(page)) {
				int ret;

				ret = HYPERVISOR_update_va_mapping(
						(unsigned long)__va(pfn << PAGE_SHIFT),
						mfn_pte(frames[i], PAGE_KERNEL),
						0);
				BUG_ON(ret);
			}
		}
	}
#endif
}
EXPORT_SYMBOL(xenmem_reservation_va_mapping_update);

void xenmem_reservation_va_mapping_reset(unsigned long count,
					 struct page **pages)
{
#ifdef CONFIG_XEN_HAVE_PVMMU
	int i;

	for (i = 0; i < count; i++) {
		/*
		 * We don't support PV MMU when Linux and Xen is using
		 * different page granularity.
		 */
		BUILD_BUG_ON(XEN_PAGE_SIZE != PAGE_SIZE);

		if (!xen_feature(XENFEAT_auto_translated_physmap)) {
			struct page *page = pages[i];
			unsigned long pfn = page_to_pfn(page);

			if (!PageHighMem(page)) {
				int ret;

				ret = HYPERVISOR_update_va_mapping(
						(unsigned long)__va(pfn << PAGE_SHIFT),
						__pte_ma(0), 0);
				BUG_ON(ret);
			}
			__set_phys_to_machine(pfn, INVALID_P2M_ENTRY);
		}
	}
#endif
}
EXPORT_SYMBOL(xenmem_reservation_va_mapping_reset);

int xenmem_reservation_increase(int count, xen_pfn_t *frames)
{
	struct xen_memory_reservation reservation = {
		.address_bits = 0,
		.extent_order = EXTENT_ORDER,
		.domid        = DOMID_SELF
	};

	set_xen_guest_handle(reservation.extent_start, frames);
	reservation.nr_extents = count;
	return HYPERVISOR_memory_op(XENMEM_populate_physmap, &reservation);
}
EXPORT_SYMBOL(xenmem_reservation_increase);

int xenmem_reservation_decrease(int count, xen_pfn_t *frames)
{
	struct xen_memory_reservation reservation = {
		.address_bits = 0,
		.extent_order = EXTENT_ORDER,
		.domid        = DOMID_SELF
	};

	set_xen_guest_handle(reservation.extent_start, frames);
	reservation.nr_extents = count;
	return HYPERVISOR_memory_op(XENMEM_decrease_reservation, &reservation);
}
EXPORT_SYMBOL(xenmem_reservation_decrease);
