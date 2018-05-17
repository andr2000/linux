/******************************************************************************
 * Xen balloon functionality
 */

#define RETRY_UNLIMITED	0

struct balloon_stats {
	/* We aim for 'current allocation' == 'target allocation'. */
	unsigned long current_pages;
	unsigned long target_pages;
	unsigned long target_unpopulated;
	/* Number of pages in high- and low-memory balloons. */
	unsigned long balloon_low;
	unsigned long balloon_high;
	unsigned long total_pages;
	unsigned long schedule_delay;
	unsigned long max_schedule_delay;
	unsigned long retry_count;
	unsigned long max_retry_count;
	unsigned long dma_pages;
};

extern struct balloon_stats balloon_stats;

struct device;

void balloon_set_new_target(unsigned long target);

int alloc_xenballooned_pages(int nr_pages, struct page **pages);
void free_xenballooned_pages(int nr_pages, struct page **pages);

int alloc_dma_xenballooned_pages(struct device *dev, bool coherent,
				 int nr_pages, struct page **pages,
				 void **vaddr, dma_addr_t *dev_bus_addr);
void free_dma_xenballooned_pages(struct device *dev, bool coherent,
				 int nr_pages, struct page **pages,
				 void *vaddr, dma_addr_t dev_bus_addr);

#ifdef CONFIG_XEN_SELFBALLOONING
extern int register_xen_selfballooning(struct device *dev);
#else
static inline int register_xen_selfballooning(struct device *dev)
{
	return -ENOSYS;
}
#endif
