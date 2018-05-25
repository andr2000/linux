/******************************************************************************
 * gntdev.c
 *
 * Device for accessing (in user-space) pages that have been granted by other
 * domains.
 *
 * DMA buffer implementation is based on drivers/gpu/drm/drm_prime.c.
 *
 * Copyright (c) 2006-2007, D G Murray.
 *           (c) 2009 Gerd Hoffmann <kraxel@redhat.com>
 *           (c) 2018 Oleksandr Andrushchenko, EPAM Systems Inc.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#undef DEBUG

#define pr_fmt(fmt) "xen:" KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/mmu_notifier.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/refcount.h>
#ifdef CONFIG_XEN_GRANT_DMA_ALLOC
#include <linux/of_device.h>
#endif
#ifdef CONFIG_XEN_GNTDEV_DMABUF
#include <linux/dma-buf.h>
#endif

#include <xen/xen.h>
#include <xen/grant_table.h>
#include <xen/balloon.h>
#include <xen/gntdev.h>
#include <xen/events.h>
#include <xen/page.h>
#include <asm/xen/hypervisor.h>
#include <asm/xen/hypercall.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Derek G. Murray <Derek.Murray@cl.cam.ac.uk>, "
	      "Gerd Hoffmann <kraxel@redhat.com>");
MODULE_DESCRIPTION("User-space granted page access driver");

static int limit = 1024*1024;
module_param(limit, int, 0644);
MODULE_PARM_DESC(limit, "Maximum number of grants that may be mapped by "
		"the gntdev device");

static atomic_t pages_mapped = ATOMIC_INIT(0);

static int use_ptemod;
#define populate_freeable_maps use_ptemod

struct gntdev_priv {
	/* maps with visible offsets in the file descriptor */
	struct list_head maps;
	/* maps that are not visible; will be freed on munmap.
	 * Only populated if populate_freeable_maps == 1 */
	struct list_head freeable_maps;
	/* lock protects maps and freeable_maps */
	struct mutex lock;
	struct mm_struct *mm;
	struct mmu_notifier mn;

#ifdef CONFIG_XEN_GRANT_DMA_ALLOC
	/* Device for which DMA memory is allocated. */
	struct device *dma_dev;
#endif

#ifdef CONFIG_XEN_GNTDEV_DMABUF
	/* Private data of the hyper DMA buffers. */

	/* List of exported DMA buffers. */
	struct list_head dmabuf_exp_list;
	/* List of wait objects. */
	struct list_head dmabuf_exp_wait_list;
	/* This is the lock which protects dma_buf_xxx lists. */
	struct mutex dmabuf_lock;
#endif
};

struct unmap_notify {
	int flags;
	/* Address relative to the start of the grant_map */
	int addr;
	int event;
};

struct grant_map {
	struct list_head next;
	struct vm_area_struct *vma;
	int index;
	int count;
	int flags;
	refcount_t users;
	struct unmap_notify notify;
	struct ioctl_gntdev_grant_ref *grants;
	struct gnttab_map_grant_ref   *map_ops;
	struct gnttab_unmap_grant_ref *unmap_ops;
	struct gnttab_map_grant_ref   *kmap_ops;
	struct gnttab_unmap_grant_ref *kunmap_ops;
	struct page **pages;
	unsigned long pages_vm_start;

#ifdef CONFIG_XEN_GRANT_DMA_ALLOC
	/*
	 * If dmabuf_vaddr is not NULL then this mapping is backed by DMA
	 * capable memory.
	 */

	/* Device for which DMA memory is allocated. */
	struct device *dma_dev;
	/* Flags used to create this DMA buffer: GNTDEV_DMABUF_FLAG_XXX. */
	bool dma_flags;
	/* Virtual/CPU address of the DMA buffer. */
	void *dma_vaddr;
	/* Bus address of the DMA buffer. */
	dma_addr_t dma_bus_addr;
#endif
};

#ifdef CONFIG_XEN_GNTDEV_DMABUF
struct xen_dmabuf {
	struct gntdev_priv *priv;
	struct dma_buf *dmabuf;
	struct list_head next;
	int fd;

	union {
		struct {
			/* Exported buffers are reference counted. */
			struct kref refcount;
			struct grant_map *map;
		} exp;
		struct {
			/* Granted references of the imported buffer. */
			grant_ref_t *refs;
		} imp;
	} u;

	/* Number of pages this buffer has. */
	int nr_pages;
	/* Pages of this buffer. */
	struct page **pages;
};

struct xen_dmabuf_wait_obj {
	struct list_head next;
	struct xen_dmabuf *xen_dmabuf;
	struct completion completion;
};

struct xen_dmabuf_attachment {
	struct sg_table *sgt;
	enum dma_data_direction dir;
};
#endif

static int unmap_grant_pages(struct grant_map *map, int offset, int pages);

static struct miscdevice gntdev_miscdev;

/* ------------------------------------------------------------------ */

static void gntdev_print_maps(struct gntdev_priv *priv,
			      char *text, int text_index)
{
#ifdef DEBUG
	struct grant_map *map;

	pr_debug("%s: maps list (priv %p)\n", __func__, priv);
	list_for_each_entry(map, &priv->maps, next)
		pr_debug("  index %2d, count %2d %s\n",
		       map->index, map->count,
		       map->index == text_index && text ? text : "");
#endif
}

static void gntdev_free_map(struct grant_map *map)
{
	if (map == NULL)
		return;

#ifdef CONFIG_XEN_GRANT_DMA_ALLOC
	if (map->dma_vaddr) {
		struct gnttab_dma_alloc_args args;

		args.dev = map->dma_dev;
		args.coherent = map->dma_flags & GNTDEV_DMA_FLAG_COHERENT;
		args.nr_pages = map->count;
		args.pages = map->pages;
		args.vaddr = map->dma_vaddr;
		args.dev_bus_addr = map->dma_bus_addr;

		gnttab_dma_free_pages(&args);
	} else if (map->pages) {
		gnttab_free_pages(map->count, map->pages);
	}
#else
	if (map->pages)
		gnttab_free_pages(map->count, map->pages);
#endif

	kfree(map->pages);
	kfree(map->grants);
	kfree(map->map_ops);
	kfree(map->unmap_ops);
	kfree(map->kmap_ops);
	kfree(map->kunmap_ops);
	kfree(map);
}

static struct grant_map *gntdev_alloc_map(struct gntdev_priv *priv, int count,
					  int dma_flags)
{
	struct grant_map *add;
	int i;

	add = kzalloc(sizeof(struct grant_map), GFP_KERNEL);
	if (NULL == add)
		return NULL;

	add->grants    = kcalloc(count, sizeof(add->grants[0]), GFP_KERNEL);
	add->map_ops   = kcalloc(count, sizeof(add->map_ops[0]), GFP_KERNEL);
	add->unmap_ops = kcalloc(count, sizeof(add->unmap_ops[0]), GFP_KERNEL);
	add->kmap_ops  = kcalloc(count, sizeof(add->kmap_ops[0]), GFP_KERNEL);
	add->kunmap_ops = kcalloc(count, sizeof(add->kunmap_ops[0]), GFP_KERNEL);
	add->pages     = kcalloc(count, sizeof(add->pages[0]), GFP_KERNEL);
	if (NULL == add->grants    ||
	    NULL == add->map_ops   ||
	    NULL == add->unmap_ops ||
	    NULL == add->kmap_ops  ||
	    NULL == add->kunmap_ops ||
	    NULL == add->pages)
		goto err;

#ifdef CONFIG_XEN_GRANT_DMA_ALLOC
	add->dma_flags = dma_flags;

	/*
	 * Check if this mapping is requested to be backed
	 * by a DMA buffer.
	 */
	if (dma_flags & (GNTDEV_DMA_FLAG_WC | GNTDEV_DMA_FLAG_COHERENT)) {
		struct gnttab_dma_alloc_args args;

		/* Remember the device, so we can free DMA memory. */
		add->dma_dev = priv->dma_dev;

		args.dev = priv->dma_dev;
		args.coherent = dma_flags & GNTDEV_DMA_FLAG_COHERENT;
		args.nr_pages = count;
		args.pages = add->pages;

		if (gnttab_dma_alloc_pages(&args))
			goto err;

		add->dma_vaddr = args.vaddr;
		add->dma_bus_addr = args.dev_bus_addr;
	} else {
		if (gnttab_alloc_pages(count, add->pages))
			goto err;
	}
#else
	if (gnttab_alloc_pages(count, add->pages))
		goto err;
#endif

	for (i = 0; i < count; i++) {
		add->map_ops[i].handle = -1;
		add->unmap_ops[i].handle = -1;
		add->kmap_ops[i].handle = -1;
		add->kunmap_ops[i].handle = -1;
	}

	add->index = 0;
	add->count = count;
	refcount_set(&add->users, 1);

	return add;

err:
	gntdev_free_map(add);
	return NULL;
}

static void gntdev_add_map(struct gntdev_priv *priv, struct grant_map *add)
{
	struct grant_map *map;

	list_for_each_entry(map, &priv->maps, next) {
		if (add->index + add->count < map->index) {
			list_add_tail(&add->next, &map->next);
			goto done;
		}
		add->index = map->index + map->count;
	}
	list_add_tail(&add->next, &priv->maps);

done:
	gntdev_print_maps(priv, "[new]", add->index);
}

static struct grant_map *gntdev_find_map_index(struct gntdev_priv *priv,
		int index, int count)
{
	struct grant_map *map;

	list_for_each_entry(map, &priv->maps, next) {
		if (map->index != index)
			continue;
		if (count && map->count != count)
			continue;
		return map;
	}
	return NULL;
}

static void gntdev_put_map(struct gntdev_priv *priv, struct grant_map *map)
{
	if (!map)
		return;

	if (!refcount_dec_and_test(&map->users))
		return;

	atomic_sub(map->count, &pages_mapped);

	if (map->notify.flags & UNMAP_NOTIFY_SEND_EVENT) {
		notify_remote_via_evtchn(map->notify.event);
		evtchn_put(map->notify.event);
	}

	if (populate_freeable_maps && priv) {
		mutex_lock(&priv->lock);
		list_del(&map->next);
		mutex_unlock(&priv->lock);
	}

	if (map->pages && !use_ptemod)
		unmap_grant_pages(map, 0, map->count);
	gntdev_free_map(map);
}

#ifdef CONFIG_XEN_GNTDEV_DMABUF
static void gntdev_remove_map(struct gntdev_priv *priv, struct grant_map *map)
{
	mutex_lock(&priv->lock);
	list_del(&map->next);
	gntdev_put_map(NULL /* already removed */, map);
	mutex_unlock(&priv->lock);
}
#endif

/* ------------------------------------------------------------------ */

static int find_grant_ptes(pte_t *pte, pgtable_t token,
		unsigned long addr, void *data)
{
	struct grant_map *map = data;
	unsigned int pgnr = (addr - map->vma->vm_start) >> PAGE_SHIFT;
	int flags = map->flags | GNTMAP_application_map | GNTMAP_contains_pte;
	u64 pte_maddr;

	BUG_ON(pgnr >= map->count);
	pte_maddr = arbitrary_virt_to_machine(pte).maddr;

	/*
	 * Set the PTE as special to force get_user_pages_fast() fall
	 * back to the slow path.  If this is not supported as part of
	 * the grant map, it will be done afterwards.
	 */
	if (xen_feature(XENFEAT_gnttab_map_avail_bits))
		flags |= (1 << _GNTMAP_guest_avail0);

	gnttab_set_map_op(&map->map_ops[pgnr], pte_maddr, flags,
			  map->grants[pgnr].ref,
			  map->grants[pgnr].domid);
	gnttab_set_unmap_op(&map->unmap_ops[pgnr], pte_maddr, flags,
			    -1 /* handle */);
	return 0;
}

#ifdef CONFIG_X86
static int set_grant_ptes_as_special(pte_t *pte, pgtable_t token,
				     unsigned long addr, void *data)
{
	set_pte_at(current->mm, addr, pte, pte_mkspecial(*pte));
	return 0;
}
#endif

static int map_grant_pages(struct grant_map *map)
{
	int i, err = 0;

	if (!use_ptemod) {
		/* Note: it could already be mapped */
		if (map->map_ops[0].handle != -1)
			return 0;
		for (i = 0; i < map->count; i++) {
			unsigned long addr = (unsigned long)
				pfn_to_kaddr(page_to_pfn(map->pages[i]));
			gnttab_set_map_op(&map->map_ops[i], addr, map->flags,
				map->grants[i].ref,
				map->grants[i].domid);
			gnttab_set_unmap_op(&map->unmap_ops[i], addr,
				map->flags, -1 /* handle */);
		}
	} else {
		/*
		 * Setup the map_ops corresponding to the pte entries pointing
		 * to the kernel linear addresses of the struct pages.
		 * These ptes are completely different from the user ptes dealt
		 * with find_grant_ptes.
		 */
		for (i = 0; i < map->count; i++) {
			unsigned long address = (unsigned long)
				pfn_to_kaddr(page_to_pfn(map->pages[i]));
			BUG_ON(PageHighMem(map->pages[i]));

			gnttab_set_map_op(&map->kmap_ops[i], address,
				map->flags | GNTMAP_host_map,
				map->grants[i].ref,
				map->grants[i].domid);
			gnttab_set_unmap_op(&map->kunmap_ops[i], address,
				map->flags | GNTMAP_host_map, -1);
		}
	}

	pr_debug("map %d+%d\n", map->index, map->count);
	err = gnttab_map_refs(map->map_ops, use_ptemod ? map->kmap_ops : NULL,
			map->pages, map->count);
	if (err)
		return err;

	for (i = 0; i < map->count; i++) {
		if (map->map_ops[i].status) {
			err = -EINVAL;
			continue;
		}

		map->unmap_ops[i].handle = map->map_ops[i].handle;
#ifdef CONFIG_XEN_GRANT_DMA_ALLOC
		if (use_ptemod) {
			map->kunmap_ops[i].handle = map->kmap_ops[i].handle;
		} else if (map->dma_vaddr) {
			unsigned long mfn;

			mfn = __pfn_to_mfn(page_to_pfn(map->pages[i]));
			map->unmap_ops[i].dev_bus_addr = __pfn_to_phys(mfn);
		}
#else
		if (use_ptemod)
			map->kunmap_ops[i].handle = map->kmap_ops[i].handle;
#endif
	}
	return err;
}

static int __unmap_grant_pages(struct grant_map *map, int offset, int pages)
{
	int i, err = 0;
	struct gntab_unmap_queue_data unmap_data;

	if (map->notify.flags & UNMAP_NOTIFY_CLEAR_BYTE) {
		int pgno = (map->notify.addr >> PAGE_SHIFT);
		if (pgno >= offset && pgno < offset + pages) {
			/* No need for kmap, pages are in lowmem */
			uint8_t *tmp = pfn_to_kaddr(page_to_pfn(map->pages[pgno]));
			tmp[map->notify.addr & (PAGE_SIZE-1)] = 0;
			map->notify.flags &= ~UNMAP_NOTIFY_CLEAR_BYTE;
		}
	}

	unmap_data.unmap_ops = map->unmap_ops + offset;
	unmap_data.kunmap_ops = use_ptemod ? map->kunmap_ops + offset : NULL;
	unmap_data.pages = map->pages + offset;
	unmap_data.count = pages;

	err = gnttab_unmap_refs_sync(&unmap_data);
	if (err)
		return err;

	for (i = 0; i < pages; i++) {
		if (map->unmap_ops[offset+i].status)
			err = -EINVAL;
		pr_debug("unmap handle=%d st=%d\n",
			map->unmap_ops[offset+i].handle,
			map->unmap_ops[offset+i].status);
		map->unmap_ops[offset+i].handle = -1;
	}
	return err;
}

static int unmap_grant_pages(struct grant_map *map, int offset, int pages)
{
	int range, err = 0;

	pr_debug("unmap %d+%d [%d+%d]\n", map->index, map->count, offset, pages);

	/* It is possible the requested range will have a "hole" where we
	 * already unmapped some of the grants. Only unmap valid ranges.
	 */
	while (pages && !err) {
		while (pages && map->unmap_ops[offset].handle == -1) {
			offset++;
			pages--;
		}
		range = 0;
		while (range < pages) {
			if (map->unmap_ops[offset+range].handle == -1)
				break;
			range++;
		}
		err = __unmap_grant_pages(map, offset, range);
		offset += range;
		pages -= range;
	}

	return err;
}

/* ------------------------------------------------------------------ */

static void gntdev_vma_open(struct vm_area_struct *vma)
{
	struct grant_map *map = vma->vm_private_data;

	pr_debug("gntdev_vma_open %p\n", vma);
	refcount_inc(&map->users);
}

static void gntdev_vma_close(struct vm_area_struct *vma)
{
	struct grant_map *map = vma->vm_private_data;
	struct file *file = vma->vm_file;
	struct gntdev_priv *priv = file->private_data;

	pr_debug("gntdev_vma_close %p\n", vma);
	if (use_ptemod) {
		/* It is possible that an mmu notifier could be running
		 * concurrently, so take priv->lock to ensure that the vma won't
		 * vanishing during the unmap_grant_pages call, since we will
		 * spin here until that completes. Such a concurrent call will
		 * not do any unmapping, since that has been done prior to
		 * closing the vma, but it may still iterate the unmap_ops list.
		 */
		mutex_lock(&priv->lock);
		map->vma = NULL;
		mutex_unlock(&priv->lock);
	}
	vma->vm_private_data = NULL;
	gntdev_put_map(priv, map);
}

static struct page *gntdev_vma_find_special_page(struct vm_area_struct *vma,
						 unsigned long addr)
{
	struct grant_map *map = vma->vm_private_data;

	return map->pages[(addr - map->pages_vm_start) >> PAGE_SHIFT];
}

static const struct vm_operations_struct gntdev_vmops = {
	.open = gntdev_vma_open,
	.close = gntdev_vma_close,
	.find_special_page = gntdev_vma_find_special_page,
};

/* ------------------------------------------------------------------ */

static void unmap_if_in_range(struct grant_map *map,
			      unsigned long start, unsigned long end)
{
	unsigned long mstart, mend;
	int err;

	if (!map->vma)
		return;
	if (map->vma->vm_start >= end)
		return;
	if (map->vma->vm_end <= start)
		return;
	mstart = max(start, map->vma->vm_start);
	mend   = min(end,   map->vma->vm_end);
	pr_debug("map %d+%d (%lx %lx), range %lx %lx, mrange %lx %lx\n",
			map->index, map->count,
			map->vma->vm_start, map->vma->vm_end,
			start, end, mstart, mend);
	err = unmap_grant_pages(map,
				(mstart - map->vma->vm_start) >> PAGE_SHIFT,
				(mend - mstart) >> PAGE_SHIFT);
	WARN_ON(err);
}

static void mn_invl_range_start(struct mmu_notifier *mn,
				struct mm_struct *mm,
				unsigned long start, unsigned long end)
{
	struct gntdev_priv *priv = container_of(mn, struct gntdev_priv, mn);
	struct grant_map *map;

	mutex_lock(&priv->lock);
	list_for_each_entry(map, &priv->maps, next) {
		unmap_if_in_range(map, start, end);
	}
	list_for_each_entry(map, &priv->freeable_maps, next) {
		unmap_if_in_range(map, start, end);
	}
	mutex_unlock(&priv->lock);
}

static void mn_release(struct mmu_notifier *mn,
		       struct mm_struct *mm)
{
	struct gntdev_priv *priv = container_of(mn, struct gntdev_priv, mn);
	struct grant_map *map;
	int err;

	mutex_lock(&priv->lock);
	list_for_each_entry(map, &priv->maps, next) {
		if (!map->vma)
			continue;
		pr_debug("map %d+%d (%lx %lx)\n",
				map->index, map->count,
				map->vma->vm_start, map->vma->vm_end);
		err = unmap_grant_pages(map, /* offset */ 0, map->count);
		WARN_ON(err);
	}
	list_for_each_entry(map, &priv->freeable_maps, next) {
		if (!map->vma)
			continue;
		pr_debug("map %d+%d (%lx %lx)\n",
				map->index, map->count,
				map->vma->vm_start, map->vma->vm_end);
		err = unmap_grant_pages(map, /* offset */ 0, map->count);
		WARN_ON(err);
	}
	mutex_unlock(&priv->lock);
}

static const struct mmu_notifier_ops gntdev_mmu_ops = {
	.release                = mn_release,
	.invalidate_range_start = mn_invl_range_start,
};

/* ------------------------------------------------------------------ */

static int gntdev_open(struct inode *inode, struct file *flip)
{
	struct gntdev_priv *priv;
	int ret = 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	INIT_LIST_HEAD(&priv->maps);
	INIT_LIST_HEAD(&priv->freeable_maps);
	mutex_init(&priv->lock);

#ifdef CONFIG_XEN_GNTDEV_DMABUF
	mutex_init(&priv->dmabuf_lock);
	INIT_LIST_HEAD(&priv->dmabuf_exp_list);
	INIT_LIST_HEAD(&priv->dmabuf_exp_wait_list);
#endif

	if (use_ptemod) {
		priv->mm = get_task_mm(current);
		if (!priv->mm) {
			kfree(priv);
			return -ENOMEM;
		}
		priv->mn.ops = &gntdev_mmu_ops;
		ret = mmu_notifier_register(&priv->mn, priv->mm);
		mmput(priv->mm);
	}

	if (ret) {
		kfree(priv);
		return ret;
	}

	flip->private_data = priv;
#ifdef CONFIG_XEN_GRANT_DMA_ALLOC
	priv->dma_dev = gntdev_miscdev.this_device;

	/*
	 * The device is not spawn from a device tree, so arch_setup_dma_ops
	 * is not called, thus leaving the device with dummy DMA ops.
	 * Fix this call of_dma_configure() with a NULL node to set
	 * default DMA ops.
	 */
	of_dma_configure(priv->dma_dev, NULL);
#endif
	pr_debug("priv %p\n", priv);

	return 0;
}

static int gntdev_release(struct inode *inode, struct file *flip)
{
	struct gntdev_priv *priv = flip->private_data;
	struct grant_map *map;

	pr_debug("priv %p\n", priv);

	mutex_lock(&priv->lock);
	while (!list_empty(&priv->maps)) {
		map = list_entry(priv->maps.next, struct grant_map, next);
		list_del(&map->next);
		gntdev_put_map(NULL /* already removed */, map);
	}
	WARN_ON(!list_empty(&priv->freeable_maps));
	mutex_unlock(&priv->lock);

	if (use_ptemod)
		mmu_notifier_unregister(&priv->mn, priv->mm);
	kfree(priv);
	return 0;
}

static long gntdev_ioctl_map_grant_ref(struct gntdev_priv *priv,
				       struct ioctl_gntdev_map_grant_ref __user *u)
{
	struct ioctl_gntdev_map_grant_ref op;
	struct grant_map *map;
	int err;

	if (copy_from_user(&op, u, sizeof(op)) != 0)
		return -EFAULT;
	pr_debug("priv %p, add %d\n", priv, op.count);
	if (unlikely(op.count <= 0))
		return -EINVAL;

	err = -ENOMEM;
	map = gntdev_alloc_map(priv, op.count, 0 /* This is not a dma-buf. */);
	if (!map)
		return err;

	if (unlikely(atomic_add_return(op.count, &pages_mapped) > limit)) {
		pr_debug("can't map: over limit\n");
		gntdev_put_map(NULL, map);
		return err;
	}

	if (copy_from_user(map->grants, &u->refs,
			   sizeof(map->grants[0]) * op.count) != 0) {
		gntdev_put_map(NULL, map);
		return -EFAULT;
	}

	mutex_lock(&priv->lock);
	gntdev_add_map(priv, map);
	op.index = map->index << PAGE_SHIFT;
	mutex_unlock(&priv->lock);

	if (copy_to_user(u, &op, sizeof(op)) != 0)
		return -EFAULT;

	return 0;
}

static long gntdev_ioctl_unmap_grant_ref(struct gntdev_priv *priv,
					 struct ioctl_gntdev_unmap_grant_ref __user *u)
{
	struct ioctl_gntdev_unmap_grant_ref op;
	struct grant_map *map;
	int err = -ENOENT;

	if (copy_from_user(&op, u, sizeof(op)) != 0)
		return -EFAULT;
	pr_debug("priv %p, del %d+%d\n", priv, (int)op.index, (int)op.count);

	mutex_lock(&priv->lock);
	map = gntdev_find_map_index(priv, op.index >> PAGE_SHIFT, op.count);
	if (map) {
		list_del(&map->next);
		if (populate_freeable_maps)
			list_add_tail(&map->next, &priv->freeable_maps);
		err = 0;
	}
	mutex_unlock(&priv->lock);
	if (map)
		gntdev_put_map(priv, map);
	return err;
}

static long gntdev_ioctl_get_offset_for_vaddr(struct gntdev_priv *priv,
					      struct ioctl_gntdev_get_offset_for_vaddr __user *u)
{
	struct ioctl_gntdev_get_offset_for_vaddr op;
	struct vm_area_struct *vma;
	struct grant_map *map;
	int rv = -EINVAL;

	if (copy_from_user(&op, u, sizeof(op)) != 0)
		return -EFAULT;
	pr_debug("priv %p, offset for vaddr %lx\n", priv, (unsigned long)op.vaddr);

	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, op.vaddr);
	if (!vma || vma->vm_ops != &gntdev_vmops)
		goto out_unlock;

	map = vma->vm_private_data;
	if (!map)
		goto out_unlock;

	op.offset = map->index << PAGE_SHIFT;
	op.count = map->count;
	rv = 0;

 out_unlock:
	up_read(&current->mm->mmap_sem);

	if (rv == 0 && copy_to_user(u, &op, sizeof(op)) != 0)
		return -EFAULT;
	return rv;
}

static long gntdev_ioctl_notify(struct gntdev_priv *priv, void __user *u)
{
	struct ioctl_gntdev_unmap_notify op;
	struct grant_map *map;
	int rc;
	int out_flags;
	unsigned int out_event;

	if (copy_from_user(&op, u, sizeof(op)))
		return -EFAULT;

	if (op.action & ~(UNMAP_NOTIFY_CLEAR_BYTE|UNMAP_NOTIFY_SEND_EVENT))
		return -EINVAL;

	/* We need to grab a reference to the event channel we are going to use
	 * to send the notify before releasing the reference we may already have
	 * (if someone has called this ioctl twice). This is required so that
	 * it is possible to change the clear_byte part of the notification
	 * without disturbing the event channel part, which may now be the last
	 * reference to that event channel.
	 */
	if (op.action & UNMAP_NOTIFY_SEND_EVENT) {
		if (evtchn_get(op.event_channel_port))
			return -EINVAL;
	}

	out_flags = op.action;
	out_event = op.event_channel_port;

	mutex_lock(&priv->lock);

	list_for_each_entry(map, &priv->maps, next) {
		uint64_t begin = map->index << PAGE_SHIFT;
		uint64_t end = (map->index + map->count) << PAGE_SHIFT;
		if (op.index >= begin && op.index < end)
			goto found;
	}
	rc = -ENOENT;
	goto unlock_out;

 found:
	if ((op.action & UNMAP_NOTIFY_CLEAR_BYTE) &&
			(map->flags & GNTMAP_readonly)) {
		rc = -EINVAL;
		goto unlock_out;
	}

	out_flags = map->notify.flags;
	out_event = map->notify.event;

	map->notify.flags = op.action;
	map->notify.addr = op.index - (map->index << PAGE_SHIFT);
	map->notify.event = op.event_channel_port;

	rc = 0;

 unlock_out:
	mutex_unlock(&priv->lock);

	/* Drop the reference to the event channel we did not save in the map */
	if (out_flags & UNMAP_NOTIFY_SEND_EVENT)
		evtchn_put(out_event);

	return rc;
}

#define GNTDEV_COPY_BATCH 16

struct gntdev_copy_batch {
	struct gnttab_copy ops[GNTDEV_COPY_BATCH];
	struct page *pages[GNTDEV_COPY_BATCH];
	s16 __user *status[GNTDEV_COPY_BATCH];
	unsigned int nr_ops;
	unsigned int nr_pages;
};

static int gntdev_get_page(struct gntdev_copy_batch *batch, void __user *virt,
			   bool writeable, unsigned long *gfn)
{
	unsigned long addr = (unsigned long)virt;
	struct page *page;
	unsigned long xen_pfn;
	int ret;

	ret = get_user_pages_fast(addr, 1, writeable, &page);
	if (ret < 0)
		return ret;

	batch->pages[batch->nr_pages++] = page;

	xen_pfn = page_to_xen_pfn(page) + XEN_PFN_DOWN(addr & ~PAGE_MASK);
	*gfn = pfn_to_gfn(xen_pfn);

	return 0;
}

static void gntdev_put_pages(struct gntdev_copy_batch *batch)
{
	unsigned int i;

	for (i = 0; i < batch->nr_pages; i++)
		put_page(batch->pages[i]);
	batch->nr_pages = 0;
}

static int gntdev_copy(struct gntdev_copy_batch *batch)
{
	unsigned int i;

	gnttab_batch_copy(batch->ops, batch->nr_ops);
	gntdev_put_pages(batch);

	/*
	 * For each completed op, update the status if the op failed
	 * and all previous ops for the segment were successful.
	 */
	for (i = 0; i < batch->nr_ops; i++) {
		s16 status = batch->ops[i].status;
		s16 old_status;

		if (status == GNTST_okay)
			continue;

		if (__get_user(old_status, batch->status[i]))
			return -EFAULT;

		if (old_status != GNTST_okay)
			continue;

		if (__put_user(status, batch->status[i]))
			return -EFAULT;
	}

	batch->nr_ops = 0;
	return 0;
}

static int gntdev_grant_copy_seg(struct gntdev_copy_batch *batch,
				 struct gntdev_grant_copy_segment *seg,
				 s16 __user *status)
{
	uint16_t copied = 0;

	/*
	 * Disallow local -> local copies since there is only space in
	 * batch->pages for one page per-op and this would be a very
	 * expensive memcpy().
	 */
	if (!(seg->flags & (GNTCOPY_source_gref | GNTCOPY_dest_gref)))
		return -EINVAL;

	/* Can't cross page if source/dest is a grant ref. */
	if (seg->flags & GNTCOPY_source_gref) {
		if (seg->source.foreign.offset + seg->len > XEN_PAGE_SIZE)
			return -EINVAL;
	}
	if (seg->flags & GNTCOPY_dest_gref) {
		if (seg->dest.foreign.offset + seg->len > XEN_PAGE_SIZE)
			return -EINVAL;
	}

	if (put_user(GNTST_okay, status))
		return -EFAULT;

	while (copied < seg->len) {
		struct gnttab_copy *op;
		void __user *virt;
		size_t len, off;
		unsigned long gfn;
		int ret;

		if (batch->nr_ops >= GNTDEV_COPY_BATCH) {
			ret = gntdev_copy(batch);
			if (ret < 0)
				return ret;
		}

		len = seg->len - copied;

		op = &batch->ops[batch->nr_ops];
		op->flags = 0;

		if (seg->flags & GNTCOPY_source_gref) {
			op->source.u.ref = seg->source.foreign.ref;
			op->source.domid = seg->source.foreign.domid;
			op->source.offset = seg->source.foreign.offset + copied;
			op->flags |= GNTCOPY_source_gref;
		} else {
			virt = seg->source.virt + copied;
			off = (unsigned long)virt & ~XEN_PAGE_MASK;
			len = min(len, (size_t)XEN_PAGE_SIZE - off);

			ret = gntdev_get_page(batch, virt, false, &gfn);
			if (ret < 0)
				return ret;

			op->source.u.gmfn = gfn;
			op->source.domid = DOMID_SELF;
			op->source.offset = off;
		}

		if (seg->flags & GNTCOPY_dest_gref) {
			op->dest.u.ref = seg->dest.foreign.ref;
			op->dest.domid = seg->dest.foreign.domid;
			op->dest.offset = seg->dest.foreign.offset + copied;
			op->flags |= GNTCOPY_dest_gref;
		} else {
			virt = seg->dest.virt + copied;
			off = (unsigned long)virt & ~XEN_PAGE_MASK;
			len = min(len, (size_t)XEN_PAGE_SIZE - off);

			ret = gntdev_get_page(batch, virt, true, &gfn);
			if (ret < 0)
				return ret;

			op->dest.u.gmfn = gfn;
			op->dest.domid = DOMID_SELF;
			op->dest.offset = off;
		}

		op->len = len;
		copied += len;

		batch->status[batch->nr_ops] = status;
		batch->nr_ops++;
	}

	return 0;
}

static long gntdev_ioctl_grant_copy(struct gntdev_priv *priv, void __user *u)
{
	struct ioctl_gntdev_grant_copy copy;
	struct gntdev_copy_batch batch;
	unsigned int i;
	int ret = 0;

	if (copy_from_user(&copy, u, sizeof(copy)))
		return -EFAULT;

	batch.nr_ops = 0;
	batch.nr_pages = 0;

	for (i = 0; i < copy.count; i++) {
		struct gntdev_grant_copy_segment seg;

		if (copy_from_user(&seg, &copy.segments[i], sizeof(seg))) {
			ret = -EFAULT;
			goto out;
		}

		ret = gntdev_grant_copy_seg(&batch, &seg, &copy.segments[i].status);
		if (ret < 0)
			goto out;

		cond_resched();
	}
	if (batch.nr_ops)
		ret = gntdev_copy(&batch);
	return ret;

  out:
	gntdev_put_pages(&batch);
	return ret;
}

#ifdef CONFIG_XEN_GNTDEV_DMABUF
/* ------------------------------------------------------------------ */
/* DMA buffer export support.                                         */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* Implementation of wait for exported DMA buffer to be released.     */
/* ------------------------------------------------------------------ */

static void dmabuf_exp_release(struct kref *kref);

static struct xen_dmabuf_wait_obj *
dmabuf_exp_wait_obj_new(struct gntdev_priv *priv,
			struct xen_dmabuf *xen_dmabuf)
{
	struct xen_dmabuf_wait_obj *obj;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		return ERR_PTR(-ENOMEM);

	init_completion(&obj->completion);
	obj->xen_dmabuf = xen_dmabuf;

	mutex_lock(&priv->dmabuf_lock);
	list_add(&obj->next, &priv->dmabuf_exp_wait_list);
	/* Put our reference and wait for xen_dmabuf's release to fire. */
	kref_put(&xen_dmabuf->u.exp.refcount, dmabuf_exp_release);
	mutex_unlock(&priv->dmabuf_lock);
	return obj;
}

static void dmabuf_exp_wait_obj_free(struct gntdev_priv *priv,
				     struct xen_dmabuf_wait_obj *obj)
{
	struct xen_dmabuf_wait_obj *cur_obj, *q;

	mutex_lock(&priv->dmabuf_lock);
	list_for_each_entry_safe(cur_obj, q, &priv->dmabuf_exp_wait_list, next)
		if (cur_obj == obj) {
			list_del(&obj->next);
			kfree(obj);
			break;
		}
	mutex_unlock(&priv->dmabuf_lock);
}

static int dmabuf_exp_wait_obj_wait(struct xen_dmabuf_wait_obj *obj,
				    u32 wait_to_ms)
{
	if (wait_for_completion_timeout(&obj->completion,
			msecs_to_jiffies(wait_to_ms)) <= 0)
		return -ETIMEDOUT;

	return 0;
}

static void dmabuf_exp_wait_obj_signal(struct gntdev_priv *priv,
				       struct xen_dmabuf *xen_dmabuf)
{
	struct xen_dmabuf_wait_obj *obj, *q;

	list_for_each_entry_safe(obj, q, &priv->dmabuf_exp_wait_list, next)
		if (obj->xen_dmabuf == xen_dmabuf) {
			pr_debug("Found xen_dmabuf in the wait list, wake\n");
			complete_all(&obj->completion);
		}
}

static struct xen_dmabuf *
dmabuf_exp_wait_obj_get_by_fd(struct gntdev_priv *priv, int fd)
{
	struct xen_dmabuf *q, *xen_dmabuf, *ret = ERR_PTR(-ENOENT);

	mutex_lock(&priv->dmabuf_lock);
	list_for_each_entry_safe(xen_dmabuf, q, &priv->dmabuf_exp_list, next)
		if (xen_dmabuf->fd == fd) {
			pr_debug("Found xen_dmabuf in the wait list\n");
			kref_get(&xen_dmabuf->u.exp.refcount);
			ret = xen_dmabuf;
			break;
		}
	mutex_unlock(&priv->dmabuf_lock);
	return ret;
}

static int dmabuf_exp_wait_released(struct gntdev_priv *priv, int fd,
				    int wait_to_ms)
{
	struct xen_dmabuf *xen_dmabuf;
	struct xen_dmabuf_wait_obj *obj;
	int ret;

	pr_debug("Will wait for dma-buf with fd %d\n", fd);
	/*
	 * Try to find the DMA buffer: if not found means that
	 * either the buffer has already been released or file descriptor
	 * provided is wrong.
	 */
	xen_dmabuf = dmabuf_exp_wait_obj_get_by_fd(priv, fd);
	if (IS_ERR(xen_dmabuf))
		return PTR_ERR(xen_dmabuf);

	/*
	 * xen_dmabuf still exists and is reference count locked by us now,
	 * so prepare to wait: allocate wait object and add it to the wait list,
	 * so we can find it on release.
	 */
	obj = dmabuf_exp_wait_obj_new(priv, xen_dmabuf);
	if (IS_ERR(obj)) {
		pr_err("Failed to setup wait object, ret %ld\n", PTR_ERR(obj));
		return PTR_ERR(obj);
	}

	ret = dmabuf_exp_wait_obj_wait(obj, wait_to_ms);
	dmabuf_exp_wait_obj_free(priv, obj);
	return ret;
}

/* ------------------------------------------------------------------ */
/* DMA buffer export support.                                         */
/* ------------------------------------------------------------------ */

static struct sg_table *
dmabuf_pages_to_sgt(struct page **pages, unsigned int nr_pages)
{
	struct sg_table *sgt;
	int ret;

	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		ret = -ENOMEM;
		goto out;
	}

	ret = sg_alloc_table_from_pages(sgt, pages, nr_pages, 0,
					nr_pages << PAGE_SHIFT,
					GFP_KERNEL);
	if (ret)
		goto out;

	return sgt;

out:
	kfree(sgt);
	return ERR_PTR(ret);
}

static int dmabuf_exp_ops_attach(struct dma_buf *dma_buf,
				 struct device *target_dev,
				 struct dma_buf_attachment *attach)
{
	struct xen_dmabuf_attachment *xen_dmabuf_attach;

	xen_dmabuf_attach = kzalloc(sizeof(*xen_dmabuf_attach), GFP_KERNEL);
	if (!xen_dmabuf_attach)
		return -ENOMEM;

	xen_dmabuf_attach->dir = DMA_NONE;
	attach->priv = xen_dmabuf_attach;
	/* Might need to pin the pages of the buffer now. */
	return 0;
}

static void dmabuf_exp_ops_detach(struct dma_buf *dma_buf,
				  struct dma_buf_attachment *attach)
{
	struct xen_dmabuf_attachment *xen_dmabuf_attach = attach->priv;

	if (xen_dmabuf_attach) {
		struct sg_table *sgt = xen_dmabuf_attach->sgt;

		if (sgt) {
			if (xen_dmabuf_attach->dir != DMA_NONE)
				dma_unmap_sg_attrs(attach->dev, sgt->sgl,
						   sgt->nents,
						   xen_dmabuf_attach->dir,
						   DMA_ATTR_SKIP_CPU_SYNC);
			sg_free_table(sgt);
		}

		kfree(sgt);
		kfree(xen_dmabuf_attach);
		attach->priv = NULL;
	}
	/* Might need to unpin the pages of the buffer now. */
}

static struct sg_table *
dmabuf_exp_ops_map_dma_buf(struct dma_buf_attachment *attach,
			   enum dma_data_direction dir)
{
	struct xen_dmabuf_attachment *xen_dmabuf_attach = attach->priv;
	struct xen_dmabuf *xen_dmabuf = attach->dmabuf->priv;
	struct sg_table *sgt;

	pr_debug("Mapping %d pages for dev %p\n", xen_dmabuf->nr_pages,
		 attach->dev);

	if (WARN_ON(dir == DMA_NONE || !xen_dmabuf_attach))
		return ERR_PTR(-EINVAL);

	/* Return the cached mapping when possible. */
	if (xen_dmabuf_attach->dir == dir)
		return xen_dmabuf_attach->sgt;

	/*
	 * Two mappings with different directions for the same attachment are
	 * not allowed.
	 */
	if (WARN_ON(xen_dmabuf_attach->dir != DMA_NONE))
		return ERR_PTR(-EBUSY);

	sgt = dmabuf_pages_to_sgt(xen_dmabuf->pages, xen_dmabuf->nr_pages);
	if (!IS_ERR(sgt)) {
		if (!dma_map_sg_attrs(attach->dev, sgt->sgl, sgt->nents, dir,
				      DMA_ATTR_SKIP_CPU_SYNC)) {
			sg_free_table(sgt);
			kfree(sgt);
			sgt = ERR_PTR(-ENOMEM);
		} else {
			xen_dmabuf_attach->sgt = sgt;
			xen_dmabuf_attach->dir = dir;
		}
	}
	if (IS_ERR(sgt))
		pr_err("Failed to map sg table for dev %p\n", attach->dev);
	return sgt;
}

static void dmabuf_exp_ops_unmap_dma_buf(struct dma_buf_attachment *attach,
					 struct sg_table *sgt,
					 enum dma_data_direction dir)
{
	/* Not implemented. The unmap is done at dmabuf_exp_ops_detach(). */
}

static void dmabuf_exp_release(struct kref *kref)
{
	struct xen_dmabuf *xen_dmabuf =
		container_of(kref, struct xen_dmabuf, u.exp.refcount);

	dmabuf_exp_wait_obj_signal(xen_dmabuf->priv, xen_dmabuf);
	list_del(&xen_dmabuf->next);
	kfree(xen_dmabuf);
}

static void dmabuf_exp_ops_release(struct dma_buf *dma_buf)
{
	struct xen_dmabuf *xen_dmabuf = dma_buf->priv;
	struct gntdev_priv *priv = xen_dmabuf->priv;

	gntdev_remove_map(priv, xen_dmabuf->u.exp.map);
	mutex_lock(&priv->dmabuf_lock);
	kref_put(&xen_dmabuf->u.exp.refcount, dmabuf_exp_release);
	mutex_unlock(&priv->dmabuf_lock);
}

static void *dmabuf_exp_ops_kmap_atomic(struct dma_buf *dma_buf,
					unsigned long page_num)
{
	/* Not implemented. */
	return NULL;
}

static void dmabuf_exp_ops_kunmap_atomic(struct dma_buf *dma_buf,
					 unsigned long page_num, void *addr)
{
	/* Not implemented. */
}

static void *dmabuf_exp_ops_kmap(struct dma_buf *dma_buf,
				 unsigned long page_num)
{
	/* Not implemented. */
	return NULL;
}

static void dmabuf_exp_ops_kunmap(struct dma_buf *dma_buf,
				  unsigned long page_num, void *addr)
{
	/* Not implemented. */
}

static int dmabuf_exp_ops_mmap(struct dma_buf *dma_buf,
			       struct vm_area_struct *vma)
{
	/* Not implemented. */
	return 0;
}

static const struct dma_buf_ops dmabuf_exp_ops =  {
	.attach = dmabuf_exp_ops_attach,
	.detach = dmabuf_exp_ops_detach,
	.map_dma_buf = dmabuf_exp_ops_map_dma_buf,
	.unmap_dma_buf = dmabuf_exp_ops_unmap_dma_buf,
	.release = dmabuf_exp_ops_release,
	.map = dmabuf_exp_ops_kmap,
	.map_atomic = dmabuf_exp_ops_kmap_atomic,
	.unmap = dmabuf_exp_ops_kunmap,
	.unmap_atomic = dmabuf_exp_ops_kunmap_atomic,
	.mmap = dmabuf_exp_ops_mmap,
};

static int dmabuf_export(struct gntdev_priv *priv, struct grant_map *map,
			 int *fd)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct xen_dmabuf *xen_dmabuf;
	int ret = 0;

	xen_dmabuf = kzalloc(sizeof(*xen_dmabuf), GFP_KERNEL);
	if (!xen_dmabuf)
		return -ENOMEM;

	kref_init(&xen_dmabuf->u.exp.refcount);

	xen_dmabuf->priv = priv;
	xen_dmabuf->nr_pages = map->count;
	xen_dmabuf->pages = map->pages;
	xen_dmabuf->u.exp.map = map;

	exp_info.exp_name = KBUILD_MODNAME;
	if (map->dma_dev->driver && map->dma_dev->driver->owner)
		exp_info.owner = map->dma_dev->driver->owner;
	else
		exp_info.owner = THIS_MODULE;
	exp_info.ops = &dmabuf_exp_ops;
	exp_info.size = map->count << PAGE_SHIFT;
	exp_info.flags = O_RDWR;
	exp_info.priv = xen_dmabuf;

	xen_dmabuf->dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(xen_dmabuf->dmabuf)) {
		ret = PTR_ERR(xen_dmabuf->dmabuf);
		xen_dmabuf->dmabuf = NULL;
		goto fail;
	}

	ret = dma_buf_fd(xen_dmabuf->dmabuf, O_CLOEXEC);
	if (ret < 0)
		goto fail;

	xen_dmabuf->fd = ret;
	*fd = ret;

	pr_debug("Exporting DMA buffer with fd %d\n", ret);

	mutex_lock(&priv->dmabuf_lock);
	list_add(&xen_dmabuf->next, &priv->dmabuf_exp_list);
	mutex_unlock(&priv->dmabuf_lock);
	return 0;

fail:
	if (xen_dmabuf->dmabuf)
		dma_buf_put(xen_dmabuf->dmabuf);
	kfree(xen_dmabuf);
	return ret;
}

static struct grant_map *
dmabuf_exp_alloc_backing_storage(struct gntdev_priv *priv, int dmabuf_flags,
				 int count)
{
	struct grant_map *map;

	if (unlikely(count <= 0))
		return ERR_PTR(-EINVAL);

	if ((dmabuf_flags & GNTDEV_DMA_FLAG_WC) &&
	    (dmabuf_flags & GNTDEV_DMA_FLAG_COHERENT)) {
		pr_err("Wrong dma-buf flags: either WC or coherent, not both\n");
		return ERR_PTR(-EINVAL);
	}

	map = gntdev_alloc_map(priv, count, dmabuf_flags);
	if (!map)
		return ERR_PTR(-ENOMEM);

	if (unlikely(atomic_add_return(count, &pages_mapped) > limit)) {
		pr_err("can't map: over limit\n");
		gntdev_put_map(NULL, map);
		return ERR_PTR(-ENOMEM);
	}
	return map;
}

static int dmabuf_exp_from_refs(struct gntdev_priv *priv, int flags,
				int count, u32 domid, u32 *refs, u32 *fd)
{
	struct grant_map *map;
	int i, ret;

	*fd = -1;

	if (use_ptemod) {
		pr_err("Cannot provide dma-buf: use_ptemode %d\n",
		       use_ptemod);
		return -EINVAL;
	}

	map = dmabuf_exp_alloc_backing_storage(priv, flags, count);
	if (IS_ERR(map))
		return PTR_ERR(map);

	for (i = 0; i < count; i++) {
		map->grants[i].domid = domid;
		map->grants[i].ref = refs[i];
	}

	mutex_lock(&priv->lock);
	gntdev_add_map(priv, map);
	mutex_unlock(&priv->lock);

	map->flags |= GNTMAP_host_map;
#if defined(CONFIG_X86)
	map->flags |= GNTMAP_device_map;
#endif

	ret = map_grant_pages(map);
	if (ret < 0)
		goto out;

	ret = dmabuf_export(priv, map, fd);
	if (ret < 0)
		goto out;

	return 0;

out:
	gntdev_remove_map(priv, map);
	return ret;
}

/* ------------------------------------------------------------------ */
/* DMA buffer import support.                                         */
/* ------------------------------------------------------------------ */

static int dmabuf_imp_release(struct gntdev_priv *priv, u32 fd)
{
	return 0;
}

static struct xen_dmabuf *
dmabuf_imp_to_refs(struct gntdev_priv *priv, int fd, int count, int domid)
{
	return ERR_PTR(-ENOMEM);
}

/* ------------------------------------------------------------------ */
/* DMA buffer IOCTL support.                                          */
/* ------------------------------------------------------------------ */

static long
gntdev_ioctl_dmabuf_exp_from_refs(struct gntdev_priv *priv,
				  struct ioctl_gntdev_dmabuf_exp_from_refs __user *u)
{
	struct ioctl_gntdev_dmabuf_exp_from_refs op;
	u32 *refs;
	long ret;

	if (copy_from_user(&op, u, sizeof(op)) != 0)
		return -EFAULT;

	refs = kcalloc(op.count, sizeof(*refs), GFP_KERNEL);
	if (!refs)
		return -ENOMEM;

	if (copy_from_user(refs, u->refs, sizeof(*refs) * op.count) != 0) {
		ret = -EFAULT;
		goto out;
	}

	ret = dmabuf_exp_from_refs(priv, op.flags, op.count,
				   op.domid, refs, &op.fd);
	if (ret)
		goto out;

	if (copy_to_user(u, &op, sizeof(op)) != 0)
		ret = -EFAULT;

out:
	kfree(refs);
	return ret;
}

static long
gntdev_ioctl_dmabuf_exp_wait_released(struct gntdev_priv *priv,
				      struct ioctl_gntdev_dmabuf_exp_wait_released __user *u)
{
	struct ioctl_gntdev_dmabuf_exp_wait_released op;

	if (copy_from_user(&op, u, sizeof(op)) != 0)
		return -EFAULT;

	return dmabuf_exp_wait_released(priv, op.fd, op.wait_to_ms);
}

static long
gntdev_ioctl_dmabuf_imp_to_refs(struct gntdev_priv *priv,
				struct ioctl_gntdev_dmabuf_imp_to_refs __user *u)
{
	struct ioctl_gntdev_dmabuf_imp_to_refs op;
	struct xen_dmabuf *xen_dmabuf;
	long ret;

	if (copy_from_user(&op, u, sizeof(op)) != 0)
		return -EFAULT;

	xen_dmabuf = dmabuf_imp_to_refs(priv, op.fd, op.count, op.domid);
	if (IS_ERR(xen_dmabuf))
		return PTR_ERR(xen_dmabuf);

	if (copy_to_user(u->refs, xen_dmabuf->u.imp.refs,
			 sizeof(*u->refs) * op.count) != 0) {
		ret = -EFAULT;
		goto out_release;
	}
	return 0;

out_release:
	dmabuf_imp_release(priv, op.fd);
	return ret;
}

static long
gntdev_ioctl_dmabuf_imp_release(struct gntdev_priv *priv,
				struct ioctl_gntdev_dmabuf_imp_release __user *u)
{
	struct ioctl_gntdev_dmabuf_imp_release op;

	if (copy_from_user(&op, u, sizeof(op)) != 0)
		return -EFAULT;

	return dmabuf_imp_release(priv, op.fd);
}
#endif

static long gntdev_ioctl(struct file *flip,
			 unsigned int cmd, unsigned long arg)
{
	struct gntdev_priv *priv = flip->private_data;
	void __user *ptr = (void __user *)arg;

	switch (cmd) {
	case IOCTL_GNTDEV_MAP_GRANT_REF:
		return gntdev_ioctl_map_grant_ref(priv, ptr);

	case IOCTL_GNTDEV_UNMAP_GRANT_REF:
		return gntdev_ioctl_unmap_grant_ref(priv, ptr);

	case IOCTL_GNTDEV_GET_OFFSET_FOR_VADDR:
		return gntdev_ioctl_get_offset_for_vaddr(priv, ptr);

	case IOCTL_GNTDEV_SET_UNMAP_NOTIFY:
		return gntdev_ioctl_notify(priv, ptr);

	case IOCTL_GNTDEV_GRANT_COPY:
		return gntdev_ioctl_grant_copy(priv, ptr);

#ifdef CONFIG_XEN_GNTDEV_DMABUF
	case IOCTL_GNTDEV_DMABUF_EXP_FROM_REFS:
		return gntdev_ioctl_dmabuf_exp_from_refs(priv, ptr);

	case IOCTL_GNTDEV_DMABUF_EXP_WAIT_RELEASED:
		return gntdev_ioctl_dmabuf_exp_wait_released(priv, ptr);

	case IOCTL_GNTDEV_DMABUF_IMP_TO_REFS:
		return gntdev_ioctl_dmabuf_imp_to_refs(priv, ptr);

	case IOCTL_GNTDEV_DMABUF_IMP_RELEASE:
		return gntdev_ioctl_dmabuf_imp_release(priv, ptr);
#endif

	default:
		pr_debug("priv %p, unknown cmd %x\n", priv, cmd);
		return -ENOIOCTLCMD;
	}

	return 0;
}

static int gntdev_mmap(struct file *flip, struct vm_area_struct *vma)
{
	struct gntdev_priv *priv = flip->private_data;
	int index = vma->vm_pgoff;
	int count = vma_pages(vma);
	struct grant_map *map;
	int i, err = -EINVAL;

	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED))
		return -EINVAL;

	pr_debug("map %d+%d at %lx (pgoff %lx)\n",
			index, count, vma->vm_start, vma->vm_pgoff);

	mutex_lock(&priv->lock);
	map = gntdev_find_map_index(priv, index, count);
	if (!map)
		goto unlock_out;
	if (use_ptemod && map->vma)
		goto unlock_out;
	if (use_ptemod && priv->mm != vma->vm_mm) {
		pr_warn("Huh? Other mm?\n");
		goto unlock_out;
	}

	refcount_inc(&map->users);

	vma->vm_ops = &gntdev_vmops;

	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP | VM_MIXEDMAP;

	if (use_ptemod)
		vma->vm_flags |= VM_DONTCOPY;

	vma->vm_private_data = map;

	if (use_ptemod)
		map->vma = vma;

	if (map->flags) {
		if ((vma->vm_flags & VM_WRITE) &&
				(map->flags & GNTMAP_readonly))
			goto out_unlock_put;
	} else {
		map->flags = GNTMAP_host_map;
		if (!(vma->vm_flags & VM_WRITE))
			map->flags |= GNTMAP_readonly;
	}

	mutex_unlock(&priv->lock);

	if (use_ptemod) {
		map->pages_vm_start = vma->vm_start;
		err = apply_to_page_range(vma->vm_mm, vma->vm_start,
					  vma->vm_end - vma->vm_start,
					  find_grant_ptes, map);
		if (err) {
			pr_warn("find_grant_ptes() failure.\n");
			goto out_put_map;
		}
	}

	err = map_grant_pages(map);
	if (err)
		goto out_put_map;

	if (!use_ptemod) {
		for (i = 0; i < count; i++) {
			err = vm_insert_page(vma, vma->vm_start + i*PAGE_SIZE,
				map->pages[i]);
			if (err)
				goto out_put_map;
		}
	} else {
#ifdef CONFIG_X86
		/*
		 * If the PTEs were not made special by the grant map
		 * hypercall, do so here.
		 *
		 * This is racy since the mapping is already visible
		 * to userspace but userspace should be well-behaved
		 * enough to not touch it until the mmap() call
		 * returns.
		 */
		if (!xen_feature(XENFEAT_gnttab_map_avail_bits)) {
			apply_to_page_range(vma->vm_mm, vma->vm_start,
					    vma->vm_end - vma->vm_start,
					    set_grant_ptes_as_special, NULL);
		}
#endif
	}

	return 0;

unlock_out:
	mutex_unlock(&priv->lock);
	return err;

out_unlock_put:
	mutex_unlock(&priv->lock);
out_put_map:
	if (use_ptemod) {
		map->vma = NULL;
		unmap_grant_pages(map, 0, map->count);
	}
	gntdev_put_map(priv, map);
	return err;
}

static const struct file_operations gntdev_fops = {
	.owner = THIS_MODULE,
	.open = gntdev_open,
	.release = gntdev_release,
	.mmap = gntdev_mmap,
	.unlocked_ioctl = gntdev_ioctl
};

static struct miscdevice gntdev_miscdev = {
	.minor        = MISC_DYNAMIC_MINOR,
	.name         = "xen/gntdev",
	.fops         = &gntdev_fops,
};

/* ------------------------------------------------------------------ */

static int __init gntdev_init(void)
{
	int err;

	if (!xen_domain())
		return -ENODEV;

	use_ptemod = !xen_feature(XENFEAT_auto_translated_physmap);

	err = misc_register(&gntdev_miscdev);
	if (err != 0) {
		pr_err("Could not register gntdev device\n");
		return err;
	}
	return 0;
}

static void __exit gntdev_exit(void)
{
	misc_deregister(&gntdev_miscdev);
}

module_init(gntdev_init);
module_exit(gntdev_exit);

/* ------------------------------------------------------------------ */
