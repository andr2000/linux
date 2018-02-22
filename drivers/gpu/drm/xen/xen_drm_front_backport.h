/*
 *  Xen para-virtual DRM device
 *
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_DRM_FRONT_BACKPORT_H_
#define __XEN_DRM_FRONT_BACKPORT_H_

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>

#include <linux/version.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,10,0)

#define PV_DRM_LINUX_VERSION	KERNEL_VERSION(4,10,0)

void drm_atomic_helper_shutdown(struct drm_device *dev);

int drm_gem_fb_prepare_fb(struct drm_plane *plane,
			  struct drm_plane_state *state);

static inline int drm_dev_is_unplugged(struct drm_device *dev)
{
	int ret = atomic_read(&dev->unplugged);
	smp_rmb();
	return ret;
}

static inline void *kvmalloc_array(size_t n, size_t size, gfp_t flags)
{
	return drm_malloc_ab(n ,size);
}

#ifndef CONFIG_DRM_XEN_FRONTEND_CMA
struct xen_gem_object;

struct drm_framebuffer *xen_drm_front_gem_fb_create_with_funcs(struct drm_device *dev,
		struct drm_file *file_priv, const struct drm_mode_fb_cmd2 *mode_cmd,
		const struct drm_framebuffer_funcs *funcs);

void xen_drm_front_gem_fb_destroy(struct drm_framebuffer *fb);

int xen_drm_front_gem_dumb_map_offset(struct drm_file *file_priv,
		struct drm_device *dev, uint32_t handle, uint64_t *offset);

#endif

int xen_drm_front_dumb_map_offset(struct drm_file *file_priv,
		struct drm_device *dev, uint32_t handle, uint64_t *offset);

int xen_drm_front_enable_vblank(struct drm_device *dev, unsigned int pipe);

void xen_drm_front_disable_vblank(struct drm_device *dev, unsigned int pipe);

#endif

#endif /* __XEN_DRM_FRONT_BACKPORT_H_ */
