#include <drm/drmP.h>
#include <drm/drm_gem.h>

#include "xen_drm_front_backport.h"
#include "xen_drm_front_drv.h"
#include "xen_drm_front_gem.h"

#ifndef CONFIG_DRM_XEN_FRONTEND_CMA

struct xen_fb {
	struct drm_framebuffer fb;
	struct xen_gem_object *xen_obj;
};

static inline struct xen_fb *to_xen_fb(struct drm_framebuffer *fb)
{
	return container_of(fb, struct xen_fb, fb);
}

static struct xen_fb *gem_fb_alloc(struct drm_device *dev,
		const struct drm_mode_fb_cmd2 *mode_cmd,
		struct xen_gem_object *xen_obj,
		const struct drm_framebuffer_funcs *funcs)
{
	struct xen_fb *xen_fb;
	int ret;

	xen_fb = kzalloc(sizeof(*xen_fb), GFP_KERNEL);
	if (!xen_fb)
		return ERR_PTR(-ENOMEM);

	drm_helper_mode_fill_fb_struct(&xen_fb->fb, mode_cmd);
	xen_fb->xen_obj = xen_obj;
	ret = drm_framebuffer_init(dev, &xen_fb->fb, funcs);
	if (ret < 0) {
		DRM_ERROR("Failed to initialize framebuffer: %d\n", ret);
		kfree(xen_fb);
		return ERR_PTR(ret);
	}

	return xen_fb;
}

struct drm_framebuffer *xen_drm_front_gem_fb_create_with_funcs(struct drm_device *dev,
		struct drm_file *file_priv,
		const struct drm_mode_fb_cmd2 *mode_cmd,
		const struct drm_framebuffer_funcs *funcs)
{
	struct xen_fb *xen_fb;
	struct xen_gem_object *xen_obj;
	struct drm_gem_object *gem_obj;
	unsigned int hsub;
	unsigned int vsub;
	unsigned int min_size;
	int ret;

	/* we do not support formats that require more than 1 plane */
	if (drm_format_num_planes(mode_cmd->pixel_format) != 1) {
		DRM_ERROR("Unsupported pixel format 0x%04x\n",
			mode_cmd->pixel_format);
		return ERR_PTR(-EINVAL);
	}

	hsub = drm_format_horz_chroma_subsampling(mode_cmd->pixel_format);
	vsub = drm_format_vert_chroma_subsampling(mode_cmd->pixel_format);

	gem_obj = drm_gem_object_lookup(file_priv, mode_cmd->handles[0]);
	if (!gem_obj) {
		DRM_ERROR("Failed to lookup GEM object\n");
		return ERR_PTR(-ENXIO);
	}

	min_size = (mode_cmd->height - 1) * mode_cmd->pitches[0] +
		mode_cmd->width *
		drm_format_plane_cpp(mode_cmd->pixel_format, 0) +
		mode_cmd->offsets[0];
	if (gem_obj->size < min_size) {
		drm_gem_object_unreference_unlocked(gem_obj);
		return ERR_PTR(-EINVAL);
	}
	xen_obj = to_xen_gem_obj(gem_obj);

	xen_fb = gem_fb_alloc(dev, mode_cmd, xen_obj, funcs);
	if (IS_ERR(xen_fb)) {
		ret = PTR_ERR(xen_fb);
		goto fail;
	}

	return &xen_fb->fb;

fail:
	drm_gem_object_unreference_unlocked(gem_obj);
	return ERR_PTR(ret);
}

void xen_drm_front_gem_fb_destroy(struct drm_framebuffer *fb)
{
	struct xen_fb *xen_fb = to_xen_fb(fb);

	if (xen_fb->xen_obj)
		drm_gem_object_unreference_unlocked(&xen_fb->xen_obj->base);

	drm_framebuffer_cleanup(fb);
	kfree(xen_fb);
}

int xen_drm_front_gem_dumb_map_offset(struct drm_file *file_priv,
		struct drm_device *dev, uint32_t handle, uint64_t *offset)
{
	struct drm_gem_object *gem_obj;
	struct xen_gem_object *xen_obj;
	int ret = 0;

	gem_obj = drm_gem_object_lookup(file_priv, handle);
	if (!gem_obj) {
		DRM_ERROR("Failed to lookup GEM object\n");
		return -ENOENT;
	}

	xen_obj = to_xen_gem_obj(gem_obj);
	/* do not allow mapping of the imported buffers */
	if (xen_obj->base.import_attach) {
		ret = -EINVAL;
	} else {
		ret = drm_gem_create_mmap_offset(gem_obj);
		if (ret < 0)
			*offset = 0;
		else
			*offset = drm_vma_node_offset_addr(&gem_obj->vma_node);
	}

	drm_gem_object_unreference_unlocked(gem_obj);
	return ret;
}
#endif /* CONFIG_DRM_XEN_FRONTEND_CMA */

int xen_drm_front_dumb_map_offset(struct drm_file *file_priv,
		struct drm_device *dev, uint32_t handle, uint64_t *offset)
{
	struct xen_drm_front_drm_info *drm_info;

	drm_info = dev->dev_private;
	return drm_info->gem_ops->dumb_map_offset(file_priv, dev,
		handle, offset);
}

int xen_drm_front_enable_vblank(struct drm_device *dev, unsigned int pipe)
{
	return 0;
}

void xen_drm_front_disable_vblank(struct drm_device *dev, unsigned int pipe)
{
}

void drm_atomic_helper_shutdown(struct drm_device *dev)
{
	struct drm_modeset_acquire_ctx ctx;
	int ret;

	drm_modeset_acquire_init(&ctx, 0);
	while (1) {
		ret = drm_modeset_lock_all_ctx(dev, &ctx);
		if (!ret)
			ret = drm_atomic_helper_disable_all(dev, &ctx);

		if (ret != -EDEADLK)
			break;

		drm_modeset_backoff(&ctx);
	}

	if (ret)
		DRM_ERROR("Disabling all crtc's during unload failed with %i\n", ret);

	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);
}

int drm_gem_fb_prepare_fb(struct drm_plane *plane,
			  struct drm_plane_state *state)
{
	if (plane->state->fb == state->fb || !state->fb)
		return 0;

	/*
	 * fence magic with dma-buf should happen here which w
	 * do not have in elder kernels
	 */
	return 0;
}
