/******************************************************************************
 * Xen grant device exported functionality
 */

#ifndef _XEN_GNTDEV_EXP_H
#define _XEN_GNTDEV_EXP_H

struct gntdev_priv *gntdev_alloc_context(struct device *dev);

void gntdev_free_context(struct gntdev_priv *priv);

int gntdev_dmabuf_exp_from_refs(struct gntdev_priv *priv, int flags,
				int count, u32 domid, u32 *refs, u32 *fd);

int gntdev_dmabuf_exp_wait_released(struct gntdev_priv *priv, u32 fd,
				    int wait_to_ms);

int gntdev_dmabuf_imp_release(struct gntdev_priv *priv, u32 fd);

u32 *gntdev_dmabuf_imp_to_refs(struct gntdev_priv *priv, int fd,
			      int count, int domid, u32 **refs);

#endif
