/* SPDX-License-Identifier: GPL-2.0 OR MIT */

/*
 * Grant device kernel API
 *
 * Copyright (C) 2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef _GRANT_DEV_H
#define _GRANT_DEV_H

#include <linux/types.h>

struct device;
struct gntdev_priv;
#ifdef CONFIG_XEN_GNTDEV_DMABUF
struct xen_dmabuf;
#endif

struct gntdev_priv *gntdev_alloc_context(struct device *dev);
void gntdev_free_context(struct gntdev_priv *priv);

#ifdef CONFIG_XEN_GNTDEV_DMABUF
int gntdev_dmabuf_exp_from_refs(struct gntdev_priv *priv, int flags,
				int count, u32 domid, u32 *refs, u32 *fd);
int gntdev_dmabuf_exp_wait_released(struct gntdev_priv *priv, int fd,
				    int wait_to_ms);

struct xen_dmabuf *gntdev_dmabuf_imp_to_refs(struct gntdev_priv *priv,
					     int fd, int count, int domid);
u32 *gntdev_dmabuf_imp_get_refs(struct xen_dmabuf *xen_dmabuf);
int gntdev_dmabuf_imp_release(struct gntdev_priv *priv, u32 fd);
#endif

#endif
