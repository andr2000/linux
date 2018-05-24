/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Xen dma-buf functionality for gntdev.
 *
 * Copyright (c) 2018 Oleksandr Andrushchenko, EPAM Systems Inc.
 */

#ifndef _GNTDEV_DMABUF_H
#define _GNTDEV_DMABUF_H

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>

struct gntdev_dmabuf_priv;
struct gntdev_dmabuf;
struct device;

struct gntdev_dmabuf_export_args {
	int dummy;
};

struct gntdev_dmabuf_priv *gntdev_dmabuf_init(void);

void gntdev_dmabuf_fini(struct gntdev_dmabuf_priv *priv);

int gntdev_dmabuf_exp_from_pages(struct gntdev_dmabuf_export_args *args);

int gntdev_dmabuf_exp_wait_released(struct gntdev_dmabuf_priv *priv, int fd,
				    int wait_to_ms);

struct gntdev_dmabuf *
gntdev_dmabuf_imp_to_refs(struct gntdev_dmabuf_priv *priv, struct device *dev,
			  int fd, int count, int domid);

u32 *gntdev_dmabuf_imp_get_refs(struct gntdev_dmabuf *gntdev_dmabuf);

int gntdev_dmabuf_imp_release(struct gntdev_dmabuf_priv *priv, u32 fd);

#endif
