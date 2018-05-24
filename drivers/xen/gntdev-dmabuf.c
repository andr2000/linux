// SPDX-License-Identifier: GPL-2.0

/*
 * Xen dma-buf functionality for gntdev.
 *
 * Copyright (c) 2018 Oleksandr Andrushchenko, EPAM Systems Inc.
 */

#include <linux/slab.h>

#include "gntdev-dmabuf.h"

struct gntdev_dmabuf_priv {
	int dummy;
};

/* ------------------------------------------------------------------ */
/* DMA buffer export support.                                         */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* Implementation of wait for exported DMA buffer to be released.     */
/* ------------------------------------------------------------------ */

int gntdev_dmabuf_exp_wait_released(struct gntdev_dmabuf_priv *priv, int fd,
				    int wait_to_ms)
{
	return -EINVAL;
}

/* ------------------------------------------------------------------ */
/* DMA buffer export support.                                         */
/* ------------------------------------------------------------------ */

int gntdev_dmabuf_exp_from_pages(struct gntdev_dmabuf_export_args *args)
{
	return -EINVAL;
}

/* ------------------------------------------------------------------ */
/* DMA buffer import support.                                         */
/* ------------------------------------------------------------------ */

struct gntdev_dmabuf *
gntdev_dmabuf_imp_to_refs(struct gntdev_dmabuf_priv *priv, struct device *dev,
			  int fd, int count, int domid)
{
	return ERR_PTR(-ENOMEM);
}

u32 *gntdev_dmabuf_imp_get_refs(struct gntdev_dmabuf *gntdev_dmabuf)
{
	return NULL;
}

int gntdev_dmabuf_imp_release(struct gntdev_dmabuf_priv *priv, u32 fd)
{
	return -EINVAL;
}

struct gntdev_dmabuf_priv *gntdev_dmabuf_init(void)
{
	struct gntdev_dmabuf_priv *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	return priv;
}

void gntdev_dmabuf_fini(struct gntdev_dmabuf_priv *priv)
{
	kfree(priv);
}
