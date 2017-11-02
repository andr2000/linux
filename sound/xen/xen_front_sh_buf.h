/*
 * Xen para-virtual sound device
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 * Copyright (C) 2016-2017 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_FRONT_SH_BUF_H
#define __XEN_FRONT_SH_BUF_H

#include <xen/grant_table.h>

#include "xen_front_evtchnl.h"

struct xen_front_sh_buf_info {
	int num_grefs;
	grant_ref_t *grefs;
	uint8_t *vdirectory;
	uint8_t *vbuffer;
	size_t vbuffer_sz;
};

static inline grant_ref_t xen_front_sh_buf_get_dir_start(
	struct xen_front_sh_buf_info *buf)
{
	if (!buf->grefs)
		return GRANT_INVALID_REF;
	return buf->grefs[0];
}

static inline void xen_front_sh_buf_clear(struct xen_front_sh_buf_info *buf)
{
	memset(buf, 0, sizeof(*buf));
}

int xen_front_sh_buf_alloc(struct xenbus_device *xb_dev,
	struct xen_front_sh_buf_info *buf, unsigned int buffer_sz);
void xen_front_sh_buf_free(struct xen_front_sh_buf_info *buf);

#endif /* __XEN_FRONT_SH_BUF_H */
