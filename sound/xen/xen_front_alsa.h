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

#ifndef __XEN_FRONT_ALSA_H
#define __XEN_FRONT_ALSA_H

struct drv_info;
struct xen_front_evtchnl_info;

int xen_front_alsa_init(struct drv_info *drv_info);
void xen_front_alsa_cleanup(struct drv_info *drv_info);

void xen_front_alsa_handle_cur_pos(struct xen_front_evtchnl_info *channel,
	uint64_t pos_bytes);

#endif /* __XEN_FRONT_ALSA_H */
