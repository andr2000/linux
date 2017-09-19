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

#ifndef __XEN_FRONT_CFG_H
#define __XEN_FRONT_CFG_H

#include <sound/core.h>
#include <sound/pcm.h>

struct drv_info;

struct xen_front_cfg_stream {
	int index;
	char *xenstore_path;
	struct snd_pcm_hardware pcm_hw;
};

struct xen_front_cfg_pcm_instance {
	char name[80];
	int device_id;
	struct snd_pcm_hardware pcm_hw;
	int  num_streams_pb;
	struct xen_front_cfg_stream *streams_pb;
	int  num_streams_cap;
	struct xen_front_cfg_stream *streams_cap;
};

struct xen_front_cfg_card {
	char name_short[32];
	char name_long[80];
	struct snd_pcm_hardware pcm_hw;
	int num_pcm_instances;
	struct xen_front_cfg_pcm_instance *pcm_instances;
};

struct xen_front_cfg_card_plat_data {
	struct drv_info *drv_info;
	struct xen_front_cfg_card cfg_card;
};

int xen_front_cfg_card(struct drv_info *drv_info,
	struct xen_front_cfg_card_plat_data *plat_data, int *stream_cnt);

#endif /* __XEN_FRONT_CFG_H */
