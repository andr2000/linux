/*
 *  Xen para-virtual DRM device
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
 * Copyright (C) 2016-2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#ifndef __XEN_DRM_FRONT_KMS_H_
#define __XEN_DRM_FRONT_KMS_H_

#include "xen_drm_front_drv.h"

int xen_drm_front_kms_init(struct xen_drm_front_drm_info *drm_info);

void xen_drm_front_kms_on_page_flip_done(
		struct xen_drm_front_drm_pipeline *pipeline,
		uint64_t fb_cookie);

#endif /* __XEN_DRM_FRONT_KMS_H_ */
