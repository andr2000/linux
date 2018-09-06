// SPDX-License-Identifier: GPL-2.0 OR MIT

/*
 * Xen para-virtual camera device
 *
 * Copyright (C) 2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#include <xen/xenbus.h>

#include <xen/interface/io/cameraif.h>

#include "xen_camera_front.h"

static int cfg_read_resolution(struct xenbus_device *xb_dev,
			       struct xen_camera_front_cfg_resolution *res,
			       const char *xenstore_base_path,
			       const char *name)
{
	struct device *dev = &xb_dev->dev;
	char *list, *tmp;
	char *cur_frame_rate;
	int num_frame_rates, i, ret, len;

	list = xenbus_read(XBT_NIL, xenstore_base_path, name, &len);
	if (IS_ERR(list)) {
		dev_err(dev, "No frame rates configured at %s/%s\n",
			xenstore_base_path, name);
		return PTR_ERR(list);
	}

	/* Make sure the list is not empty. */
	if (!list[0]) {
		ret = -EINVAL;
		goto fail;
	}

	/*
	 * At the first pass find out how many frame rates are there.
	 * At the second pass read frame rates, validate and store.
	 * Start from 1 as a single one frame rate configuration will
	 * have no separators at all.
	 */
	num_frame_rates = 1;
	for (i = 0; list[i]; i++)
		if (list[i] == XENCAMERA_LIST_SEPARATOR[0])
			num_frame_rates++;
	/* We have number of separators, number of frame rates is one more. */

	res->frame_rate = devm_kcalloc(dev, num_frame_rates,
				       sizeof(*res->frame_rate), GFP_KERNEL);
	if (!res->frame_rate) {
		ret = -ENOMEM;
		goto fail;
	}

	i = 0;
	/* Remember the original pointer, so we can kfree it after strsep. */
	tmp = list;
	while ((cur_frame_rate = strsep(&tmp, XENCAMERA_LIST_SEPARATOR))) {
		int cnt, num, denom;

		cnt = sscanf(cur_frame_rate,
			     "%d" XENCAMERA_FRAME_RATE_SEPARATOR "%d",
			     &num, &denom);
		if (cnt != 2) {
			dev_err(dev, "Wrong frame rate %s\n", cur_frame_rate);
			ret = -EINVAL;
			goto fail;
		}

		res->frame_rate[i].numerator = num;
		res->frame_rate[i++].denominator = denom;
	}

	res->num_frame_rates = num_frame_rates;

	ret = 0;

fail:
	kfree(list);
	return ret;
}

static int cfg_read_format(struct xenbus_device *xb_dev,
			   struct xen_camera_front_cfg_format *fmt,
			   const char *xenstore_base_path,
			   const char *name)
{
	struct device *dev = &xb_dev->dev;
	char **dir_nodes;
	char *xs_res_base_path;
	int num_resolutions, i, ret, cnt, width, height;

	if (strlen(name) != 4) {
		dev_info(dev, "%s isn't a FOURCC code\n", name);
		return -EINVAL;
	}

	fmt->pixel_format =  v4l2_fourcc(name[0], name[1], name[2], name[3]);

	/* Find out how many resolutions are configured. */
	dir_nodes = xenbus_directory(XBT_NIL, xenstore_base_path, name,
				     &num_resolutions);
	if (IS_ERR(dir_nodes)) {
		dev_err(dev, "No resolutions configured for format %s\n", name);
		return -EINVAL;
	}

	xs_res_base_path = kasprintf(GFP_KERNEL, "%s/%s",
				     xenstore_base_path, name);
	if (!xs_res_base_path) {
		ret = -ENOMEM;
		goto fail;
	}

	fmt->resolution = devm_kcalloc(dev, num_resolutions,
				       sizeof(*fmt->resolution), GFP_KERNEL);
	if (!fmt->resolution) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < num_resolutions; i++) {
		cnt = sscanf(dir_nodes[i],
			     "%d" XENCAMERA_RESOLUTION_SEPARATOR "%d",
			     &width, &height);
		if (cnt != 2) {
			dev_err(dev, "Wrong resolution %s\n", dir_nodes[i]);
			ret = -EINVAL;
			goto fail;
		}
		ret = cfg_read_resolution(xb_dev, &fmt->resolution[i],
			       xs_res_base_path, dir_nodes[i]);
		if (ret < 0)
			goto fail;

		fmt->resolution[i].width = width;
		fmt->resolution[i].height = height;
	}

	fmt->num_resolutions = num_resolutions;

	ret = 0;

fail:
	kfree(xs_res_base_path);
	kfree(dir_nodes);
	return ret;
}

static void cfg_dump(struct xen_camera_front_info *front_info)
{
	struct device *dev = &front_info->xb_dev->dev;
	struct xen_camera_front_cfg_card *cfg = &front_info->cfg;
	int fmt, res, rate;

	for (fmt = 0; fmt < cfg->num_formats; fmt++) {
		struct xen_camera_front_cfg_format *format = &cfg->format[fmt];
		char *pixel_format = (char *)&format->pixel_format;

		dev_info(dev, "Format[%d] %c%c%c%c\n", fmt,
			 pixel_format[0], pixel_format[1],
			 pixel_format[2], pixel_format[3]);

		for (res = 0; res < format->num_resolutions; res++) {
			struct xen_camera_front_cfg_resolution *resolution =
				&format->resolution[res];

			dev_info(dev, "\tResolution [%d] %dx%d\n", res,
				 resolution->width, resolution->height);

			for (rate = 0; rate < resolution->num_frame_rates;
			     rate++) {
				struct xen_camera_front_cfg_frame_rate *fr =
					&resolution->frame_rate[rate];

				dev_info(dev, "\t\tFrame rate [%d] %d/%d\n",
					 rate, fr->numerator, fr->denominator);
			}
		}
	}
}

int xen_camera_front_cfg_init(struct xen_camera_front_info *front_info)
{
	struct xenbus_device *xb_dev = front_info->xb_dev;
	struct device *dev = &xb_dev->dev;
	struct xen_camera_front_cfg_card *cfg = &front_info->cfg;
	char **dir_nodes;
	char *xs_fmt_base_path;
	int num_formats;
	int i, ret;

	cfg->num_formats = 0;
	/* Find out how many formats are configured. */
	dir_nodes = xenbus_directory(XBT_NIL, xb_dev->nodename,
				     XENCAMERA_FIELD_FORMATS,
				     &num_formats);
	if (IS_ERR(dir_nodes)) {
		dev_err(dev, "No formats configured\n");
		return -EINVAL;
	}

	xs_fmt_base_path = kasprintf(GFP_KERNEL, "%s/%s",
				     xb_dev->nodename, XENCAMERA_FIELD_FORMATS);
	if (!xs_fmt_base_path) {
		ret = -ENOMEM;
		goto fail;
	}

	cfg->format = devm_kcalloc(dev, num_formats, sizeof(*cfg->format),
				   GFP_KERNEL);
	if (!cfg->format) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < num_formats; i++) {
		ret = cfg_read_format(xb_dev, &cfg->format[i],
				      xs_fmt_base_path, dir_nodes[i]);
		if (ret < 0)
			goto fail;
	}

	cfg->num_formats = num_formats;

	cfg_dump(front_info);

	ret = 0;

fail:
	kfree(xs_fmt_base_path);
	kfree(dir_nodes);
	return ret;
}

