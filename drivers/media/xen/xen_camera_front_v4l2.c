// SPDX-License-Identifier: GPL-2.0 OR MIT

/*
 * Xen para-virtual camera device
 *
 * Based on V4L2 PCI Skeleton Driver: samples/v4l/v4l2-pci-skeleton.c
 *
 * Copyright (C) 2018 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#include <linux/videodev2.h>
#include <linux/v4l2-dv-timings.h>

#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>

#include <xen/xenbus.h>

#include <xen/interface/io/cameraif.h>

#include "xen_camera_front.h"
#include "xen_camera_front_v4l2.h"

struct cfg_control_ids {
	u32 xen_type;
	u32 v4l2_cid;
};

static const struct cfg_control_ids CFG_CONTROL_IDS[] = {
	{
		.xen_type = XENCAMERA_CTRL_BRIGHTNESS,
		.v4l2_cid = V4L2_CID_BRIGHTNESS,
	},
	{
		.xen_type = XENCAMERA_CTRL_CONTRAST,
		.v4l2_cid = V4L2_CID_CONTRAST,
	},
	{
		.xen_type = XENCAMERA_CTRL_SATURATION,
		.v4l2_cid = V4L2_CID_SATURATION,
	},
	{
		.xen_type = XENCAMERA_CTRL_HUE,
		.v4l2_cid = V4L2_CID_HUE
	},
};

int xen_camera_front_v4l2_to_v4l2_cid(int xen_type)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(CFG_CONTROL_IDS); i++)
		if (CFG_CONTROL_IDS[i].xen_type == xen_type)
			return CFG_CONTROL_IDS[i].v4l2_cid;
	return -EINVAL;
}

int xen_camera_front_v4l2_to_xen_type(int v4l2_cid)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(CFG_CONTROL_IDS); i++)
		if (CFG_CONTROL_IDS[i].v4l2_cid == v4l2_cid)
			return CFG_CONTROL_IDS[i].xen_type;
	return -EINVAL;
}

static int ioctl_querycap(struct file *file, void *fh,
			  struct v4l2_capability *cap)
{
	strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strlcpy(cap->card, "V4L2 para-virtualized camera", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:xen_bus", sizeof(cap->bus_info));
	return 0;
}

static int ioctl_try_fmt_vid_cap(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	return 0;
}

static int ioctl_s_fmt_vid_cap(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	return 0;
}

static int ioctl_g_fmt_vid_cap(struct file *file,
			       void *fh, struct v4l2_format *f)
{
	return 0;
}

static int ioctl_enum_fmt_vid_cap(struct file *file, void *fh,
				  struct v4l2_fmtdesc *f)
{
	struct xen_camera_front_v4l2_info *v4l2_info = video_drvdata(file);
	struct xen_camera_front_cfg_card *cfg = &v4l2_info->front_info->cfg;

	if (f->index >= cfg->num_formats)
		return -EINVAL;

	f->pixelformat = cfg->format[f->index].pixel_format;
	return 0;
}

static struct xen_camera_front_cfg_format *
get_format(struct xen_camera_front_cfg_card *cfg, u32 pixel_format)
{
	int i;

	for (i = 0; i < cfg->num_formats; i++) {
		struct xen_camera_front_cfg_format *format = &cfg->format[i];

		if (format->pixel_format == pixel_format)
			return format;
	}
	return NULL;
}

int ioctl_enum_framesizes(struct file *file, void *fh,
			  struct v4l2_frmsizeenum *fsize)
{
	struct xen_camera_front_v4l2_info *v4l2_info = video_drvdata(file);
	struct xen_camera_front_cfg_card *cfg = &v4l2_info->front_info->cfg;
	struct xen_camera_front_cfg_format *format;

	format = get_format(cfg, fsize->pixel_format);
	if (!format || (fsize->index >= format->num_resolutions))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = format->resolution[fsize->index].width;
	fsize->discrete.height = format->resolution[fsize->index].height;
	return 0;
}

int ioctl_enum_frameintervals(struct file *file, void *fh,
			      struct v4l2_frmivalenum *fival)
{
	struct xen_camera_front_v4l2_info *v4l2_info = video_drvdata(file);
	struct xen_camera_front_cfg_card *cfg = &v4l2_info->front_info->cfg;
	struct xen_camera_front_cfg_format *format;
	struct xen_camera_front_cfg_resolution *resolution = NULL;
	int i;

	format = get_format(cfg, fival->pixel_format);
	if (!format)
		return -EINVAL;

	for (i = 0; i < format->num_resolutions; i++) {
		struct xen_camera_front_cfg_resolution *r =
			&format->resolution[i];

		if ((r->width == fival->width) &&
		    (r->height == fival->height)) {
			resolution = r;
			break;
		}
	}

	if (!resolution || (fival->index >= resolution->num_frame_rates))
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator =
		resolution->frame_rate[fival->index].numerator;
	fival->discrete.denominator =
		resolution->frame_rate[fival->index].denominator;
	return 0;
}

static int ioctl_g_std(struct file *file, void *fh, v4l2_std_id *norm)
{
	return 0;
}

static int ioctl_s_std(struct file *file, void *fh, v4l2_std_id norm)
{
	return 0;
}

static int ioctl_querystd(struct file *file, void *fh, v4l2_std_id *a)
{
	return 0;
}

static int ioctl_s_dv_timings(struct file *file, void *fh,
			      struct v4l2_dv_timings *timings)
{
	return 0;
}

static int ioctl_g_dv_timings(struct file *file, void *fh,
			      struct v4l2_dv_timings *timings)
{
	return 0;
}

static int ioctl_query_dv_timings(struct file *file, void *fh,
				  struct v4l2_dv_timings *timings)
{
	return 0;
}

static int ioctl_enum_dv_timings(struct file *file, void *fh,
				 struct v4l2_enum_dv_timings *timings)
{
	return 0;
}

static int ioctl_dv_timings_cap(struct file *file, void *fh,
				struct v4l2_dv_timings_cap *cap)
{
	return 0;
}

static int ioctl_enum_input(struct file *file, void *fh,
			    struct v4l2_input *inp)
{
	return 0;
}

static int ioctl_g_input(struct file *file, void *fh, unsigned int *i)
{
	return 0;
}

static int ioctl_s_input(struct file *file, void *fh, unsigned int i)
{
	return 0;
}

static const struct v4l2_ioctl_ops ioctl_ops = {
	.vidioc_querycap = ioctl_querycap,
	.vidioc_try_fmt_vid_cap = ioctl_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = ioctl_s_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = ioctl_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = ioctl_enum_fmt_vid_cap,

	.vidioc_enum_framesizes = ioctl_enum_framesizes,
	.vidioc_enum_frameintervals = ioctl_enum_frameintervals,

	.vidioc_g_std = ioctl_g_std,
	.vidioc_s_std = ioctl_s_std,
	.vidioc_querystd = ioctl_querystd,

	.vidioc_s_dv_timings = ioctl_s_dv_timings,
	.vidioc_g_dv_timings = ioctl_g_dv_timings,
	.vidioc_enum_dv_timings = ioctl_enum_dv_timings,
	.vidioc_query_dv_timings = ioctl_query_dv_timings,
	.vidioc_dv_timings_cap = ioctl_dv_timings_cap,

	.vidioc_enum_input = ioctl_enum_input,
	.vidioc_g_input = ioctl_g_input,
	.vidioc_s_input = ioctl_s_input,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_log_status = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static int s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct xen_camera_front_v4l2_info *v4l2_info =
		container_of(ctrl->handler, struct xen_camera_front_v4l2_info,
			     ctrl_handler);
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		/* fall-through */
	case V4L2_CID_CONTRAST:
		/* fall-through */
	case V4L2_CID_SATURATION:
		/* fall-through */
	case V4L2_CID_HUE:
		return xen_camera_front_set_control(v4l2_info->front_info,
						    ctrl->id, ctrl->val);

	default:
		break;
	}

	return -EINVAL;
}

static const struct v4l2_ctrl_ops ctrl_ops = {
	.s_ctrl = s_ctrl,
};

static int init_controls(struct xen_camera_front_info *front_info)
{
	struct xen_camera_front_cfg_card *cfg = &front_info->cfg;
	struct xen_camera_front_v4l2_info *v4l2_info = front_info->v4l2_info;
	struct v4l2_ctrl_handler *hdl = &v4l2_info->ctrl_handler;
	int i, ret;

	v4l2_ctrl_handler_init(hdl, cfg->num_controls);

	for (i = 0; i < cfg->num_controls; i++)
		v4l2_ctrl_new_std(hdl, &ctrl_ops,
				  cfg->ctrl[i].v4l2_cid,
				  cfg->ctrl[i].minimum,
				  cfg->ctrl[i].maximum,
				  cfg->ctrl[i].step,
				  cfg->ctrl[i].default_value);
	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(&v4l2_info->ctrl_handler);
		return ret;
	}
	v4l2_info->v4l2_dev.ctrl_handler = hdl;
	return 0;
}

int xen_camera_front_v4l2_init(struct xen_camera_front_info *front_info)
{
	struct device *dev = &front_info->xb_dev->dev;
	struct xen_camera_front_v4l2_info *v4l2_info;
	struct video_device *vdev;
	struct vb2_queue *q;
	int ret;

	v4l2_info = devm_kzalloc(dev, sizeof(*v4l2_info), GFP_KERNEL);
	if (!v4l2_info)
		return -ENOMEM;

	v4l2_info->front_info = front_info;
	front_info->v4l2_info = v4l2_info;

	strlcpy(v4l2_info->v4l2_dev.name, XENCAMERA_DRIVER_NAME,
		sizeof(v4l2_info->v4l2_dev.name));

	ret = v4l2_device_register(dev, &v4l2_info->v4l2_dev);
	if (ret < 0)
		return ret;

	mutex_init(&v4l2_info->lock);

	/* Add the controls if any. */
	if (front_info->cfg.num_controls) {
		ret = init_controls(front_info);
		if (ret < 0)
			goto fail_unregister_v4l2;
	}

	/* Initialize the vb2 queue. */
	q = &v4l2_info->queue;

	vdev = &v4l2_info->vdev;
	strlcpy(vdev->name, KBUILD_MODNAME, sizeof(vdev->name));
	/*
	 * There is nothing to clean up, so release is set to an empty release
	 * function. The release callback must be non-NULL.
	 */
	vdev->release = video_device_release_empty;
	vdev->fops = &fops;
	vdev->ioctl_ops = &ioctl_ops;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE |
			    V4L2_CAP_STREAMING;
	/*
	 * The main serialization lock. All ioctls are serialized by this
	 * lock. Exception: if q->lock is set, then the streaming ioctls
	 * are serialized by that separate lock.
	 */
	vdev->lock = &v4l2_info->lock;
	vdev->queue = q;
	vdev->v4l2_dev = &v4l2_info->v4l2_dev;
	/* Supported SDTV standards, if any */
//	vdev->tvnorms = SKEL_TVNORMS;
	video_set_drvdata(vdev, v4l2_info);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0)
		goto fail;

	dev_info(dev, "V4L2 " XENCAMERA_DRIVER_NAME " driver loaded\n");

	return 0;

fail:
	video_unregister_device(&front_info->v4l2_info->vdev);
fail_unregister_v4l2:
	v4l2_device_unregister(&v4l2_info->v4l2_dev);
	return ret;
}

void xen_camera_front_v4l2_fini(struct xen_camera_front_info *front_info)
{
	struct xen_camera_front_v4l2_info *v4l2_info = front_info->v4l2_info;

	if (!v4l2_info)
		return;

	video_unregister_device(&v4l2_info->vdev);
	if (v4l2_info->v4l2_dev.ctrl_handler)
		v4l2_ctrl_handler_free(v4l2_info->v4l2_dev.ctrl_handler);
	v4l2_device_unregister(&v4l2_info->v4l2_dev);
}

