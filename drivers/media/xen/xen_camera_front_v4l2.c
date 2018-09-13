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

#include <media/videobuf2-dma-contig.h>

#include <xen/xenbus.h>

#include <xen/interface/io/cameraif.h>

#include "xen_camera_front.h"
#include "xen_camera_front_v4l2.h"

struct xen_camera_buffer {
	struct vb2_buffer vb;
	struct list_head list;
};

struct xen_to_v4l2 {
	int xen;
	int v4l2;
};

static const struct xen_to_v4l2 XEN_TYPE_TO_V4L2_CID[] = {
	{
		.xen = XENCAMERA_CTRL_BRIGHTNESS,
		.v4l2 = V4L2_CID_BRIGHTNESS,
	},
	{
		.xen = XENCAMERA_CTRL_CONTRAST,
		.v4l2 = V4L2_CID_CONTRAST,
	},
	{
		.xen = XENCAMERA_CTRL_SATURATION,
		.v4l2 = V4L2_CID_SATURATION,
	},
	{
		.xen = XENCAMERA_CTRL_HUE,
		.v4l2 = V4L2_CID_HUE
	},
};

static const struct xen_to_v4l2 XEN_COLORSPACE_TO_V4L2[] = {
	{
		.xen = XENCAMERA_COLORSPACE_SMPTE170M,
		.v4l2 = V4L2_COLORSPACE_SMPTE170M,
	},
	{
		.xen = XENCAMERA_COLORSPACE_REC709,
		.v4l2 = V4L2_COLORSPACE_REC709,
	},
	{
		.xen = XENCAMERA_COLORSPACE_SRGB,
		.v4l2 = V4L2_COLORSPACE_SRGB,
	},
	{
		.xen = XENCAMERA_COLORSPACE_OPRGB,
		.v4l2 = V4L2_COLORSPACE_ADOBERGB,
	},
	{
		.xen = XENCAMERA_COLORSPACE_BT2020,
		.v4l2 = V4L2_COLORSPACE_BT2020,
	},
	{
		.xen = XENCAMERA_COLORSPACE_DCI_P3,
		.v4l2 = V4L2_COLORSPACE_DCI_P3,
	},
};

static const struct xen_to_v4l2 XEN_XFER_FUNC_TO_V4L2[] = {
	{
		.xen = XENCAMERA_YCBCR_ENC_IGNORE,
		.v4l2 = V4L2_XFER_FUNC_DEFAULT,
	},
	{
		.xen = XENCAMERA_XFER_FUNC_709,
		.v4l2 = V4L2_XFER_FUNC_709,
	},
	{
		.xen = XENCAMERA_XFER_FUNC_SRGB,
		.v4l2 = V4L2_XFER_FUNC_SRGB,
	},
	{
		.xen = XENCAMERA_XFER_FUNC_OPRGB,
		.v4l2 = V4L2_XFER_FUNC_ADOBERGB,
	},
	{
		.xen = XENCAMERA_XFER_FUNC_NONE,
		.v4l2 = V4L2_XFER_FUNC_NONE,
	},
	{
		.xen = XENCAMERA_XFER_FUNC_DCI_P3,
		.v4l2 = V4L2_XFER_FUNC_DCI_P3,
	},
	{
		.xen = XENCAMERA_XFER_FUNC_SMPTE2084,
		.v4l2 = V4L2_XFER_FUNC_SMPTE2084,
	},
};

static const struct xen_to_v4l2 XEN_YCBCR_ENC_TO_V4L2[] = {
	{
		.xen = XENCAMERA_YCBCR_ENC_601,
		.v4l2 = V4L2_YCBCR_ENC_601,
	},
	{
		.xen = XENCAMERA_YCBCR_ENC_709,
		.v4l2 = V4L2_YCBCR_ENC_709,
	},
	{
		.xen = XENCAMERA_YCBCR_ENC_XV601,
		.v4l2 = V4L2_YCBCR_ENC_XV601,
	},
	{
		.xen = XENCAMERA_YCBCR_ENC_XV709,
		.v4l2 = V4L2_YCBCR_ENC_XV709,
	},
	{
		.xen = XENCAMERA_YCBCR_ENC_BT2020,
		.v4l2 = V4L2_YCBCR_ENC_BT2020,
	},
	{
		.xen = XENCAMERA_YCBCR_ENC_BT2020_CONST_LUM,
		.v4l2 = V4L2_YCBCR_ENC_BT2020_CONST_LUM,
	},
};

static const struct xen_to_v4l2 XEN_QUANTIZATION_TO_V4L2[] = {
	{
		.xen = XENCAMERA_QUANTIZATION_FULL_RANGE,
		.v4l2 = V4L2_QUANTIZATION_FULL_RANGE,
	},
	{
		.xen = XENCAMERA_QUANTIZATION_LIM_RANGE,
		.v4l2 = V4L2_QUANTIZATION_LIM_RANGE,
	},
};

static int xen_to_v4l2(int xen, const struct xen_to_v4l2 *table,
		       size_t table_sz)
{
	int i;

	for (i = 0; i < table_sz; i++)
		if (table[i].xen == xen)
			return table[i].v4l2;
	return -EINVAL;
}

static int v4l2_to_xen(int v4l2, const struct xen_to_v4l2 *table,
		       size_t table_sz)
{
	int i;

	for (i = 0; i < table_sz; i++)
		if (table[i].v4l2 == v4l2)
			return table[i].xen;
	return -EINVAL;
}

int xen_camera_front_v4l2_to_v4l2_cid(int xen_type)
{
	return xen_to_v4l2(xen_type, XEN_TYPE_TO_V4L2_CID,
			   ARRAY_SIZE(XEN_TYPE_TO_V4L2_CID));
}

int xen_camera_front_v4l2_to_xen_type(int v4l2_cid)
{
	return v4l2_to_xen(v4l2_cid, XEN_TYPE_TO_V4L2_CID,
			   ARRAY_SIZE(XEN_TYPE_TO_V4L2_CID));
}

/*
 * Setup the constraints of the queue: besides setting the number of planes
 * per buffer and the size and allocation context of each plane, it also
 * checks if sufficient buffers have been allocated. Usually 3 is a good
 * minimum number: many DMA engines need a minimum of 2 buffers in the
 * queue and you need to have another available for userspace processing.
 */
static int queue_setup(struct vb2_queue *vq,
		       unsigned int *nbuffers, unsigned int *nplanes,
		       unsigned int sizes[], struct device *alloc_devs[])
{
	struct xen_camera_front_v4l2_info *v4l2_info = vb2_get_drv_priv(vq);

	return 0;
}

/*
 * Prepare the buffer for queueing to the DMA engine: check and set the
 * payload size.
 */
static int buffer_prepare(struct vb2_buffer *vb)
{
	struct xen_camera_front_v4l2_info *v4l2_info =
		vb2_get_drv_priv(vb->vb2_queue);
	return 0;
}

/*
 * Queue this buffer to the DMA engine.
 */
static void buffer_queue(struct vb2_buffer *vb)
{
	struct xen_camera_front_v4l2_info *v4l2_info =
		vb2_get_drv_priv(vb->vb2_queue);
}

/*
 * Start streaming. First check if the minimum number of buffers have been
 * queued. If not, then return -ENOBUFS and the vb2 framework will call
 * this function again the next time a buffer has been queued until enough
 * buffers are available to actually start the DMA engine.
 */
static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct xen_camera_front_v4l2_info *v4l2_info = vb2_get_drv_priv(vq);

	return 0;
}

/*
 * Stop the DMA engine. Any remaining buffers in the DMA queue are dequeued
 * and passed on to the vb2 framework marked as STATE_ERROR.
 */
static void stop_streaming(struct vb2_queue *vq)
{
	struct xen_camera_front_v4l2_info *v4l2_info = vb2_get_drv_priv(vq);
}

/*
 * The vb2 queue ops. Note that since q->lock is set we can use the standard
 * vb2_ops_wait_prepare/finish helper functions. If q->lock would be NULL,
 * then this driver would have to provide these ops.
 */
static const struct vb2_ops qops = {
	.queue_setup		= queue_setup,
	.buf_prepare		= buffer_prepare,
	.buf_queue		= buffer_queue,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int ioctl_querycap(struct file *file, void *fh,
			  struct v4l2_capability *cap)
{
	strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strlcpy(cap->card, "V4L2 para-virtualized camera", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:xen_bus", sizeof(cap->bus_info));
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

static struct xen_camera_front_cfg_resolution *
get_resolution(struct xen_camera_front_cfg_format *format,
	       int width, int height)
{
	int i;

	for (i = 0; i < format->num_resolutions; i++) {
		struct xen_camera_front_cfg_resolution *r =
			&format->resolution[i];

		if ((r->width == width) && (r->height == height))
			return r;
	}
	return NULL;
}

static int xen_cfg_to_v4l2_fmt(struct xen_camera_front_v4l2_info *v4l2_info,
			       struct xencamera_config *cfg,
			       struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	int ret;

	mp->width = cfg->width;
	mp->height = cfg->height;
	mp->pixelformat = cfg->pixel_format;

	/* Always progressive image. */
	mp->field = V4L2_FIELD_NONE;

	ret = xen_to_v4l2(cfg->colorspace, XEN_COLORSPACE_TO_V4L2,
			  ARRAY_SIZE(XEN_COLORSPACE_TO_V4L2));
	if (ret < 0)
		return ret;
	mp->colorspace = ret;

	ret = xen_to_v4l2(cfg->xfer_func, XEN_XFER_FUNC_TO_V4L2,
			  ARRAY_SIZE(XEN_XFER_FUNC_TO_V4L2));
	if (ret < 0)
		return ret;
	mp->xfer_func = ret;

	ret = xen_to_v4l2(cfg->ycbcr_enc, XEN_YCBCR_ENC_TO_V4L2,
			  ARRAY_SIZE(XEN_YCBCR_ENC_TO_V4L2));
	if (ret < 0)
		return ret;
	mp->ycbcr_enc = ret;

	ret = xen_to_v4l2(cfg->quantization, XEN_QUANTIZATION_TO_V4L2,
			  ARRAY_SIZE(XEN_QUANTIZATION_TO_V4L2));
	if (ret < 0)
		return ret;
	mp->quantization = ret;

	return 0;
}

static int v4l2_fmt_to_xen_cfg(struct xen_camera_front_v4l2_info *v4l2_info,
			       struct v4l2_format *f,
			       struct xencamera_config *cfg)
{
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	int ret;

	cfg->width = mp->width;
	cfg->height = mp->height;
	cfg->pixel_format = mp->pixelformat;

	ret = v4l2_to_xen(mp->colorspace, XEN_COLORSPACE_TO_V4L2,
			  ARRAY_SIZE(XEN_COLORSPACE_TO_V4L2));
	if (ret < 0)
		return ret;
	cfg->colorspace = ret;

	ret = v4l2_to_xen(mp->xfer_func, XEN_XFER_FUNC_TO_V4L2,
			  ARRAY_SIZE(XEN_XFER_FUNC_TO_V4L2));
	if (ret < 0)
		return ret;
	cfg->xfer_func = ret;

	ret = v4l2_to_xen(mp->ycbcr_enc, XEN_YCBCR_ENC_TO_V4L2,
			  ARRAY_SIZE(XEN_YCBCR_ENC_TO_V4L2));
	if (ret < 0)
		return ret;
	cfg->ycbcr_enc = ret;

	ret = v4l2_to_xen(mp->quantization, XEN_QUANTIZATION_TO_V4L2,
			  ARRAY_SIZE(XEN_QUANTIZATION_TO_V4L2));
	if (ret < 0)
		return ret;
	cfg->quantization = ret;

	return 0;
}

static int xen_buf_to_mplane(struct xen_camera_front_info *front_info,
			     struct v4l2_pix_format_mplane *mp)
{
	struct xencamera_buf_get_layout_resp buf_layout;
	int ret, p;

	ret =  xen_camera_front_get_buf_layout(front_info, &buf_layout);
	if (ret < 0)
		return ret;

	mp->num_planes = buf_layout.num_planes;

	for (p = 0; p < mp->num_planes; p++) {
		mp->plane_fmt[p].bytesperline = buf_layout.plane_stride[p];
		mp->plane_fmt[p].sizeimage = buf_layout.plane_size[p];
	}
	return 0;
}

static int ioctl_try_fmt_vid_cap_mplane(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct xen_camera_front_v4l2_info *v4l2_info = video_drvdata(file);
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	struct xen_camera_front_cfg_format *format;
	struct xen_camera_front_cfg_resolution *resolution;

	/* First check local config if we support what was requested. */
	format = get_format(&v4l2_info->front_info->cfg, mp->pixelformat);
	if (!format)
		return -EINVAL;

	resolution = get_resolution(format, mp->width, mp->height);
	if (!resolution)
		return -EINVAL;

	return 0;
}

static int ioctl_s_fmt_vid_cap_mplane(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	struct xen_camera_front_v4l2_info *v4l2_info = video_drvdata(file);
	struct xencamera_config cfg;
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	int ret;

	ret = v4l2_fmt_to_xen_cfg(v4l2_info, f, &cfg);
	if (ret < 0)
		return ret;

	/* Ask the backend to validate and set the configuration. */
	ret = xen_camera_front_set_config(v4l2_info->front_info, &cfg, &cfg);
	if (ret < 0)
		return ret;

	ret =  xen_cfg_to_v4l2_fmt(v4l2_info, &cfg, f);
	if (ret < 0)
		return ret;

	return xen_buf_to_mplane(v4l2_info->front_info, mp);
}

static int ioctl_g_fmt_vid_cap_mplane(struct file *file,
				      void *fh, struct v4l2_format *f)
{
	struct xen_camera_front_v4l2_info *v4l2_info = video_drvdata(file);
	struct xencamera_config cfg;
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	int ret;

	ret = xen_camera_front_get_config(v4l2_info->front_info, &cfg);
	if (ret < 0)
		return ret;

	ret =  xen_cfg_to_v4l2_fmt(v4l2_info, &cfg, f);
	if (ret < 0)
		return ret;

	return xen_buf_to_mplane(v4l2_info->front_info, mp);
}

static int ioctl_enum_fmt_vid_cap_mplane(struct file *file, void *fh,
					 struct v4l2_fmtdesc *f)
{
	struct xen_camera_front_v4l2_info *v4l2_info = video_drvdata(file);
	struct xen_camera_front_cfg_card *cfg = &v4l2_info->front_info->cfg;

	if (f->index >= cfg->num_formats)
		return -EINVAL;

	f->pixelformat = cfg->format[f->index].pixel_format;
	return 0;
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
	struct xen_camera_front_cfg_resolution *resolution;

	format = get_format(cfg, fival->pixel_format);
	if (!format)
		return -EINVAL;

	resolution = get_resolution(format, fival->width, fival->height);
	if (!resolution || (fival->index >= resolution->num_frame_rates))
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator =
		resolution->frame_rate[fival->index].numerator;
	fival->discrete.denominator =
		resolution->frame_rate[fival->index].denominator;
	return 0;
}

static int ioctl_enum_input(struct file *file, void *fh,
			    struct v4l2_input *inp)
{
	if (inp->index > 0)
		return -EINVAL;

	strlcpy(inp->name, "Xen PV camera", sizeof(inp->name));
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	return 0;
}

static int ioctl_g_input(struct file *file, void *fh, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int ioctl_s_input(struct file *file, void *fh, unsigned int i)
{
	return (i > 0) ? -EINVAL : 0;
}

static const struct v4l2_ioctl_ops ioctl_ops = {
	.vidioc_querycap = ioctl_querycap,
	.vidioc_try_fmt_vid_cap_mplane = ioctl_try_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane = ioctl_s_fmt_vid_cap_mplane,
	.vidioc_g_fmt_vid_cap_mplane = ioctl_g_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_cap_mplane = ioctl_enum_fmt_vid_cap_mplane,

	.vidioc_enum_framesizes = ioctl_enum_framesizes,
	.vidioc_enum_frameintervals = ioctl_enum_frameintervals,

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
	INIT_LIST_HEAD(&v4l2_info->buf_list);

	q = &v4l2_info->queue;

	/*
	 * We only support multiplane operation even for
	 * single plane buffers.
	 */
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->dev = dev;
	q->drv_priv = v4l2_info;
	q->buf_struct_size = sizeof(struct xen_camera_buffer);
	q->ops = &qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 2;
	/*
	 * The serialization lock for the streaming ioctls. This is the same
	 * as the main serialization lock, but if some of the non-streaming
	 * ioctls could take a long time to execute, then you might want to
	 * have a different lock here to prevent VIDIOC_DQBUF from being
	 * blocked while waiting for another action to finish. This is
	 * generally not needed for PCI devices, but USB devices usually do
	 * want a separate lock here.
	 */
	q->lock = &v4l2_info->lock;
	ret = vb2_queue_init(q);
	if (ret)
		goto fail_unregister_ctrl;

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
	video_set_drvdata(vdev, v4l2_info);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0)
		goto fail;

	dev_info(dev, "V4L2 " XENCAMERA_DRIVER_NAME " driver loaded\n");

	return 0;

fail:
	video_unregister_device(&front_info->v4l2_info->vdev);
fail_unregister_ctrl:
	v4l2_ctrl_handler_free(v4l2_info->v4l2_dev.ctrl_handler);
	v4l2_info->v4l2_dev.ctrl_handler = NULL;
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

