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
//#include <linux/v4l2-dv-timings.h>

#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
//#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-sg.h>
#include <media/videobuf2-v4l2.h>

#include <xen/xenbus.h>

#include <xen/interface/io/cameraif.h>

#include "xen_camera_front.h"
#include "xen_camera_front_v4l2.h"
#include "xen_camera_front_shbuf.h"

struct xen_camera_front_v4l2_info {
	struct xen_camera_front_info *front_info;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_ctrl_handler ctrl_handler;
	/* ioctl serialization mutex. */
	struct mutex lock;

	struct vb2_queue queue;
	spinlock_t qlock;
	struct list_head buf_list;
	unsigned sequence;

	/* Size of a camera buffer. */
	size_t v4l2_buffer_sz;
};

struct xen_camera_buffer {
	struct vb2_v4l2_buffer vb;
	/* Xen shared buffer backing this V4L2 buffer's memory. */
	struct xen_camera_front_shbuf shbuf;
	/* Is this buffer queued or not. */
	bool is_queued;

	struct list_head list;
};

static struct xen_camera_buffer *
to_xen_camera_buffer(struct vb2_buffer *vb)
{
	return container_of(vb, struct xen_camera_buffer, vb.vb2_buf);
}

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
		.xen = XENCAMERA_COLORSPACE_DEFAULT,
		.v4l2 = V4L2_COLORSPACE_DEFAULT,
	},
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
		.v4l2 = V4L2_COLORSPACE_OPRGB,
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
		.xen = XENCAMERA_XFER_FUNC_DEFAULT,
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
		.v4l2 = V4L2_XFER_FUNC_OPRGB,
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
		.xen = XENCAMERA_YCBCR_ENC_IGNORE,
		.v4l2 = V4L2_YCBCR_ENC_DEFAULT,
	},
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
		.xen = XENCAMERA_QUANTIZATION_DEFAULT,
		.v4l2 = V4L2_QUANTIZATION_DEFAULT,
	},
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

static int xen_buf_layout_to_format(struct xen_camera_front_info *front_info,
				    struct v4l2_pix_format *sp)
{
	struct xencamera_buf_get_layout_resp buf_layout;
	int ret;

	ret =  xen_camera_front_get_buf_layout(front_info, &buf_layout);
	if (ret < 0)
		return ret;

	WARN_ON(buf_layout.num_planes != 1);

	sp->bytesperline = buf_layout.plane_stride[0];
	sp->sizeimage = buf_layout.plane_size[0];
	return 0;
}

static void buf_list_add(struct xen_camera_front_v4l2_info *v4l2_info,
			 struct xen_camera_buffer *xen_buf)
{
	unsigned long flags;

	spin_lock_irqsave(&v4l2_info->qlock, flags);
	list_add(&xen_buf->list, &v4l2_info->buf_list);
	spin_unlock_irqrestore(&v4l2_info->qlock, flags);
}

static void buf_list_remove(struct xen_camera_front_v4l2_info *v4l2_info,
			    struct xen_camera_buffer *xen_buf)
{
	unsigned long flags;

	spin_lock_irqsave(&v4l2_info->qlock, flags);
	list_del(&xen_buf->list);
	spin_unlock_irqrestore(&v4l2_info->qlock, flags);
}

static void
buf_list_destroy_all_shbuf(struct xen_camera_front_v4l2_info *v4l2_info)
{
	struct xen_camera_buffer *buf, *node;
	unsigned long flags;

	spin_lock_irqsave(&v4l2_info->qlock, flags);
	list_for_each_entry_safe(buf, node, &v4l2_info->buf_list, list) {
		list_del(&buf->list);
		xen_camera_front_destroy_shbuf(&buf->shbuf);
	}
	spin_unlock_irqrestore(&v4l2_info->qlock, flags);
}

static void buf_list_set_queued(struct xen_camera_front_v4l2_info *v4l2_info,
				struct xen_camera_buffer *xen_buf, bool state)
{
	unsigned long flags;

	spin_lock_irqsave(&v4l2_info->qlock, flags);
	xen_buf->is_queued = state;
	spin_unlock_irqrestore(&v4l2_info->qlock, flags);
}

static void buf_list_return_queued(struct xen_camera_front_v4l2_info *v4l2_info,
				   enum vb2_buffer_state state)
{
	struct xen_camera_buffer *buf, *node;
	unsigned long flags;

	spin_lock_irqsave(&v4l2_info->qlock, flags);
	list_for_each_entry_safe(buf, node, &v4l2_info->buf_list, list) {
		if (buf->is_queued) {
			vb2_buffer_done(&buf->vb.vb2_buf, state);
			buf->is_queued = false;
		}
	}
	spin_unlock_irqrestore(&v4l2_info->qlock, flags);
}

/*
 * Setup the constraints of the queue: besides setting the number of planes
 * per buffer and the size and allocation context of each plane, it also
 * checks if sufficient buffers have been allocated.
 */
static int queue_setup(struct vb2_queue *vq,
		       unsigned int *nbuffers, unsigned int *nplanes,
		       unsigned int sizes[], struct device *alloc_devs[])
{
	struct xen_camera_front_v4l2_info *v4l2_info = vb2_get_drv_priv(vq);
	struct v4l2_pix_format sp;
	int ret;

	printk("%s\n", __FUNCTION__);
	ret = xen_buf_layout_to_format(v4l2_info->front_info, &sp);
	if (ret < 0)
		return ret;

	if (vq->num_buffers + *nbuffers < 2)
		*nbuffers = 2;

	/* Check if backend can handle that many buffers. */
	ret = xen_camera_front_buf_request(v4l2_info->front_info, *nbuffers);
	if (ret < 0)
		return ret;
	*nbuffers = ret;

	if (*nplanes)
		return sizes[0] < sp.sizeimage ? -EINVAL : 0;

	*nplanes = 1;

	sizes[0] = sp.sizeimage;

	/* Remember the negotiated buffer size. */
	v4l2_info->v4l2_buffer_sz = sp.sizeimage;

	return 0;
}

/*
 * Called once after allocating a buffer (in MMAP case)
 * or after acquiring a new USERPTR buffer or queueing a new
 * dma-buf; drivers may perform additional buffer-related initialization;
 * initialization failure (return != 0) will prevent
 * queue setup from completing successfully; optional.
 */
static int buffer_init(struct vb2_buffer *vb)
{
	struct xen_camera_front_v4l2_info *v4l2_info =
		vb2_get_drv_priv(vb->vb2_queue);
	struct xen_camera_buffer *xen_buf = to_xen_camera_buffer(vb);
	struct sg_table *sgt;
	int ret;

	if (vb2_plane_size(vb, 0) < v4l2_info->v4l2_buffer_sz) {
		dev_err(&v4l2_info->front_info->xb_dev->dev,
			"Buffer too small (%lu < %lu)\n",
			vb2_plane_size(vb, 0), v4l2_info->v4l2_buffer_sz);
		return -EINVAL;
	}

	/* We only support a single plane. */
	sgt = vb2_dma_sg_plane_desc(vb, 0);
	if (!sgt)
		return -EFAULT;

	ret = xen_camera_front_buf_create(v4l2_info->front_info,
					  &xen_buf->shbuf, vb->index, sgt);
	if (ret < 0)
		return ret;

	buf_list_add(v4l2_info, xen_buf);
	return 0;
}

/*
 * Called once before the buffer is freed; drivers may
 * perform any additional cleanup; optional.
 */
static void buffer_cleanup(struct vb2_buffer *vb)
{
	struct xen_camera_front_v4l2_info *v4l2_info =
		vb2_get_drv_priv(vb->vb2_queue);
	struct xen_camera_buffer *xen_buf = to_xen_camera_buffer(vb);
	int ret;

	printk("%s\n", __FUNCTION__);
	ret = xen_camera_front_buf_destroy(v4l2_info->front_info,
					   &xen_buf->shbuf, vb->index);
	if (ret < 0)
		dev_err(&v4l2_info->front_info->xb_dev->dev,
			"Failed to cleanup buffer with index %d: %d\n",
			vb->index, ret);
	buf_list_remove(v4l2_info, xen_buf);
}

/*
 * Called every time the buffer is queued from userspace
 * and from the VIDIOC_PREPARE_BUF() ioctl; drivers may
 * perform any initialization required before each
 * hardware operation in this callback; drivers can
 * access/modify the buffer here as it is still synced for
 * the CPU; drivers that support VIDIOC_CREATE_BUFS() must
 * also validate the buffer size; if an error is returned,
 * the buffer will not be queued in driver; optional.
 */
static int buffer_prepare(struct vb2_buffer *vb)
{
	struct xen_camera_front_v4l2_info *v4l2_info =
		vb2_get_drv_priv(vb->vb2_queue);
	size_t size = v4l2_info->v4l2_buffer_sz;
	int ret;

	printk("%s\n", __FUNCTION__);
	if (vb2_plane_size(vb, 0) < size) {
		dev_err(&v4l2_info->front_info->xb_dev->dev,
			"Buffer too small (%lu < %lu)\n",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	/*
	 * FIXME: we might have an error here while communicating to
	 * the backend, but .buf_queue callback doesn't allow us to return
	 * any error code: queue the buffer to the backend now.
	 */
	ret = xen_camera_front_buf_queue(v4l2_info->front_info, vb->index);
	if (ret < 0)
		dev_err(&v4l2_info->front_info->xb_dev->dev,
			"Failed to queue buffer with index %d: %d\n",
			vb->index, ret);
	return ret;
}

/*
 * Called before every dequeue of the buffer back to
 * userspace; the buffer is synced for the CPU, so drivers
 * can access/modify the buffer contents; drivers may
 * perform any operations required before userspace
 * accesses the buffer; optional. The buffer state can be
 * one of the following: %DONE and %ERROR occur while
 * streaming is in progress, and the %PREPARED state occurs
 * when the queue has been canceled and all pending
 * buffers are being returned to their default %DEQUEUED
 * state. Typically you only have to do something if the
 * state is %VB2_BUF_STATE_DONE, since in all other cases
 * the buffer contents will be ignored anyway.
 */
static void buffer_finish(struct vb2_buffer *vb)
{
	struct xen_camera_front_v4l2_info *v4l2_info =
		vb2_get_drv_priv(vb->vb2_queue);
	int ret;

	ret = xen_camera_front_buf_dequeue(v4l2_info->front_info, vb->index);
	if (ret < 0)
		dev_err(&v4l2_info->front_info->xb_dev->dev,
			"Failed to dequeue buffer with index %d: %d\n",
			vb->index, ret);
}

/*
 * Passes buffer vb to the driver; driver may start
 * hardware operation on this buffer; driver should give
 * the buffer back by calling vb2_buffer_done() function;
 * it is allways called after calling VIDIOC_STREAMON()
 * ioctl; might be called before @start_streaming callback
 * if user pre-queued buffers before calling
 * VIDIOC_STREAMON().
 */
static void buffer_queue(struct vb2_buffer *vb)
{
	struct xen_camera_front_v4l2_info *v4l2_info =
		vb2_get_drv_priv(vb->vb2_queue);
	struct xen_camera_buffer *xen_buf = to_xen_camera_buffer(vb);

	printk("%s\n", __FUNCTION__);
	buf_list_set_queued(v4l2_info, xen_buf, true);
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
	int ret = 0;

	printk("%s\n", __FUNCTION__);

	v4l2_info->sequence = 0;

	if (ret)
		buf_list_return_queued(v4l2_info, VB2_BUF_STATE_QUEUED);
	return ret;
}

/*
 * Stop the DMA engine. Any remaining buffers in the DMA queue are dequeued
 * and passed on to the vb2 framework marked as STATE_ERROR.
 */
static void stop_streaming(struct vb2_queue *vq)
{
	struct xen_camera_front_v4l2_info *v4l2_info = vb2_get_drv_priv(vq);

	printk("%s\n", __FUNCTION__);
	buf_list_return_queued(v4l2_info, VB2_BUF_STATE_ERROR);
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
	.buf_finish		= buffer_finish,
	.buf_init		= buffer_init,
	.buf_cleanup		= buffer_cleanup,

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
	struct v4l2_pix_format *sp = &f->fmt.pix;
	int ret;

	sp->width = cfg->width;
	sp->height = cfg->height;
	sp->pixelformat = cfg->pixel_format;

	/* Always progressive image. */
	sp->field = V4L2_FIELD_NONE;

	ret = xen_to_v4l2(cfg->colorspace, XEN_COLORSPACE_TO_V4L2,
			  ARRAY_SIZE(XEN_COLORSPACE_TO_V4L2));
	if (ret < 0)
		return ret;
	sp->colorspace = ret;

	ret = xen_to_v4l2(cfg->xfer_func, XEN_XFER_FUNC_TO_V4L2,
			  ARRAY_SIZE(XEN_XFER_FUNC_TO_V4L2));
	if (ret < 0)
		return ret;
	sp->xfer_func = ret;

	ret = xen_to_v4l2(cfg->ycbcr_enc, XEN_YCBCR_ENC_TO_V4L2,
			  ARRAY_SIZE(XEN_YCBCR_ENC_TO_V4L2));
	if (ret < 0)
		return ret;
	sp->ycbcr_enc = ret;

	ret = xen_to_v4l2(cfg->quantization, XEN_QUANTIZATION_TO_V4L2,
			  ARRAY_SIZE(XEN_QUANTIZATION_TO_V4L2));
	if (ret < 0)
		return ret;
	sp->quantization = ret;

	return 0;
}

static int v4l2_fmt_to_xen_cfg(struct xen_camera_front_v4l2_info *v4l2_info,
			       struct v4l2_format *f,
			       struct xencamera_config *cfg)
{
	struct v4l2_pix_format *sp = &f->fmt.pix;
	int ret;

	cfg->width = sp->width;
	cfg->height = sp->height;
	cfg->pixel_format = sp->pixelformat;

	ret = v4l2_to_xen(sp->colorspace, XEN_COLORSPACE_TO_V4L2,
			  ARRAY_SIZE(XEN_COLORSPACE_TO_V4L2));
	if (ret < 0)
		return ret;
	cfg->colorspace = ret;

	ret = v4l2_to_xen(sp->xfer_func, XEN_XFER_FUNC_TO_V4L2,
			  ARRAY_SIZE(XEN_XFER_FUNC_TO_V4L2));
	if (ret < 0)
		return ret;
	cfg->xfer_func = ret;

	ret = v4l2_to_xen(sp->ycbcr_enc, XEN_YCBCR_ENC_TO_V4L2,
			  ARRAY_SIZE(XEN_YCBCR_ENC_TO_V4L2));
	if (ret < 0)
		return ret;
	cfg->ycbcr_enc = ret;

	ret = v4l2_to_xen(sp->quantization, XEN_QUANTIZATION_TO_V4L2,
			  ARRAY_SIZE(XEN_QUANTIZATION_TO_V4L2));
	if (ret < 0)
		return ret;
	cfg->quantization = ret;

	return 0;
}

static int ioctl_s_fmt_vid_cap(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	struct xen_camera_front_v4l2_info *v4l2_info = video_drvdata(file);
	struct xencamera_config cfg;
	struct v4l2_pix_format *sp = &f->fmt.pix;
	int ret;

	/*
	 * It is not allowed to change the format while buffers for use with
	 * streaming have already been allocated.
	 */
	if (vb2_is_busy(&v4l2_info->queue))
		return -EBUSY;

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

	return xen_buf_layout_to_format(v4l2_info->front_info, sp);
}

static int ioctl_g_fmt_vid_cap(struct file *file,
			       void *fh, struct v4l2_format *f)
{
	struct xen_camera_front_v4l2_info *v4l2_info = video_drvdata(file);
	struct xencamera_config cfg;
	struct v4l2_pix_format *sp = &f->fmt.pix;
	int ret;

	ret = xen_camera_front_get_config(v4l2_info->front_info, &cfg);
	if (ret < 0)
		return ret;

	ret =  xen_cfg_to_v4l2_fmt(v4l2_info, &cfg, f);
	if (ret < 0)
		return ret;

	return xen_buf_layout_to_format(v4l2_info->front_info, sp);
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
	/*
	 * vidioc_try_fmt_vid_cap is not supported intentionally due to
	 * possible race conditions when frontends in different VMs
	 * may try configuration and then this configuration becomes
	 * unavailble, because some other frontend allocates buffers
	 * and starts streaming with different settings.
	 */
	.vidioc_s_fmt_vid_cap = ioctl_s_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = ioctl_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = ioctl_enum_fmt_vid_cap,

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

	INIT_LIST_HEAD(&v4l2_info->buf_list);
	spin_lock_init(&v4l2_info->qlock);

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

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_USERPTR;
	q->dev = dev;
	q->drv_priv = v4l2_info;
	q->buf_struct_size = sizeof(struct xen_camera_buffer);
	q->ops = &qops;
	/*
	 * It is easier for us to work with vb2_dma_sg_memops
	 * rather than vb2_dma_contig_memops as this might relax
	 * requirements for memory subsystem.
	 */
	q->mem_ops = &vb2_dma_sg_memops;
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
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
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

	/* TODO: queue might not be initialized because of init errors. */
	if (vb2_is_busy(&v4l2_info->queue))
		dev_err(&v4l2_info->front_info->xb_dev->dev,
			"Unplugging while streaming!!!!\n");

	buf_list_destroy_all_shbuf(v4l2_info);

	video_unregister_device(&v4l2_info->vdev);
	if (v4l2_info->v4l2_dev.ctrl_handler)
		v4l2_ctrl_handler_free(v4l2_info->v4l2_dev.ctrl_handler);
	v4l2_device_unregister(&v4l2_info->v4l2_dev);
}

