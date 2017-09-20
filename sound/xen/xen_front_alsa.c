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

#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>

#include <xen/xen.h>
#include <xen/xenbus.h>

#include <xen/interface/io/sndif.h>

#include "xen_front.h"
#include "xen_front_alsa.h"
#include "xen_front_cfg.h"
#include "xen_front_evtchnl.h"
#include "xen_front_sh_buf.h"

struct pcm_stream_info {
	bool is_open;
	int index;
	struct snd_pcm_hardware pcm_hw;
	struct xen_front_evtchnl_pair_info *evt_pair;
	struct xen_front_sh_buf_info sh_buf;

	/* number of processed frames as reported by the backend */
	snd_pcm_uframes_t be_cur_frame;
	/* current HW pointer to be reported via .period callback */
	atomic_t hw_ptr;
	/* modulo of the number of processed frames - for period detection */
	uint32_t out_frames;
};

struct pcm_instance_info {
	struct card_info *card_info;
	struct snd_pcm *pcm;
	struct snd_pcm_hardware pcm_hw;
	int num_pcm_streams_pb;
	struct pcm_stream_info *streams_pb;
	int num_pcm_streams_cap;
	struct pcm_stream_info *streams_cap;
};

struct card_info {
	struct drv_info *drv_info;
	struct snd_card *card;
	struct snd_pcm_hardware pcm_hw;
	int num_pcm_instances;
	struct pcm_instance_info *pcm_instances;
};

struct alsa_sndif_sample_format {
	uint8_t sndif;
	snd_pcm_format_t alsa;
};

static const struct alsa_sndif_sample_format ALSA_SNDIF_FORMATS[] = {
	{
		.sndif = XENSND_PCM_FORMAT_U8,
		.alsa = SNDRV_PCM_FORMAT_U8
	},
	{
		.sndif = XENSND_PCM_FORMAT_S8,
		.alsa = SNDRV_PCM_FORMAT_S8
	},
	{
		.sndif = XENSND_PCM_FORMAT_U16_LE,
		.alsa = SNDRV_PCM_FORMAT_U16_LE
	},
	{
		.sndif = XENSND_PCM_FORMAT_U16_BE,
		.alsa = SNDRV_PCM_FORMAT_U16_BE
	},
	{
		.sndif = XENSND_PCM_FORMAT_S16_LE,
		.alsa = SNDRV_PCM_FORMAT_S16_LE
	},
	{
		.sndif = XENSND_PCM_FORMAT_S16_BE,
		.alsa = SNDRV_PCM_FORMAT_S16_BE
	},
	{
		.sndif = XENSND_PCM_FORMAT_U24_LE,
		.alsa = SNDRV_PCM_FORMAT_U24_LE
	},
	{
		.sndif = XENSND_PCM_FORMAT_U24_BE,
		.alsa = SNDRV_PCM_FORMAT_U24_BE
	},
	{
		.sndif = XENSND_PCM_FORMAT_S24_LE,
		.alsa = SNDRV_PCM_FORMAT_S24_LE
	},
	{
		.sndif = XENSND_PCM_FORMAT_S24_BE,
		.alsa = SNDRV_PCM_FORMAT_S24_BE
	},
	{
		.sndif = XENSND_PCM_FORMAT_U32_LE,
		.alsa = SNDRV_PCM_FORMAT_U32_LE
	},
	{
		.sndif = XENSND_PCM_FORMAT_U32_BE,
		.alsa = SNDRV_PCM_FORMAT_U32_BE
	},
	{
		.sndif = XENSND_PCM_FORMAT_S32_LE,
		.alsa = SNDRV_PCM_FORMAT_S32_LE
	},
	{
		.sndif = XENSND_PCM_FORMAT_S32_BE,
		.alsa = SNDRV_PCM_FORMAT_S32_BE
	},
	{
		.sndif = XENSND_PCM_FORMAT_A_LAW,
		.alsa = SNDRV_PCM_FORMAT_A_LAW
	},
	{
		.sndif = XENSND_PCM_FORMAT_MU_LAW,
		.alsa = SNDRV_PCM_FORMAT_MU_LAW
	},
	{
		.sndif = XENSND_PCM_FORMAT_F32_LE,
		.alsa = SNDRV_PCM_FORMAT_FLOAT_LE
	},
	{
		.sndif = XENSND_PCM_FORMAT_F32_BE,
		.alsa = SNDRV_PCM_FORMAT_FLOAT_BE
	},
	{
		.sndif = XENSND_PCM_FORMAT_F64_LE,
		.alsa = SNDRV_PCM_FORMAT_FLOAT64_LE
	},
	{
		.sndif = XENSND_PCM_FORMAT_F64_BE,
		.alsa = SNDRV_PCM_FORMAT_FLOAT64_BE
	},
	{
		.sndif = XENSND_PCM_FORMAT_IEC958_SUBFRAME_LE,
		.alsa = SNDRV_PCM_FORMAT_IEC958_SUBFRAME_LE
	},
	{
		.sndif = XENSND_PCM_FORMAT_IEC958_SUBFRAME_BE,
		.alsa = SNDRV_PCM_FORMAT_IEC958_SUBFRAME_BE
	},
	{
		.sndif = XENSND_PCM_FORMAT_IMA_ADPCM,
		.alsa = SNDRV_PCM_FORMAT_IMA_ADPCM
	},
	{
		.sndif = XENSND_PCM_FORMAT_MPEG,
		.alsa = SNDRV_PCM_FORMAT_MPEG
	},
	{
		.sndif = XENSND_PCM_FORMAT_GSM,
		.alsa = SNDRV_PCM_FORMAT_GSM
	},
};

static int alsa_to_sndif_format(snd_pcm_format_t format)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ALSA_SNDIF_FORMATS); i++)
		if (ALSA_SNDIF_FORMATS[i].alsa == format)
			return ALSA_SNDIF_FORMATS[i].sndif;
	return -EINVAL;
}

static void stream_clear(struct pcm_stream_info *stream)
{
	stream->is_open = false;
	stream->be_cur_frame = 0;
	stream->out_frames = 0;
	atomic_set(&stream->hw_ptr, 0);
	stream->evt_pair->req.evt_next_id = 0;
	stream->evt_pair->evt.evt_next_id = 0;
	xen_front_sh_buf_clear(&stream->sh_buf);
}

static struct xensnd_req *be_stream_prepare_req(
	struct xen_front_evtchnl_info *evt_chnl, uint8_t operation)
{
	struct xensnd_req *req;

	req = RING_GET_REQUEST(&evt_chnl->u.req.ring,
		evt_chnl->u.req.ring.req_prod_pvt);
	req->operation = operation;
	req->id = evt_chnl->evt_next_id++;
	evt_chnl->evt_id = req->id;
	return req;
}

static void be_stream_free(struct pcm_stream_info *stream)
{
	xen_front_sh_buf_free(&stream->sh_buf);
	stream_clear(stream);
}

static int be_stream_do_io(struct xen_front_evtchnl_info *evt_chnl)
{
	if (unlikely(evt_chnl->state != EVTCHNL_STATE_CONNECTED))
		return -EIO;

	reinit_completion(&evt_chnl->u.req.completion);
	xen_front_evtchnl_flush(evt_chnl);
	return 0;
}

static int be_stream_wait_io(struct xen_front_evtchnl_info *evt_chnl)
{
	if (wait_for_completion_timeout(
			&evt_chnl->u.req.completion,
			msecs_to_jiffies(VSND_WAIT_BACK_MS)) <= 0)
		return -ETIMEDOUT;

	return evt_chnl->u.req.resp_status;
}

static int be_stream_open(struct pcm_stream_info *stream,
	snd_pcm_format_t format, unsigned int channels, unsigned int rate,
	uint32_t buffer_sz, uint32_t period_sz)
{
	struct xen_front_evtchnl_info *evt_chnl = &stream->evt_pair->req;
	struct xensnd_req *req;
	uint8_t sndif_format;
	unsigned long flags;
	int ret;

	sndif_format = alsa_to_sndif_format(format);
	if (sndif_format < 0) {
		dev_err(&evt_chnl->drv_info->xb_dev->dev,
			"Unsupported sample format: %d\n", format);
		return sndif_format;
	}

	spin_lock_irqsave(&evt_chnl->drv_info->io_lock, flags);
	stream->is_open = false;
	req = be_stream_prepare_req(evt_chnl, XENSND_OP_OPEN);
	req->op.open.pcm_format = sndif_format;
	req->op.open.pcm_channels = channels;
	req->op.open.pcm_rate = rate;
	req->op.open.buffer_sz = buffer_sz;
	req->op.open.period_sz = period_sz;
	req->op.open.gref_directory = xen_front_sh_buf_get_dir_start(
		&stream->sh_buf);

	ret = be_stream_do_io(evt_chnl);
	spin_unlock_irqrestore(&evt_chnl->drv_info->io_lock, flags);

	if (ret < 0)
		return ret;

	ret = be_stream_wait_io(evt_chnl);
	stream->is_open = ret < 0 ? false : true;
	return ret;
}

static int be_stream_close(struct pcm_stream_info *stream)
{
	struct xen_front_evtchnl_info *evt_chnl = &stream->evt_pair->req;
	struct xensnd_req *req;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&evt_chnl->drv_info->io_lock, flags);
	stream->is_open = false;
	req = be_stream_prepare_req(evt_chnl, XENSND_OP_CLOSE);

	ret = be_stream_do_io(evt_chnl);
	spin_unlock_irqrestore(&evt_chnl->drv_info->io_lock, flags);

	if (ret < 0)
		return ret;

	return be_stream_wait_io(evt_chnl);
}

static int be_stream_write(struct pcm_stream_info *stream,
	unsigned long pos, unsigned long count)
{
	struct xen_front_evtchnl_info *evt_chnl = &stream->evt_pair->req;
	struct xensnd_req *req;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&evt_chnl->drv_info->io_lock, flags);
	req = be_stream_prepare_req(evt_chnl, XENSND_OP_WRITE);
	req->op.rw.length = count;
	req->op.rw.offset = pos;

	ret = be_stream_do_io(evt_chnl);
	spin_unlock_irqrestore(&evt_chnl->drv_info->io_lock, flags);

	if (ret < 0)
		return ret;

	return be_stream_wait_io(evt_chnl);
}

static int be_stream_read(struct pcm_stream_info *stream,
	unsigned long pos, unsigned long count)
{
	struct xen_front_evtchnl_info *evt_chnl = &stream->evt_pair->req;
	struct xensnd_req *req;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&evt_chnl->drv_info->io_lock, flags);
	req = be_stream_prepare_req(evt_chnl, XENSND_OP_READ);
	req->op.rw.length = count;
	req->op.rw.offset = pos;

	ret = be_stream_do_io(evt_chnl);
	spin_unlock_irqrestore(&evt_chnl->drv_info->io_lock, flags);

	if (ret < 0)
		return ret;

	return be_stream_wait_io(evt_chnl);
}

static int be_stream_trigger(struct pcm_stream_info *stream, int type)
{
	struct xen_front_evtchnl_info *evt_chnl = &stream->evt_pair->req;
	struct xensnd_req *req;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&evt_chnl->drv_info->io_lock, flags);
	req = be_stream_prepare_req(evt_chnl, XENSND_OP_TRIGGER);
	req->op.trigger.type = type;

	ret = be_stream_do_io(evt_chnl);
	spin_unlock_irqrestore(&evt_chnl->drv_info->io_lock, flags);

	if (ret < 0)
		return ret;

	return be_stream_wait_io(evt_chnl);
}

static struct pcm_stream_info *stream_get(
	struct snd_pcm_substream *substream)
{
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct pcm_stream_info *stream;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		stream = &pcm_instance->streams_pb[substream->number];
	else
		stream = &pcm_instance->streams_cap[substream->number];
	return stream;
}

static int alsa_open(struct snd_pcm_substream *substream)
{
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct pcm_stream_info *stream = stream_get(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct drv_info *drv_info = pcm_instance->card_info->drv_info;
	unsigned long flags;

	/*
	 * return our HW properties: override defaults with those configured
	 * via XenStore
	 */
	runtime->hw = stream->pcm_hw;
	runtime->hw.info &= ~(SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_DOUBLE |
		SNDRV_PCM_INFO_BATCH |
		SNDRV_PCM_INFO_NONINTERLEAVED |
		SNDRV_PCM_INFO_RESUME |
		SNDRV_PCM_INFO_PAUSE);
	runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	stream->evt_pair = &drv_info->evt_pairs[stream->index];
	stream->evt_pair->req.state = EVTCHNL_STATE_CONNECTED;
	stream->evt_pair->evt.state = EVTCHNL_STATE_CONNECTED;
	stream->evt_pair->evt.u.evt.substream = substream;
	stream_clear(stream);
	spin_unlock_irqrestore(&drv_info->io_lock, flags);
	return 0;
}

static int alsa_close(struct snd_pcm_substream *substream)
{
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct pcm_stream_info *stream = stream_get(substream);
	struct drv_info *drv_info = pcm_instance->card_info->drv_info;
	unsigned long flags;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	stream->evt_pair->req.state = EVTCHNL_STATE_DISCONNECTED;
	stream->evt_pair->evt.state = EVTCHNL_STATE_DISCONNECTED;
	spin_unlock_irqrestore(&drv_info->io_lock, flags);
	return 0;
}

static int alsa_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct pcm_stream_info *stream = stream_get(substream);
	struct drv_info *drv_info = pcm_instance->card_info->drv_info;
	int ret;

	/*
	 * this callback may be called multiple times,
	 * so free the previously allocated shared buffer if any
	 */
	be_stream_free(stream);

	ret = xen_front_sh_buf_alloc(drv_info->xb_dev, &stream->sh_buf,
		params_buffer_bytes(params));
	if (ret < 0)
		goto fail;

	return 0;

fail:
	be_stream_free(stream);
	dev_err(&drv_info->xb_dev->dev,
		"Failed to allocate buffers for stream idx %d\n",
		stream->index);
	return ret;
}

static int alsa_hw_free(struct snd_pcm_substream *substream)
{
	struct pcm_stream_info *stream = stream_get(substream);
	int ret;

	ret = be_stream_close(stream);
	be_stream_free(stream);
	return ret;
}

static int alsa_prepare(struct snd_pcm_substream *substream)
{
	struct pcm_stream_info *stream = stream_get(substream);

	if (!stream->is_open) {
		struct snd_pcm_runtime *runtime = substream->runtime;

		return be_stream_open(stream, runtime->format,
			runtime->channels, runtime->rate,
			snd_pcm_lib_buffer_bytes(substream),
			snd_pcm_lib_period_bytes(substream));
	}

	return 0;
}

static int alsa_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int type;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		type = XENSND_OP_TRIGGER_START;
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		type = XENSND_OP_TRIGGER_RESUME;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		type = XENSND_OP_TRIGGER_STOP;
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
		type = XENSND_OP_TRIGGER_PAUSE;
		break;

	default:
		return -EINVAL;
	}

	return be_stream_trigger(stream_get(substream), type);
}

void xen_front_alsa_handle_cur_pos(struct xen_front_evtchnl_info *channel,
	uint64_t pos_bytes)
{
	struct snd_pcm_substream *substream = channel->u.evt.substream;
	struct pcm_stream_info *stream = stream_get(substream);
	snd_pcm_uframes_t delta, new_hw_ptr, cur_frame;

	cur_frame = bytes_to_frames(substream->runtime, pos_bytes);

	delta = cur_frame - stream->be_cur_frame;
	stream->be_cur_frame = cur_frame;

	new_hw_ptr = (snd_pcm_uframes_t)atomic_read(&stream->hw_ptr);
	new_hw_ptr = (new_hw_ptr + delta) % substream->runtime->buffer_size;
	atomic_set(&stream->hw_ptr, (int)new_hw_ptr);

	stream->out_frames += delta;
	if (stream->out_frames > substream->runtime->period_size) {
		stream->out_frames %= substream->runtime->period_size;
		snd_pcm_period_elapsed(substream);
	}
}

static snd_pcm_uframes_t alsa_pointer(struct snd_pcm_substream *substream)
{
	struct pcm_stream_info *stream = stream_get(substream);

	return (snd_pcm_uframes_t)atomic_read(&stream->hw_ptr);
}

static int alsa_pb_copy_user(struct snd_pcm_substream *substream,
	int channel, unsigned long pos, void __user *src,
	unsigned long count)
{
	struct pcm_stream_info *stream = stream_get(substream);

	if (unlikely(pos + count > stream->sh_buf.vbuffer_sz))
		return -EINVAL;

	if (copy_from_user(stream->sh_buf.vbuffer + pos, src, count))
		return -EFAULT;

	return be_stream_write(stream, pos, count);
}

static int alsa_pb_copy_kernel(struct snd_pcm_substream *substream,
	int channel, unsigned long pos, void *src, unsigned long count)
{
	struct pcm_stream_info *stream = stream_get(substream);

	if (unlikely(pos + count > stream->sh_buf.vbuffer_sz))
		return -EINVAL;

	memcpy(stream->sh_buf.vbuffer + pos, src, count);
	return be_stream_write(stream, pos, count);
}

static int alsa_cap_copy_user(struct snd_pcm_substream *substream,
	int channel, unsigned long pos, void __user *dst,
	unsigned long count)
{
	struct pcm_stream_info *stream = stream_get(substream);
	int ret;

	if (unlikely(pos + count > stream->sh_buf.vbuffer_sz))
		return -EINVAL;

	ret = be_stream_read(stream, pos, count);
	if (ret < 0)
		return ret;

	return copy_to_user(dst, stream->sh_buf.vbuffer + pos, count) ?
		-EFAULT : 0;
}

static int alsa_cap_copy_kernel(struct snd_pcm_substream *substream,
	int channel, unsigned long pos, void *dst, unsigned long count)
{
	struct pcm_stream_info *stream = stream_get(substream);
	int ret;

	if (unlikely(pos + count > stream->sh_buf.vbuffer_sz))
		return -EINVAL;

	ret = be_stream_read(stream, pos, count);
	if (ret < 0)
		return ret;

	memcpy(dst, stream->sh_buf.vbuffer + pos, count);
	return 0;
}

static int alsa_pb_fill_silence(struct snd_pcm_substream *substream,
	int channel, unsigned long pos, unsigned long count)
{
	struct pcm_stream_info *stream = stream_get(substream);

	if (unlikely(pos + count > stream->sh_buf.vbuffer_sz))
		return -EINVAL;

	memset(stream->sh_buf.vbuffer + pos, 0, count);
	return be_stream_write(stream, pos, count);
}

/*
 * FIXME: The mmaped data transfer is asynchronous and there is no
 * ack signal from user-space when it is done. This is the
 * reason it is not implemented in the PV driver as we do need
 * to know when the buffer can be transferred to the backend.
 */

static struct snd_pcm_ops snd_drv_alsa_playback_ops = {
	.open = alsa_open,
	.close = alsa_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = alsa_hw_params,
	.hw_free = alsa_hw_free,
	.prepare = alsa_prepare,
	.trigger = alsa_trigger,
	.pointer = alsa_pointer,
	.copy_user = alsa_pb_copy_user,
	.copy_kernel = alsa_pb_copy_kernel,
	.fill_silence = alsa_pb_fill_silence,
};

static struct snd_pcm_ops snd_drv_alsa_capture_ops = {
	.open = alsa_open,
	.close = alsa_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = alsa_hw_params,
	.hw_free = alsa_hw_free,
	.prepare = alsa_prepare,
	.trigger = alsa_trigger,
	.pointer = alsa_pointer,
	.copy_user = alsa_cap_copy_user,
	.copy_kernel = alsa_cap_copy_kernel,
};

static int new_pcm_instance(struct card_info *card_info,
	struct xen_front_cfg_pcm_instance *instance_cfg,
	struct pcm_instance_info *pcm_instance_info)
{
	struct snd_pcm *pcm;
	int ret, i;

	dev_dbg(&card_info->drv_info->xb_dev->dev,
		"New PCM device \"%s\" with id %d playback %d capture %d",
		instance_cfg->name,
		instance_cfg->device_id,
		instance_cfg->num_streams_pb,
		instance_cfg->num_streams_cap);

	pcm_instance_info->card_info = card_info;

	pcm_instance_info->pcm_hw = instance_cfg->pcm_hw;

	if (instance_cfg->num_streams_pb) {
		pcm_instance_info->streams_pb = devm_kcalloc(
			&card_info->card->card_dev,
			instance_cfg->num_streams_pb,
			sizeof(struct pcm_stream_info),
			GFP_KERNEL);
		if (!pcm_instance_info->streams_pb)
			return -ENOMEM;
	}

	if (instance_cfg->num_streams_cap) {
		pcm_instance_info->streams_cap = devm_kcalloc(
			&card_info->card->card_dev,
			instance_cfg->num_streams_cap,
			sizeof(struct pcm_stream_info),
			GFP_KERNEL);
		if (!pcm_instance_info->streams_cap)
			return -ENOMEM;
	}

	pcm_instance_info->num_pcm_streams_pb =
			instance_cfg->num_streams_pb;
	pcm_instance_info->num_pcm_streams_cap =
			instance_cfg->num_streams_cap;

	for (i = 0; i < pcm_instance_info->num_pcm_streams_pb; i++) {
		pcm_instance_info->streams_pb[i].pcm_hw =
			instance_cfg->streams_pb[i].pcm_hw;
		pcm_instance_info->streams_pb[i].index =
			instance_cfg->streams_pb[i].index;
	}

	for (i = 0; i < pcm_instance_info->num_pcm_streams_cap; i++) {
		pcm_instance_info->streams_cap[i].pcm_hw =
			instance_cfg->streams_cap[i].pcm_hw;
		pcm_instance_info->streams_cap[i].index =
			instance_cfg->streams_cap[i].index;
	}

	ret = snd_pcm_new(card_info->card, instance_cfg->name,
			instance_cfg->device_id,
			instance_cfg->num_streams_pb,
			instance_cfg->num_streams_cap,
			&pcm);
	if (ret < 0)
		return ret;

	pcm->private_data = pcm_instance_info;
	pcm->info_flags = 0;
	/* we want to handle all PCM operations in non-atomic context */
	pcm->nonatomic = true;
	strncpy(pcm->name, "Virtual card PCM", sizeof(pcm->name));

	if (instance_cfg->num_streams_pb)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
				&snd_drv_alsa_playback_ops);

	if (instance_cfg->num_streams_cap)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
				&snd_drv_alsa_capture_ops);

	pcm_instance_info->pcm = pcm;
	return 0;
}

static int snd_drv_probe(struct platform_device *pdev)
{
	struct card_info *card_info;
	struct xen_front_cfg_card_plat_data *platdata;
	struct snd_card *card;
	int ret, i;

	platdata = dev_get_platdata(&pdev->dev);

	dev_dbg(&pdev->dev, "Creating virtual sound card\n");

	ret = snd_card_new(&pdev->dev, 0, XENSND_DRIVER_NAME, THIS_MODULE,
		sizeof(struct card_info), &card);
	if (ret < 0)
		return ret;

	card_info = card->private_data;
	card_info->drv_info = platdata->drv_info;
	card_info->card = card;
	card_info->pcm_instances = devm_kcalloc(&pdev->dev,
			platdata->cfg_card.num_pcm_instances,
			sizeof(struct pcm_instance_info), GFP_KERNEL);
	if (!card_info->pcm_instances) {
		ret = -ENOMEM;
		goto fail;
	}

	card_info->num_pcm_instances = platdata->cfg_card.num_pcm_instances;
	card_info->pcm_hw = platdata->cfg_card.pcm_hw;

	for (i = 0; i < platdata->cfg_card.num_pcm_instances; i++) {
		ret = new_pcm_instance(card_info,
			&platdata->cfg_card.pcm_instances[i],
			&card_info->pcm_instances[i]);
		if (ret < 0)
			goto fail;
	}

	strncpy(card->driver, XENSND_DRIVER_NAME, sizeof(card->driver));
	strncpy(card->shortname, platdata->cfg_card.name_short,
		sizeof(card->shortname));
	strncpy(card->longname, platdata->cfg_card.name_long,
		sizeof(card->longname));

	ret = snd_card_register(card);
	if (ret < 0)
		goto fail;

	platform_set_drvdata(pdev, card);
	return 0;

fail:
	snd_card_free(card);
	return ret;
}

static int snd_drv_remove(struct platform_device *pdev)
{
	struct card_info *info;
	struct snd_card *card = platform_get_drvdata(pdev);

	info = card->private_data;
	dev_dbg(&pdev->dev, "Removing virtual sound card %d\n",
		info->card->number);
	snd_card_free(card);
	return 0;
}

static struct platform_driver snd_drv_info = {
	.probe	= snd_drv_probe,
	.remove	= snd_drv_remove,
	.driver	= {
		.name	= XENSND_DRIVER_NAME,
	},
};

void xen_front_alsa_cleanup(struct drv_info *drv_info)
{
	if (!drv_info->snd_drv_registered)
		return;

	if (drv_info->snd_drv_pdev) {
		struct platform_device *snd_drv_pdev;

		snd_drv_pdev = drv_info->snd_drv_pdev;
		if (snd_drv_pdev)
			platform_device_unregister(snd_drv_pdev);
	}

	platform_driver_unregister(&snd_drv_info);
	drv_info->snd_drv_registered = false;
}

int xen_front_alsa_init(struct drv_info *drv_info)
{
	struct platform_device *snd_drv_pdev;
	int ret;

	ret = platform_driver_register(&snd_drv_info);
	if (ret < 0)
		return ret;

	drv_info->snd_drv_registered = true;
	/* pass card configuration via platform data */
	snd_drv_pdev = platform_device_register_data(NULL,
		XENSND_DRIVER_NAME, 0, &drv_info->cfg_plat_data,
		sizeof(drv_info->cfg_plat_data));
	if (IS_ERR(snd_drv_pdev))
		goto fail;

	drv_info->snd_drv_pdev = snd_drv_pdev;
	return 0;

fail:
	dev_err(&drv_info->xb_dev->dev,
		"failed to register virtual sound driver\n");
	xen_front_alsa_cleanup(drv_info);
	return -ENODEV;
}
