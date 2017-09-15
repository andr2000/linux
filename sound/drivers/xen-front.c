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
 * Based on: sound/drivers/dummy.c
 *
 * Copyright (C) 2016-2017 EPAM Systems Inc.
 *
 * Author: Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 */

#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include <sound/core.h>
#include <sound/pcm.h>

#include <xen/events.h>
#include <xen/grant_table.h>
#include <xen/platform_pci.h>
#include <xen/xen.h>
#include <xen/xenbus.h>

#include <xen/interface/io/sndif.h>

/*
 * FIXME: usage of grant reference 0 as invalid grant reference:
 * grant reference 0 is valid, but never exposed to a PV driver,
 * because of the fact it is already in use/reserved by the PV console.
 */
#define GRANT_INVALID_REF	0
/* timeout in ms to wait for backend to respond */
#define VSND_WAIT_BACK_MS	3000
/* maximum number of supported streams */
#define VSND_MAX_STREAM		8

enum evtchnl_state {
	EVTCHNL_STATE_DISCONNECTED,
	EVTCHNL_STATE_CONNECTED,
};

enum evtchnl_type {
	EVTCHNL_TYPE_REQ,
	EVTCHNL_TYPE_EVT,
};

struct evtchnl_info {
	struct drv_info *drv_info;
	int gref;
	int port;
	int irq;
	int index;
	/* state of the event channel */
	enum evtchnl_state state;
	enum evtchnl_type type;
	/* either response id or incoming event id */
	uint16_t evt_id;
	/* next request id or next expected event id */
	uint16_t evt_next_id;
	union {
		struct {
			struct xen_sndif_front_ring ring;
			struct completion completion;
			/* latest response status */
			int resp_status;
			/* HW parameters received from the backend */
			struct xensnd_get_hw_resp pcm_hw;
		} req;
		struct {
			struct xensnd_event_page *page;
			/* this is needed to handle XENSND_EVT_CUR_POS event */
			struct snd_pcm_substream *substream;
		} evt;
	} u;
};

struct evtchnl_pair_info {
	struct evtchnl_info req;
	struct evtchnl_info evt;
};

struct sh_buf_info {
	int num_grefs;
	grant_ref_t *grefs;
	uint8_t *vdirectory;
	uint8_t *vbuffer;
	size_t vbuffer_sz;
};

struct pcm_stream_info {
	int index;
	struct snd_pcm_hardware pcm_hw;
	struct evtchnl_pair_info *evt_pair;
	bool is_open;
	struct sh_buf_info sh_buf;

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

struct cfg_stream {
	int index;
	char *xenstore_path;
	struct snd_pcm_hardware pcm_hw;
};

struct cfg_pcm_instance {
	char name[80];
	int device_id;
	struct snd_pcm_hardware pcm_hw;
	int  num_streams_pb;
	struct cfg_stream *streams_pb;
	int  num_streams_cap;
	struct cfg_stream *streams_cap;
};

struct cfg_card {
	char name_short[32];
	char name_long[80];
	struct snd_pcm_hardware pcm_hw;
	int num_pcm_instances;
	struct cfg_pcm_instance *pcm_instances;
};

struct card_plat_data {
	struct drv_info *drv_info;
	struct cfg_card cfg_card;
};

struct drv_info {
	struct xenbus_device *xb_dev;
	spinlock_t io_lock;
	struct mutex mutex;
	bool snd_drv_registered;
	struct platform_device *snd_drv_pdev;
	int num_evt_pairs;
	struct evtchnl_pair_info *evt_pairs;
	struct card_plat_data cfg_plat_data;
};

static inline void evtchnl_flush(struct evtchnl_info *channel);
static inline void sh_buf_clear(struct sh_buf_info *buf);
static void sh_buf_free(struct sh_buf_info *buf);
static int sh_buf_alloc(struct xenbus_device *xb_dev, struct sh_buf_info *buf,
	unsigned int buffer_size);
static grant_ref_t sh_buf_get_dir_start(struct sh_buf_info *buf);

#define MAX_BUFFER_SIZE		(64 * 1024)
#define MIN_PERIOD_SIZE		64
#define MAX_PERIOD_SIZE		MAX_BUFFER_SIZE
#define USE_FORMATS		(SNDRV_PCM_FMTBIT_U8 | \
				 SNDRV_PCM_FMTBIT_S16_LE)
#define USE_RATE		(SNDRV_PCM_RATE_CONTINUOUS | \
				 SNDRV_PCM_RATE_8000_48000)
#define USE_RATE_MIN		5512
#define USE_RATE_MAX		48000
#define USE_CHANNELS_MIN	1
#define USE_CHANNELS_MAX	2
#define USE_PERIODS_MIN		2
#define USE_PERIODS_MAX		(MAX_BUFFER_SIZE / MIN_PERIOD_SIZE)

static struct snd_pcm_hardware snd_drv_pcm_hw_default = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_RESUME |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.formats = USE_FORMATS,
	.rates = USE_RATE,
	.rate_min = USE_RATE_MIN,
	.rate_max = USE_RATE_MAX,
	.channels_min = USE_CHANNELS_MIN,
	.channels_max = USE_CHANNELS_MAX,
	.buffer_bytes_max = MAX_BUFFER_SIZE,
	.period_bytes_min = MIN_PERIOD_SIZE,
	.period_bytes_max = MAX_PERIOD_SIZE,
	.periods_min = USE_PERIODS_MIN,
	.periods_max = USE_PERIODS_MAX,
	.fifo_size = 0,
};

static void snd_drv_copy_pcm_hw(struct snd_pcm_hardware *dst,
	struct snd_pcm_hardware *src,
	struct snd_pcm_hardware *ref_pcm_hw)
{
	*dst = *ref_pcm_hw;

	if (src->formats)
		dst->formats = src->formats;

	if (src->buffer_bytes_max)
		dst->buffer_bytes_max = src->buffer_bytes_max;

	if (src->period_bytes_min)
		dst->period_bytes_min = src->period_bytes_min;

	if (src->period_bytes_max)
		dst->period_bytes_max = src->period_bytes_max;

	if (src->periods_min)
		dst->periods_min = src->periods_min;

	if (src->periods_max)
		dst->periods_max = src->periods_max;

	if (src->rates)
		dst->rates = src->rates;

	if (src->rate_min)
		dst->rate_min = src->rate_min;

	if (src->rate_max)
		dst->rate_max = src->rate_max;

	if (src->channels_min)
		dst->channels_min = src->channels_min;

	if (src->channels_max)
		dst->channels_max = src->channels_max;

	if (src->buffer_bytes_max) {
		dst->buffer_bytes_max = src->buffer_bytes_max;
		dst->period_bytes_max = src->buffer_bytes_max /
			src->periods_min;
		dst->periods_max = dst->buffer_bytes_max /
			dst->period_bytes_min;
	}
}

struct ALSA_SNDIF_SAMPLE_FORMAT {
	uint8_t sndif;
	snd_pcm_format_t alsa;
};

static struct ALSA_SNDIF_SAMPLE_FORMAT alsa_sndif_formats[] = {
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

	for (i = 0; i < ARRAY_SIZE(alsa_sndif_formats); i++)
		if (alsa_sndif_formats[i].alsa == format)
			return alsa_sndif_formats[i].sndif;
	return -EINVAL;
}

static void snd_drv_stream_clear(struct pcm_stream_info *stream)
{
	stream->is_open = false;
	stream->be_cur_frame = 0;
	stream->out_frames = 0;
	atomic_set(&stream->hw_ptr, 0);
	stream->evt_pair->req.evt_next_id = 0;
	stream->evt_pair->evt.evt_next_id = 0;
	sh_buf_clear(&stream->sh_buf);
}

static struct xensnd_req *snd_drv_be_stream_prepare_req(
	struct evtchnl_info *evtchnl, uint8_t operation)
{
	struct xensnd_req *req;

	req = RING_GET_REQUEST(&evtchnl->u.req.ring,
		evtchnl->u.req.ring.req_prod_pvt);
	req->operation = operation;
	req->id = evtchnl->evt_next_id++;
	evtchnl->evt_id = req->id;
	return req;
}

static void snd_drv_be_stream_free(struct pcm_stream_info *stream)
{
	sh_buf_free(&stream->sh_buf);
	snd_drv_stream_clear(stream);
}

static int snd_drv_be_stream_do_io(struct evtchnl_info *evtchnl)
{
	if (unlikely(evtchnl->state != EVTCHNL_STATE_CONNECTED))
		return -EIO;

	reinit_completion(&evtchnl->u.req.completion);
	evtchnl_flush(evtchnl);
	return 0;
}

static inline int snd_drv_be_stream_wait_io(struct evtchnl_info *evtchnl)
{
	if (wait_for_completion_timeout(
			&evtchnl->u.req.completion,
			msecs_to_jiffies(VSND_WAIT_BACK_MS)) <= 0) {
		printk("%s timeout\n", __FUNCTION__);
		return -ETIMEDOUT;
	}
	printk("%s response %d\n", __FUNCTION__, evtchnl->u.req.resp_status);
	return evtchnl->u.req.resp_status;
}

static int snd_drv_be_stream_open(struct snd_pcm_substream *substream,
	struct pcm_stream_info *stream)
{
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct drv_info *drv_info;
	struct xensnd_req *req;
	struct evtchnl_info *evt_chnl;
	unsigned long flags;
	int ret;

	drv_info = pcm_instance->card_info->drv_info;

	ret = alsa_to_sndif_format(runtime->format);
	if (ret < 0) {
		dev_err(&drv_info->xb_dev->dev,
			"Unsupported sample format: %d\n", runtime->format);
		return ret;
	}

	spin_lock_irqsave(&drv_info->io_lock, flags);
	stream->is_open = false;
	evt_chnl = &stream->evt_pair->req;
	req = snd_drv_be_stream_prepare_req(evt_chnl, XENSND_OP_OPEN);
	req->op.open.pcm_format = (uint8_t)ret;
	req->op.open.pcm_channels = runtime->channels;
	req->op.open.pcm_rate = runtime->rate;
	req->op.open.buffer_sz = runtime->buffer_size;
	req->op.open.gref_directory = sh_buf_get_dir_start(&stream->sh_buf);

	printk("buffer_size %lu stream->sh_buf.vbuffer_sz %zu\n", runtime->buffer_size, stream->sh_buf.vbuffer_sz);
	printk("period_size %lu\n", runtime->period_size);


	ret = snd_drv_be_stream_do_io(evt_chnl);
	spin_unlock_irqrestore(&drv_info->io_lock, flags);

	if (ret < 0)
		return ret;

	ret = snd_drv_be_stream_wait_io(evt_chnl);
	stream->is_open = ret < 0 ? false : true;
	return ret;
}

static int snd_drv_be_stream_close(struct snd_pcm_substream *substream,
	struct pcm_stream_info *stream)
{
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct drv_info *drv_info;
	struct xensnd_req *req;
	struct evtchnl_info *evt_chnl;
	unsigned long flags;
	int ret;

	drv_info = pcm_instance->card_info->drv_info;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	stream->is_open = false;
	evt_chnl = &stream->evt_pair->req;
	req = snd_drv_be_stream_prepare_req(evt_chnl, XENSND_OP_CLOSE);

	ret = snd_drv_be_stream_do_io(evt_chnl);
	spin_unlock_irqrestore(&drv_info->io_lock, flags);

	if (ret < 0)
		return ret;

	return snd_drv_be_stream_wait_io(evt_chnl);
}

static int snd_drv_be_stream_get_hw_params(struct snd_pcm_substream *substream,
	struct pcm_stream_info *stream, struct xensnd_get_hw_resp *hw)
{
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct drv_info *drv_info;
	struct xensnd_req *req;
	struct evtchnl_info *evt_chnl;
	unsigned long flags;
	int ret;

	drv_info = pcm_instance->card_info->drv_info;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	evt_chnl = &stream->evt_pair->req;
	req = snd_drv_be_stream_prepare_req(evt_chnl, XENSND_OP_GET_HW_PARAMS);

	ret = snd_drv_be_stream_do_io(evt_chnl);
	spin_unlock_irqrestore(&drv_info->io_lock, flags);

	if (ret < 0)
		return ret;

	ret = snd_drv_be_stream_wait_io(evt_chnl);

	if (ret < 0)
		return ret;

	*hw = evt_chnl->u.req.pcm_hw;
	return 0;
}

static int snd_drv_be_stream_trigger(struct snd_pcm_substream *substream,
	struct pcm_stream_info *stream, int type)
{
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct drv_info *drv_info;
	struct xensnd_req *req;
	struct evtchnl_info *evt_chnl;
	unsigned long flags;
	int ret;

	drv_info = pcm_instance->card_info->drv_info;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	evt_chnl = &stream->evt_pair->req;
	req = snd_drv_be_stream_prepare_req(evt_chnl, XENSND_OP_TRIGGER);
	req->op.trigger.type = type;

	ret = snd_drv_be_stream_do_io(evt_chnl);
	spin_unlock_irqrestore(&drv_info->io_lock, flags);

	if (ret < 0)
		return ret;

	return snd_drv_be_stream_wait_io(evt_chnl);
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

static int snd_drv_alsa_open(struct snd_pcm_substream *substream)
{
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct pcm_stream_info *stream = stream_get(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct drv_info *drv_info;
	struct xensnd_get_hw_resp backend_hw;
	unsigned long flags;
	int ret;

	printk("%s %d index %d\n", __FUNCTION__, __LINE__, stream->index);
	/*
	 * return our HW properties: override defaults with those configured
	 * via XenStore
	 */
	snd_drv_copy_pcm_hw(&runtime->hw, &stream->pcm_hw,
		&pcm_instance->pcm_hw);
	runtime->hw.info &= ~(SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_DOUBLE |
		SNDRV_PCM_INFO_BATCH |
		SNDRV_PCM_INFO_NONINTERLEAVED |
		SNDRV_PCM_INFO_RESUME |
		SNDRV_PCM_INFO_PAUSE);
	runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;

	drv_info = pcm_instance->card_info->drv_info;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	stream->evt_pair = &drv_info->evt_pairs[stream->index];
	snd_drv_stream_clear(stream);
	stream->evt_pair->req.state = EVTCHNL_STATE_CONNECTED;
	stream->evt_pair->evt.state = EVTCHNL_STATE_CONNECTED;
	stream->evt_pair->evt.u.evt.substream = substream;
	printk("%s %d substream %p\n", __FUNCTION__, __LINE__, stream->evt_pair->evt.u.evt.substream);
	printk("%s %d channel %p\n", __FUNCTION__, __LINE__, &stream->evt_pair->evt);
	spin_unlock_irqrestore(&drv_info->io_lock, flags);

	/* now get HW parameters from the backend */
	ret = snd_drv_be_stream_get_hw_params(substream, stream, &backend_hw);
	if (ret < 0)
		return ret;

	runtime->hw.period_bytes_min = backend_hw.period_sz_min;
	runtime->hw.period_bytes_max = backend_hw.period_sz_max;
	runtime->hw.buffer_bytes_max = backend_hw.buffer_sz_max;
	runtime->hw.periods_max = backend_hw.buffer_sz_max /
		backend_hw.period_sz_min;
	return 0;
}

static int snd_drv_alsa_close(struct snd_pcm_substream *substream)
{
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct pcm_stream_info *stream = stream_get(substream);
	struct drv_info *drv_info;
	unsigned long flags;

	printk("%s %d\n", __FUNCTION__, __LINE__);
	drv_info = pcm_instance->card_info->drv_info;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	stream->evt_pair->req.state = EVTCHNL_STATE_DISCONNECTED;
	stream->evt_pair->evt.state = EVTCHNL_STATE_DISCONNECTED;
	spin_unlock_irqrestore(&drv_info->io_lock, flags);
	return 0;
}

static int snd_drv_alsa_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct pcm_stream_info *stream = stream_get(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct drv_info *drv_info;
	unsigned int buffer_size;
	int ret;

	printk("%s %d\n", __FUNCTION__, __LINE__);

	printk("buffer_size %lu\n", runtime->buffer_size);
	printk("period_size %lu\n", runtime->period_size);

	/*
	 * this callback may be called multiple times,
	 * so free the previously allocated shared buffer if any
	 */
	snd_drv_be_stream_free(stream);

	drv_info = pcm_instance->card_info->drv_info;
	buffer_size = params_buffer_bytes(params);

	printk("buffer_size %u, bytes\n", buffer_size);

	ret = sh_buf_alloc(drv_info->xb_dev, &stream->sh_buf, buffer_size);
	if (ret < 0)
		goto fail;

	return 0;

fail:
	dev_err(&drv_info->xb_dev->dev,
		"Failed to allocate buffers for stream idx %d\n",
		stream->index);
	snd_drv_be_stream_free(stream);
	return ret;
}

static int snd_drv_alsa_hw_free(struct snd_pcm_substream *substream)
{
	struct pcm_stream_info *stream = stream_get(substream);
	int ret;

	ret = snd_drv_be_stream_close(substream, stream);
	snd_drv_be_stream_free(stream);
	return ret;
}

static int snd_drv_alsa_prepare(struct snd_pcm_substream *substream)
{
	struct pcm_stream_info *stream = stream_get(substream);

	if (!stream->is_open)
		return snd_drv_be_stream_open(substream, stream);

	return 0;
}

static int snd_drv_alsa_trigger(struct snd_pcm_substream *substream, int cmd)
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
		return 0;
	}

	return snd_drv_be_stream_trigger(substream,
		stream_get(substream), type);
}

static void snd_drv_handle_appl_ptr(struct evtchnl_info *channel,
	uint64_t at)
{
	struct snd_pcm_substream *substream = channel->u.evt.substream;
	struct pcm_stream_info *stream = stream_get(substream);
	snd_pcm_uframes_t delta, new_hw_ptr, cur_frame;

	cur_frame = bytes_to_frames(substream->runtime, at);
	printk("%s cur_frame %lu\n", __FUNCTION__, cur_frame);

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

static inline snd_pcm_uframes_t snd_drv_alsa_pointer(
	struct snd_pcm_substream *substream)
{
	struct pcm_stream_info *stream = stream_get(substream);

	printk("runtime->hw_ptr_base %lu\n", substream->runtime->hw_ptr_base);
	return (snd_pcm_uframes_t)atomic_read(&stream->hw_ptr);
}

static int snd_drv_alsa_playback_do_write(struct snd_pcm_substream *substream,
	unsigned long pos, unsigned long count)
{
	struct pcm_stream_info *stream = stream_get(substream);
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct drv_info *drv_info;
	struct xensnd_req *req;
	struct evtchnl_info *evt_chnl;
	unsigned long flags;
	int ret;

	drv_info = pcm_instance->card_info->drv_info;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	evt_chnl = &stream->evt_pair->req;
	req = snd_drv_be_stream_prepare_req(evt_chnl, XENSND_OP_WRITE);
	req->op.rw.length = count;
	req->op.rw.offset = pos;

	ret = snd_drv_be_stream_do_io(evt_chnl);
	printk("%s %d do io ret = %d, count %lu pos %lu\n", __FUNCTION__, __LINE__, ret, count, pos);
	spin_unlock_irqrestore(&drv_info->io_lock, flags);

	if (ret < 0)
		return ret;

	return snd_drv_be_stream_wait_io(evt_chnl);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
static int snd_drv_alsa_playback_copy_user(struct snd_pcm_substream *substream,
		int channel, unsigned long pos, void __user *src,
		unsigned long count)
{
	struct pcm_stream_info *stream = stream_get(substream);

	printk("%s %d count %lu\n", __FUNCTION__, __LINE__, count);
	if (unlikely(pos + count > stream->sh_buf.vbuffer_sz))
		return -EINVAL;

	if (copy_from_user(stream->sh_buf.vbuffer + pos, src, count))
		return -EFAULT;

	return snd_drv_alsa_playback_do_write(substream, pos, count);
}

static int snd_drv_alsa_playback_copy_kernel(struct snd_pcm_substream *substream,
		int channel, unsigned long pos, void *src, unsigned long count)
{
	struct pcm_stream_info *stream = stream_get(substream);

	if (unlikely(pos + count > stream->sh_buf.vbuffer_sz))
		return -EINVAL;

	memcpy(stream->sh_buf.vbuffer + pos, src, count);
	return snd_drv_alsa_playback_do_write(substream, pos, count);
}
#else
static int snd_drv_alsa_playback_copy_user(struct snd_pcm_substream *substream,
	int channel, snd_pcm_uframes_t pos, void __user *src,
	snd_pcm_uframes_t count)
{
	struct pcm_stream_info *stream = stream_get(substream);

	count = frames_to_bytes(substream->runtime, count);
	pos = frames_to_bytes(substream->runtime, pos);

	printk("%s %d count %lu\n", __FUNCTION__, __LINE__, count);
	if (unlikely(pos + count > stream->sh_buf.vbuffer_sz))
		return -EINVAL;

	if (copy_from_user(stream->sh_buf.vbuffer + pos, src, count))
		return -EFAULT;

	return snd_drv_alsa_playback_do_write(substream, pos, count);
}
#endif

static int snd_drv_alsa_playback_do_read(struct snd_pcm_substream *substream,
	unsigned long pos, unsigned long count)
{
	struct pcm_stream_info *stream = stream_get(substream);
	struct pcm_instance_info *pcm_instance =
		snd_pcm_substream_chip(substream);
	struct drv_info *drv_info;
	struct xensnd_req *req;
	struct evtchnl_info *evt_chnl;
	unsigned long flags;
	int ret;

	drv_info = pcm_instance->card_info->drv_info;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	evt_chnl = &stream->evt_pair->req;
	req = snd_drv_be_stream_prepare_req(evt_chnl, XENSND_OP_READ);
	req->op.rw.length = count;
	req->op.rw.offset = pos;

	ret = snd_drv_be_stream_do_io(evt_chnl);
	spin_unlock_irqrestore(&drv_info->io_lock, flags);

	if (ret < 0)
		return ret;

	return snd_drv_be_stream_wait_io(evt_chnl);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
static int snd_drv_alsa_capture_copy_user(struct snd_pcm_substream *substream,
		int channel, unsigned long pos, void __user *dst,
		unsigned long count)
{
	struct pcm_stream_info *stream = stream_get(substream);
	int ret;

	if (unlikely(pos + count > stream->sh_buf.vbuffer_sz))
		return -EINVAL;

	ret = snd_drv_alsa_playback_do_read(substream, pos, count);
	if (ret < 0)
		return ret;

	return copy_to_user(dst, stream->sh_buf.vbuffer + pos, count) ?
		-EFAULT : 0;
}

static int snd_drv_alsa_capture_copy_kernel(struct snd_pcm_substream *substream,
		int channel, unsigned long pos, void *dst, unsigned long count)
{
	struct pcm_stream_info *stream = stream_get(substream);
	int ret;

	if (unlikely(pos + count > stream->sh_buf.vbuffer_sz))
		return -EINVAL;

	ret = snd_drv_alsa_playback_do_read(substream, pos, count);
	if (ret < 0)
		return ret;

	memcpy(dst, stream->sh_buf.vbuffer + pos, count);
	return 0;
}
#else
static int snd_drv_alsa_capture_copy_user(struct snd_pcm_substream *substream,
	int channel, snd_pcm_uframes_t pos, void __user *dst,
	snd_pcm_uframes_t count)
{
	struct pcm_stream_info *stream = stream_get(substream);
	int ret;

	count = frames_to_bytes(substream->runtime, count);
	pos = frames_to_bytes(substream->runtime, pos);

	if (unlikely(pos + count > stream->sh_buf.vbuffer_sz))
		return -EINVAL;

	ret = snd_drv_alsa_playback_do_read(substream, pos, count);
	if (ret < 0)
		return ret;

	return copy_to_user(dst, stream->sh_buf.vbuffer + pos, count) ?
		-EFAULT : 0;
}
#endif

static int snd_drv_alsa_playback_fill_silence(struct snd_pcm_substream *substream,
	int channel, unsigned long pos, unsigned long count)
{
	struct pcm_stream_info *stream = stream_get(substream);

	if (unlikely(pos + count > stream->sh_buf.vbuffer_sz))
		return -EINVAL;

	memset(stream->sh_buf.vbuffer + pos, 0, count);
	return snd_drv_alsa_playback_do_write(substream, pos, count);
}

/*
 * FIXME: The mmaped data transfer is asynchronous and there is no
 * ack signal from user-space when it is done. This is the
 * reason it is not implemented in the PV driver as we do need
 * to know when the buffer can be transferred to the backend.
 */

static struct snd_pcm_ops snd_drv_alsa_playback_ops = {
	.open = snd_drv_alsa_open,
	.close = snd_drv_alsa_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_drv_alsa_hw_params,
	.hw_free = snd_drv_alsa_hw_free,
	.prepare = snd_drv_alsa_prepare,
	.trigger = snd_drv_alsa_trigger,
	.pointer = snd_drv_alsa_pointer,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
	.copy_user = snd_drv_alsa_playback_copy_user,
	.copy_kernel = snd_drv_alsa_playback_copy_kernel,
	.fill_silence = snd_drv_alsa_playback_fill_silence,
#else
	.copy = snd_drv_alsa_playback_copy_user,
	.silence = snd_drv_alsa_playback_fill_silence,
#endif
};

static struct snd_pcm_ops snd_drv_alsa_capture_ops = {
	.open = snd_drv_alsa_open,
	.close = snd_drv_alsa_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_drv_alsa_hw_params,
	.hw_free = snd_drv_alsa_hw_free,
	.prepare = snd_drv_alsa_prepare,
	.trigger = snd_drv_alsa_trigger,
	.pointer = snd_drv_alsa_pointer,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
	.copy_user = snd_drv_alsa_capture_copy_user,
	.copy_kernel = snd_drv_alsa_capture_copy_kernel,
#else
	.copy = snd_drv_alsa_capture_copy_user,
#endif
};

static int snd_drv_new_pcm(struct card_info *card_info,
	struct cfg_pcm_instance *instance_config,
	struct pcm_instance_info *pcm_instance_info)
{
	struct snd_pcm *pcm;
	int ret, i;

	dev_dbg(&card_info->drv_info->xb_dev->dev,
		"New PCM device \"%s\" with id %d playback %d capture %d",
		instance_config->name,
		instance_config->device_id,
		instance_config->num_streams_pb,
		instance_config->num_streams_cap);

	pcm_instance_info->card_info = card_info;

	snd_drv_copy_pcm_hw(&pcm_instance_info->pcm_hw,
		&instance_config->pcm_hw, &card_info->pcm_hw);

	if (instance_config->num_streams_pb) {
		pcm_instance_info->streams_pb = devm_kcalloc(
			&card_info->card->card_dev,
			instance_config->num_streams_pb,
			sizeof(struct pcm_stream_info),
			GFP_KERNEL);
		if (!pcm_instance_info->streams_pb)
			return -ENOMEM;
	}

	if (instance_config->num_streams_cap) {
		pcm_instance_info->streams_cap = devm_kcalloc(
			&card_info->card->card_dev,
			instance_config->num_streams_cap,
			sizeof(struct pcm_stream_info),
			GFP_KERNEL);
		if (!pcm_instance_info->streams_cap)
			return -ENOMEM;
	}

	pcm_instance_info->num_pcm_streams_pb =
			instance_config->num_streams_pb;
	pcm_instance_info->num_pcm_streams_cap =
			instance_config->num_streams_cap;

	for (i = 0; i < pcm_instance_info->num_pcm_streams_pb; i++) {
		pcm_instance_info->streams_pb[i].pcm_hw =
			instance_config->streams_pb[i].pcm_hw;
		pcm_instance_info->streams_pb[i].index =
			instance_config->streams_pb[i].index;
	}

	for (i = 0; i < pcm_instance_info->num_pcm_streams_cap; i++) {
		pcm_instance_info->streams_cap[i].pcm_hw =
			instance_config->streams_cap[i].pcm_hw;
		pcm_instance_info->streams_cap[i].index =
			instance_config->streams_cap[i].index;
	}

	ret = snd_pcm_new(card_info->card, instance_config->name,
			instance_config->device_id,
			instance_config->num_streams_pb,
			instance_config->num_streams_cap,
			&pcm);
	if (ret < 0)
		return ret;

	pcm->private_data = pcm_instance_info;
	pcm->info_flags = 0;
	strncpy(pcm->name, "Virtual card PCM", sizeof(pcm->name));

	if (instance_config->num_streams_pb)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
				&snd_drv_alsa_playback_ops);

	if (instance_config->num_streams_cap)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
				&snd_drv_alsa_capture_ops);

	pcm_instance_info->pcm = pcm;
	return 0;
}

static int snd_drv_probe(struct platform_device *pdev)
{
	struct card_info *card_info;
	struct card_plat_data *platdata;
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
		ret = snd_drv_new_pcm(card_info,
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

static void snd_drv_cleanup(struct drv_info *drv_info)
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

static int snd_drv_init(struct drv_info *drv_info)
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
	snd_drv_cleanup(drv_info);
	return -ENODEV;
}

static irqreturn_t evtchnl_interrupt_req(int irq, void *dev_id)
{
	struct evtchnl_info *channel = dev_id;
	struct drv_info *drv_info = channel->drv_info;
	struct xensnd_resp *resp;
	RING_IDX i, rp;
	unsigned long flags;

	printk("%s state %d\n", __FUNCTION__, channel->state);
	spin_lock_irqsave(&drv_info->io_lock, flags);
	if (unlikely(channel->state != EVTCHNL_STATE_CONNECTED))
		goto out;

again:
	rp = channel->u.req.ring.sring->rsp_prod;
	/* ensure we see queued responses up to rp */
	rmb();

	for (i = channel->u.req.ring.rsp_cons; i != rp; i++) {
		resp = RING_GET_RESPONSE(&channel->u.req.ring, i);
		if (resp->id != channel->evt_id)
			continue;
		switch (resp->operation) {
		case XENSND_OP_OPEN:
			/* fall through */
		case XENSND_OP_CLOSE:
			/* fall through */
		case XENSND_OP_READ:
			/* fall through */
		case XENSND_OP_WRITE:
			channel->u.req.resp_status = resp->status;
			complete(&channel->u.req.completion);
			break;

		case XENSND_OP_GET_HW_PARAMS:
			channel->u.req.resp_status = resp->status;
			channel->u.req.pcm_hw = resp->op.hw;
			complete(&channel->u.req.completion);
			break;

		default:
			dev_err(&drv_info->xb_dev->dev,
				"Operation %d is not supported\n",
				resp->operation);
			break;
		}
	}

	channel->u.req.ring.rsp_cons = i;
	if (i != channel->u.req.ring.req_prod_pvt) {
		int more_to_do;

		RING_FINAL_CHECK_FOR_RESPONSES(&channel->u.req.ring,
			more_to_do);
		if (more_to_do)
			goto again;
	} else
		channel->u.req.ring.sring->rsp_event = i + 1;

out:
	spin_unlock_irqrestore(&drv_info->io_lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t evtchnl_interrupt_evt(int irq, void *dev_id)
{
	struct evtchnl_info *channel = dev_id;
	struct drv_info *drv_info = channel->drv_info;
	struct xensnd_event_page *page = channel->u.evt.page;
	uint32_t cons, prod;
	unsigned long flags;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	if (unlikely(channel->state != EVTCHNL_STATE_CONNECTED))
		goto out;

	prod = page->in_prod;
	/* ensure we see ring contents up to prod */
	virt_rmb();
	if (prod == page->in_cons)
		goto out;

	for (cons = page->in_cons; cons != prod; cons++) {
		struct xensnd_evt *event;

		event = &XENSND_IN_RING_REF(page, cons);
		if (unlikely(event->id != channel->evt_id++))
			continue;

		switch (event->type) {
		case XENSND_EVT_CUR_POS:
			spin_unlock_irqrestore(&drv_info->io_lock, flags);
			snd_drv_handle_appl_ptr(channel,
				event->op.cur_pos.position);
			spin_lock_irqsave(&drv_info->io_lock, flags);
			break;
		}
	}
	page->in_cons = cons;
	/* ensure ring contents */
	virt_wmb();
out:
	spin_unlock_irqrestore(&drv_info->io_lock, flags);
	return IRQ_HANDLED;
}

static inline void evtchnl_flush(
		struct evtchnl_info *channel)
{
	int notify;

	channel->u.req.ring.req_prod_pvt++;
	RING_PUSH_REQUESTS_AND_CHECK_NOTIFY(&channel->u.req.ring, notify);
	if (notify)
		notify_remote_via_irq(channel->irq);
}

static void evtchnl_free(struct drv_info *drv_info,
		struct evtchnl_info *channel)
{
	unsigned long page = 0;

	printk("Freeing evtchnl %s port %d IRQ %d gref %d dev_id %p\n",
		channel->type == EVTCHNL_TYPE_REQ ? "REQ" : "EVT",
		channel->port, channel->irq, channel->gref, channel);
	if (channel->type == EVTCHNL_TYPE_REQ)
		page = (unsigned long)channel->u.req.ring.sring;
	else if (channel->type == EVTCHNL_TYPE_EVT)
		page = (unsigned long)channel->u.evt.page;

	if (!page)
		return;

	channel->state = EVTCHNL_STATE_DISCONNECTED;
	if (channel->type == EVTCHNL_TYPE_REQ) {
		/* release all who still waits for response if any */
		channel->u.req.resp_status = -EIO;
		printk("complete_all %p\n", &channel->u.req.completion);
		complete_all(&channel->u.req.completion);
	}

	if (channel->irq)
		unbind_from_irqhandler(channel->irq, channel);

	if (channel->port)
		xenbus_free_evtchn(drv_info->xb_dev, channel->port);

	/* End access and free the pages */
	if (channel->gref != GRANT_INVALID_REF)
		gnttab_end_foreign_access(channel->gref, 0, page);

	if (channel->type == EVTCHNL_TYPE_REQ)
		channel->u.req.ring.sring = NULL;
	else
		channel->u.evt.page = NULL;

	memset(channel, 0, sizeof(*channel));
}

static void evtchnl_free_all(struct drv_info *drv_info)
{
	int i;

	if (!drv_info->evt_pairs)
		return;

	for (i = 0; i < drv_info->num_evt_pairs; i++) {
		evtchnl_free(drv_info, &drv_info->evt_pairs[i].req);
		evtchnl_free(drv_info, &drv_info->evt_pairs[i].evt);
	}
	kfree(drv_info->evt_pairs);
	drv_info->evt_pairs = NULL;
}

static int evtchnl_alloc(struct drv_info *drv_info, int index,
	struct evtchnl_info *channel, enum evtchnl_type type)
{
	struct xenbus_device *xb_dev = drv_info->xb_dev;
	unsigned long page;
	grant_ref_t gref;
	irq_handler_t handler;
	int ret;

	printk("%s %d stream index %d type %s\n", __FUNCTION__, __LINE__, index, type == EVTCHNL_TYPE_REQ ? "REQ" : "EVT");
	memset(channel, 0, sizeof(*channel));
	channel->type = type;
	channel->index = index;
	channel->drv_info = drv_info;
	channel->state = EVTCHNL_STATE_DISCONNECTED;
	channel->gref = GRANT_INVALID_REF;
	page = get_zeroed_page(GFP_NOIO | __GFP_HIGH);
	if (!page) {
		ret = -ENOMEM;
		goto fail;
	}

	if (type == EVTCHNL_TYPE_REQ) {
		struct xen_sndif_sring *sring = (struct xen_sndif_sring *)page;

		printk("init completion %p\n", &channel->u.req.completion);
		init_completion(&channel->u.req.completion);
		SHARED_RING_INIT(sring);
		FRONT_RING_INIT(&channel->u.req.ring, sring, XEN_PAGE_SIZE);

		ret = xenbus_grant_ring(xb_dev, sring, 1, &gref);
		if (ret < 0)
			goto fail;

		handler = evtchnl_interrupt_req;
	} else {
		channel->u.evt.page = (struct xensnd_event_page *)page;
		ret = gnttab_grant_foreign_access(xb_dev->otherend_id,
			virt_to_gfn((void *)page), 0);
		if (ret < 0)
			goto fail;

		gref = ret;
		handler = evtchnl_interrupt_evt;
	}
	channel->gref = gref;

	ret = xenbus_alloc_evtchn(xb_dev, &channel->port);
	if (ret < 0)
		goto fail;

	ret = bind_evtchn_to_irqhandler(channel->port,
		handler, 0, xb_dev->devicetype, channel);
	if (ret < 0)
		goto fail;

	channel->irq = ret;

	printk("Allocated evtchnl %s port %d IRQ %d gref %d dev_id %p\n",
		channel->type == EVTCHNL_TYPE_REQ ? "REQ" : "EVT",
		channel->port, channel->irq, channel->gref, channel);
	return 0;

fail:
	dev_err(&xb_dev->dev, "Failed to allocate ring: %d\n", ret);
	return ret;
}

static int evtchnl_publish(struct xenbus_transaction xbt,
	struct evtchnl_info *channel,
	const char *path, const char *node_ring,
	const char *node_chnl)
{
	struct xenbus_device *xb_dev = channel->drv_info->xb_dev;
	int ret;

	printk("%s %d type %s\n", __FUNCTION__, __LINE__, channel->type == EVTCHNL_TYPE_REQ ? "REQ" : "EVT");

	/* write control channel ring reference */
	ret = xenbus_printf(xbt, path, node_ring, "%u", channel->gref);
	if (ret < 0) {
		dev_err(&xb_dev->dev, "Error writing ring-ref: %d\n", ret);
		return ret;
	}

	/* write event channel ring reference */
	ret = xenbus_printf(xbt, path, node_chnl, "%u", channel->port);
	if (ret < 0) {
		dev_err(&xb_dev->dev, "Error writing event channel: %d\n", ret);
		return ret;
	}

	return 0;
}

static int evtchnl_create_all(struct drv_info *drv_info, int num_streams)
{
	struct cfg_card *cfg_card;
	int d, ret = 0;

	drv_info->evt_pairs = kcalloc(num_streams,
		sizeof(struct evtchnl_pair_info), GFP_KERNEL);
	if (!drv_info->evt_pairs)
		return -ENOMEM;

	cfg_card = &drv_info->cfg_plat_data.cfg_card;
	printk("%s %d num_streams %d\n", __FUNCTION__, __LINE__, num_streams);
	/* iterate over devices and their streams and create event channels */
	for (d = 0; d < cfg_card->num_pcm_instances; d++) {
		struct cfg_pcm_instance *pcm_instance;
		int s, index;

		pcm_instance = &cfg_card->pcm_instances[d];
		printk("%s %d instance %d\n", __FUNCTION__, __LINE__, d);

		for (s = 0; s < pcm_instance->num_streams_pb; s++) {
			index = pcm_instance->streams_pb[s].index;

			ret = evtchnl_alloc(drv_info, index,
				&drv_info->evt_pairs[index].req,
				EVTCHNL_TYPE_REQ);
			if (ret < 0) {
				dev_err(&drv_info->xb_dev->dev,
					"Error allocating control channel\n");
				goto fail;
			}

			ret = evtchnl_alloc(drv_info, index,
				&drv_info->evt_pairs[index].evt,
				EVTCHNL_TYPE_EVT);
			if (ret < 0) {
				dev_err(&drv_info->xb_dev->dev,
					"Error allocating in-event channel\n");
				goto fail;
			}
		}

		for (s = 0; s < pcm_instance->num_streams_cap; s++) {
			index = pcm_instance->streams_cap[s].index;

			ret = evtchnl_alloc(drv_info, index,
				&drv_info->evt_pairs[index].req,
				EVTCHNL_TYPE_REQ);
			if (ret < 0) {
				dev_err(&drv_info->xb_dev->dev,
					"Error allocating control channel\n");
				goto fail;
			}

			ret = evtchnl_alloc(drv_info, index,
				&drv_info->evt_pairs[index].evt,
				EVTCHNL_TYPE_EVT);
			if (ret < 0) {
				dev_err(&drv_info->xb_dev->dev,
					"Error allocating in-event channel\n");
				goto fail;
			}
		}
	}
	if (ret < 0)
		goto fail;

	drv_info->num_evt_pairs = num_streams;
	return 0;

fail:
	evtchnl_free_all(drv_info);
	return ret;
}

static int evtchnl_publish_all(struct drv_info *drv_info)
{
	struct xenbus_transaction xbt;
	struct cfg_card *cfg_card;
	int ret, d;

	cfg_card = &drv_info->cfg_plat_data.cfg_card;

again:
	ret = xenbus_transaction_start(&xbt);
	if (ret < 0) {
		xenbus_dev_fatal(drv_info->xb_dev, ret, "starting transaction");
		return ret;
	}

	printk("%s %d drv_info->num_evt_pairs %d\n", __FUNCTION__, __LINE__, drv_info->num_evt_pairs);

	for (d = 0; d < cfg_card->num_pcm_instances; d++) {
		struct cfg_pcm_instance *pcm_instance;
		int s, index;

		pcm_instance = &cfg_card->pcm_instances[d];

		printk("%s %d pcm_instanse %d\n", __FUNCTION__, __LINE__, d);

		for (s = 0; s < pcm_instance->num_streams_pb; s++) {
			index = pcm_instance->streams_pb[s].index;

			printk("%s %d index %d\n", __FUNCTION__, __LINE__, index);
			ret = evtchnl_publish(xbt,
				&drv_info->evt_pairs[index].req,
				pcm_instance->streams_pb[s].xenstore_path,
				XENSND_FIELD_REQ_RING_REF,
				XENSND_FIELD_REQ_EVT_CHNL);
			if (ret < 0)
				goto fail;

			ret = evtchnl_publish(xbt,
				&drv_info->evt_pairs[index].evt,
				pcm_instance->streams_pb[s].xenstore_path,
				XENSND_FIELD_EVT_RING_REF,
				XENSND_FIELD_EVT_EVT_CHNL);
			if (ret < 0)
				goto fail;
		}

		for (s = 0; s < pcm_instance->num_streams_cap; s++) {
			index = pcm_instance->streams_cap[s].index;

			printk("%s %d index %d\n", __FUNCTION__, __LINE__, index);
			ret = evtchnl_publish(xbt,
				&drv_info->evt_pairs[index].req,
				pcm_instance->streams_cap[s].xenstore_path,
				XENSND_FIELD_REQ_RING_REF,
				XENSND_FIELD_REQ_EVT_CHNL);
			if (ret < 0)
				goto fail;

			ret = evtchnl_publish(xbt,
				&drv_info->evt_pairs[index].evt,
				pcm_instance->streams_cap[s].xenstore_path,
				XENSND_FIELD_EVT_RING_REF,
				XENSND_FIELD_EVT_EVT_CHNL);
			if (ret < 0)
				goto fail;
		}
	}
	ret = xenbus_transaction_end(xbt, 0);
	if (ret < 0) {
		if (ret == -EAGAIN)
			goto again;
		xenbus_dev_fatal(drv_info->xb_dev, ret,
			"completing transaction");
		goto fail_to_end;
	}
	return 0;
fail:
	xenbus_transaction_end(xbt, 1);
fail_to_end:
	xenbus_dev_fatal(drv_info->xb_dev, ret, "writing XenStore");
	return ret;
}

struct CFG_HW_SAMPLE_RATE {
	const char *name;
	unsigned int mask;
	unsigned int value;
};

static struct CFG_HW_SAMPLE_RATE cfg_hw_supported_rates[] = {
	{ .name = "5512",   .mask = SNDRV_PCM_RATE_5512,   .value = 5512 },
	{ .name = "8000",   .mask = SNDRV_PCM_RATE_8000,   .value = 8000 },
	{ .name = "11025",  .mask = SNDRV_PCM_RATE_11025,  .value = 11025 },
	{ .name = "16000",  .mask = SNDRV_PCM_RATE_16000,  .value = 16000 },
	{ .name = "22050",  .mask = SNDRV_PCM_RATE_22050,  .value = 22050 },
	{ .name = "32000",  .mask = SNDRV_PCM_RATE_32000,  .value = 32000 },
	{ .name = "44100",  .mask = SNDRV_PCM_RATE_44100,  .value = 44100 },
	{ .name = "48000",  .mask = SNDRV_PCM_RATE_48000,  .value = 48000 },
	{ .name = "64000",  .mask = SNDRV_PCM_RATE_64000,  .value = 64000 },
	{ .name = "96000",  .mask = SNDRV_PCM_RATE_96000,  .value = 96000 },
	{ .name = "176400", .mask = SNDRV_PCM_RATE_176400, .value = 176400 },
	{ .name = "192000", .mask = SNDRV_PCM_RATE_192000, .value = 192000 },
};

struct CFG_HW_SAMPLE_FORMAT {
	const char *name;
	u64 mask;
};

static struct CFG_HW_SAMPLE_FORMAT cfg_hw_supported_formats[] = {
	{
		.name = XENSND_PCM_FORMAT_U8_STR,
		.mask = SNDRV_PCM_FMTBIT_U8
	},
	{
		.name = XENSND_PCM_FORMAT_S8_STR,
		.mask = SNDRV_PCM_FMTBIT_S8
	},
	{
		.name = XENSND_PCM_FORMAT_U16_LE_STR,
		.mask = SNDRV_PCM_FMTBIT_U16_LE
	},
	{
		.name = XENSND_PCM_FORMAT_U16_BE_STR,
		.mask = SNDRV_PCM_FMTBIT_U16_BE
	},
	{
		.name = XENSND_PCM_FORMAT_S16_LE_STR,
		.mask = SNDRV_PCM_FMTBIT_S16_LE
	},
	{
		.name = XENSND_PCM_FORMAT_S16_BE_STR,
		.mask = SNDRV_PCM_FMTBIT_S16_BE
	},
	{
		.name = XENSND_PCM_FORMAT_U24_LE_STR,
		.mask = SNDRV_PCM_FMTBIT_U24_LE
	},
	{
		.name = XENSND_PCM_FORMAT_U24_BE_STR,
		.mask = SNDRV_PCM_FMTBIT_U24_BE
	},
	{
		.name = XENSND_PCM_FORMAT_S24_LE_STR,
		.mask = SNDRV_PCM_FMTBIT_S24_LE
	},
	{
		.name = XENSND_PCM_FORMAT_S24_BE_STR,
		.mask = SNDRV_PCM_FMTBIT_S24_BE
	},
	{
		.name = XENSND_PCM_FORMAT_U32_LE_STR,
		.mask = SNDRV_PCM_FMTBIT_U32_LE
	},
	{
		.name = XENSND_PCM_FORMAT_U32_BE_STR,
		.mask = SNDRV_PCM_FMTBIT_U32_BE
	},
	{
		.name = XENSND_PCM_FORMAT_S32_LE_STR,
		.mask = SNDRV_PCM_FMTBIT_S32_LE
	},
	{
		.name = XENSND_PCM_FORMAT_S32_BE_STR,
		.mask = SNDRV_PCM_FMTBIT_S32_BE
	},
	{
		.name = XENSND_PCM_FORMAT_A_LAW_STR,
		.mask = SNDRV_PCM_FMTBIT_A_LAW
	},
	{
		.name = XENSND_PCM_FORMAT_MU_LAW_STR,
		.mask = SNDRV_PCM_FMTBIT_MU_LAW
	},
	{
		.name = XENSND_PCM_FORMAT_F32_LE_STR,
		.mask = SNDRV_PCM_FMTBIT_FLOAT_LE
	},
	{
		.name = XENSND_PCM_FORMAT_F32_BE_STR,
		.mask = SNDRV_PCM_FMTBIT_FLOAT_BE
	},
	{
		.name = XENSND_PCM_FORMAT_F64_LE_STR,
		.mask = SNDRV_PCM_FMTBIT_FLOAT64_LE
	},
	{
		.name = XENSND_PCM_FORMAT_F64_BE_STR,
		.mask = SNDRV_PCM_FMTBIT_FLOAT64_BE
	},
	{
		.name = XENSND_PCM_FORMAT_IEC958_SUBFRAME_LE_STR,
		.mask = SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE
	},
	{
		.name = XENSND_PCM_FORMAT_IEC958_SUBFRAME_BE_STR,
		.mask = SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_BE
	},
	{
		.name = XENSND_PCM_FORMAT_IMA_ADPCM_STR,
		.mask = SNDRV_PCM_FMTBIT_IMA_ADPCM
	},
	{
		.name = XENSND_PCM_FORMAT_MPEG_STR,
		.mask = SNDRV_PCM_FMTBIT_MPEG
	},
	{
		.name = XENSND_PCM_FORMAT_GSM_STR,
		.mask = SNDRV_PCM_FMTBIT_GSM
	},
};

static void cfg_hw_rates(char *list, unsigned int len,
	const char *path, struct snd_pcm_hardware *pcm_hw)
{
	char *cur_rate;
	unsigned int cur_mask;
	unsigned int cur_value;
	unsigned int rates;
	unsigned int rate_min;
	unsigned int rate_max;
	int i;

	rates = 0;
	rate_min = -1;
	rate_max = 0;
	while ((cur_rate = strsep(&list, XENSND_LIST_SEPARATOR))) {
		for (i = 0; i < ARRAY_SIZE(cfg_hw_supported_rates); i++)
			if (!strncasecmp(cur_rate,
					cfg_hw_supported_rates[i].name,
					XENSND_SAMPLE_RATE_MAX_LEN)) {
				cur_mask = cfg_hw_supported_rates[i].mask;
				cur_value = cfg_hw_supported_rates[i].value;
				rates |= cur_mask;
				if (rate_min > cur_value)
					rate_min = cur_value;
				if (rate_max < cur_value)
					rate_max = cur_value;
			}
	}

	if (rates) {
		pcm_hw->rates = rates;
		pcm_hw->rate_min = rate_min;
		pcm_hw->rate_max = rate_max;
	}
}

static void cfg_formats(char *list, unsigned int len,
	const char *path, struct snd_pcm_hardware *pcm_hw)
{
	u64 formats;
	char *cur_format;
	int i;

	formats = 0;
	while ((cur_format = strsep(&list, XENSND_LIST_SEPARATOR))) {
		for (i = 0; i < ARRAY_SIZE(cfg_hw_supported_formats); i++)
			if (!strncasecmp(cur_format,
					cfg_hw_supported_formats[i].name,
					XENSND_SAMPLE_FORMAT_MAX_LEN))
				formats |= cfg_hw_supported_formats[i].mask;
	}

	if (formats)
		pcm_hw->formats = formats;
}

static void cfg_pcm_hw(const char *path,
	struct snd_pcm_hardware *parent_pcm_hw,
	struct snd_pcm_hardware *pcm_hw)
{
	char *list;
	int val;
	size_t buf_sz;
	unsigned int len;

	/* inherit parent's PCM HW and read overrides if any */
	*pcm_hw = *parent_pcm_hw;

	val = xenbus_read_unsigned(path, XENSND_FIELD_CHANNELS_MIN, 0);
	if (val)
		pcm_hw->channels_min = val;

	val = xenbus_read_unsigned(path, XENSND_FIELD_CHANNELS_MAX, 0);
	if (val)
		pcm_hw->channels_max = val;

	list = xenbus_read(XBT_NIL, path, XENSND_FIELD_SAMPLE_RATES, &len);
	if (!IS_ERR(list)) {
		cfg_hw_rates(list, len, path, pcm_hw);
		kfree(list);
	}

	list = xenbus_read(XBT_NIL, path, XENSND_FIELD_SAMPLE_FORMATS, &len);
	if (!IS_ERR(list)) {
		cfg_formats(list, len, path, pcm_hw);
		kfree(list);
	}

	buf_sz = xenbus_read_unsigned(path, XENSND_FIELD_BUFFER_SIZE, 0);
	if (buf_sz)
		pcm_hw->buffer_bytes_max = buf_sz;
}

static int cfg_get_stream_type(const char *path, int index,
	int *num_pb, int *num_cap)
{
	char *str = NULL;
	char *stream_path;
	int ret;

	*num_pb = 0;
	*num_cap = 0;
	stream_path = kasprintf(GFP_KERNEL, "%s/%d", path, index);
	if (!stream_path) {
		ret = -ENOMEM;
		goto fail;
	}

	str = xenbus_read(XBT_NIL, stream_path, XENSND_FIELD_TYPE, NULL);
	if (IS_ERR(str)) {
		ret = -EINVAL;
		goto fail;
	}

	if (!strncasecmp(str, XENSND_STREAM_TYPE_PLAYBACK,
		sizeof(XENSND_STREAM_TYPE_PLAYBACK)))
		(*num_pb)++;
	else if (!strncasecmp(str, XENSND_STREAM_TYPE_CAPTURE,
		sizeof(XENSND_STREAM_TYPE_CAPTURE)))
		(*num_cap)++;
	else {
		ret = -EINVAL;
		goto fail;
	}
	ret = 0;

fail:
	kfree(stream_path);
	kfree(str);
	return ret;
}

static int cfg_stream(struct drv_info *drv_info,
	struct cfg_pcm_instance *pcm_instance,
	const char *path, int index, int *cur_pb, int *cur_cap,
	int *stream_cnt)
{
	char *str = NULL;
	char *stream_path;
	struct cfg_stream *stream;
	int ret;

	stream_path = devm_kasprintf(&drv_info->xb_dev->dev,
		GFP_KERNEL, "%s/%d", path, index);
	if (!stream_path) {
		ret = -ENOMEM;
		goto fail;
	}

	str = xenbus_read(XBT_NIL, stream_path, XENSND_FIELD_TYPE, NULL);
	if (IS_ERR(str)) {
		ret = -EINVAL;
		goto fail;
	}

	if (!strncasecmp(str, XENSND_STREAM_TYPE_PLAYBACK,
		sizeof(XENSND_STREAM_TYPE_PLAYBACK))) {
		stream = &pcm_instance->streams_pb[(*cur_pb)++];
	} else if (!strncasecmp(str, XENSND_STREAM_TYPE_CAPTURE,
		sizeof(XENSND_STREAM_TYPE_CAPTURE))) {
		stream = &pcm_instance->streams_cap[(*cur_cap)++];
	} else {
		ret = -EINVAL;
		goto fail;
	}

	/* get next stream index */
	stream->index = (*stream_cnt)++;
	stream->xenstore_path = stream_path;
	/*
	 * check in Xen store if PCM HW configuration exists for this stream
	 * and update if so, e.g. we inherit all values from device's PCM HW,
	 * but can still override some of the values for the stream
	 */
	cfg_pcm_hw(stream->xenstore_path,
		&pcm_instance->pcm_hw, &stream->pcm_hw);
	ret = 0;

fail:
	kfree(str);
	return ret;
}

static int cfg_device(struct drv_info *drv_info,
	struct cfg_pcm_instance *pcm_instance,
	struct snd_pcm_hardware *parent_pcm_hw,
	const char *path, int node_index, int *stream_cnt)
{
	char *str;
	char *device_path;
	int ret, i, num_streams;
	int num_pb, num_cap;
	int cur_pb, cur_cap;
	char node[3];

	device_path = kasprintf(GFP_KERNEL, "%s/%d", path, node_index);
	if (!device_path)
		return -ENOMEM;

	str = xenbus_read(XBT_NIL, device_path, XENSND_FIELD_DEVICE_NAME, NULL);
	if (!IS_ERR(str)) {
		strncpy(pcm_instance->name, str, sizeof(pcm_instance->name));
		kfree(str);
	}

	pcm_instance->device_id = node_index;

	/*
	 * check in Xen store if PCM HW configuration exists for this device
	 * and update if so, e.g. we inherit all values from card's PCM HW,
	 * but can still override some of the values for the device
	 */
	cfg_pcm_hw(device_path, parent_pcm_hw, &pcm_instance->pcm_hw);

	/*
	 * find out how many streams were configured in Xen store:
	 * streams must have sequential unique IDs, so stop when one
	 * does not exist
	 */
	num_streams = 0;
	do {
		snprintf(node, sizeof(node), "%d", num_streams);
		if (!xenbus_exists(XBT_NIL, device_path, node))
			break;

		num_streams++;
	} while (num_streams < VSND_MAX_STREAM);

	pcm_instance->num_streams_pb = 0;
	pcm_instance->num_streams_cap = 0;
	/* get number of playback and capture streams */
	for (i = 0; i < num_streams; i++) {
		ret = cfg_get_stream_type(device_path, i, &num_pb, &num_cap);
		if (ret < 0)
			goto fail;

		pcm_instance->num_streams_pb += num_pb;
		pcm_instance->num_streams_cap += num_cap;
	}

	if (pcm_instance->num_streams_pb) {
		pcm_instance->streams_pb = devm_kcalloc(
			&drv_info->xb_dev->dev,
			pcm_instance->num_streams_pb,
			sizeof(struct cfg_stream), GFP_KERNEL);
		if (!pcm_instance->streams_pb) {
			ret = -ENOMEM;
			goto fail;
		}
	}

	if (pcm_instance->num_streams_cap) {
		pcm_instance->streams_cap = devm_kcalloc(
			&drv_info->xb_dev->dev,
			pcm_instance->num_streams_cap,
			sizeof(struct cfg_stream), GFP_KERNEL);
		if (!pcm_instance->streams_cap) {
			ret = -ENOMEM;
			goto fail;
		}
	}

	cur_pb = 0;
	cur_cap = 0;
	for (i = 0; i < num_streams; i++) {
		ret = cfg_stream(drv_info,
			pcm_instance, device_path, i, &cur_pb, &cur_cap,
			stream_cnt);
		if (ret < 0)
			goto fail;
	}
	ret = 0;

fail:
	kfree(device_path);
	return ret;
}

static int cfg_card(struct drv_info *drv_info,
	struct card_plat_data *plat_data, int *stream_cnt)
{
	struct xenbus_device *xb_dev = drv_info->xb_dev;
	int ret, num_devices, i;
	char node[3];

	num_devices = 0;
	do {
		snprintf(node, sizeof(node), "%d", num_devices);
		if (!xenbus_exists(XBT_NIL, xb_dev->nodename, node))
			break;

		num_devices++;
	} while (num_devices < SNDRV_PCM_DEVICES);

	if (!num_devices) {
		dev_warn(&xb_dev->dev,
			"No devices configured for sound card at %s\n",
			xb_dev->nodename);
		return -ENODEV;
	}

	/* start from default PCM HW configuration for the card */
	cfg_pcm_hw(xb_dev->nodename, &snd_drv_pcm_hw_default,
		&plat_data->cfg_card.pcm_hw);

	plat_data->cfg_card.pcm_instances = devm_kcalloc(
		&drv_info->xb_dev->dev, num_devices,
		sizeof(struct cfg_pcm_instance), GFP_KERNEL);
	if (!plat_data->cfg_card.pcm_instances)
		return -ENOMEM;

	for (i = 0; i < num_devices; i++) {
		ret = cfg_device(drv_info,
			&plat_data->cfg_card.pcm_instances[i],
			&plat_data->cfg_card.pcm_hw,
			xb_dev->nodename, i, stream_cnt);
		if (ret < 0)
			return ret;
	}
	plat_data->cfg_card.num_pcm_instances = num_devices;
	return 0;
}

static void xenbus_drv_remove_internal(struct drv_info *drv_info)
{
	snd_drv_cleanup(drv_info);
	evtchnl_free_all(drv_info);
}

static inline grant_ref_t sh_buf_get_dir_start(struct sh_buf_info *buf)
{
	if (!buf->grefs)
		return GRANT_INVALID_REF;
	return buf->grefs[0];
}

static inline void sh_buf_clear(struct sh_buf_info *buf)
{
	memset(buf, 0, sizeof(*buf));
}

static void sh_buf_free(struct sh_buf_info *buf)
{
	int i;

	if (buf->grefs) {
		for (i = 0; i < buf->num_grefs; i++)
			if (buf->grefs[i] != GRANT_INVALID_REF)
				gnttab_end_foreign_access(buf->grefs[i],
						0, 0UL);
		kfree(buf->grefs);
	}
	kfree(buf->vdirectory);
	free_pages_exact(buf->vbuffer, buf->vbuffer_sz);
	sh_buf_clear(buf);
}

/*
 * number of grant references a page can hold with respect to the
 * xendispl_page_directory header
 */
#define XENSND_NUM_GREFS_PER_PAGE ((XEN_PAGE_SIZE - \
	offsetof(struct xensnd_page_directory, gref)) / \
	sizeof(grant_ref_t))

static void sh_buf_fill_page_dir(struct sh_buf_info *buf, int num_pages_dir)
{
	struct xensnd_page_directory *page_dir;
	unsigned char *ptr;
	int i, cur_gref, grefs_left, to_copy;

	ptr = buf->vdirectory;
	grefs_left = buf->num_grefs - num_pages_dir;
	/*
	 * skip grant references at the beginning, they are for pages granted
	 * for the page directory itself
	 */
	cur_gref = num_pages_dir;
	for (i = 0; i < num_pages_dir; i++) {
		page_dir = (struct xensnd_page_directory *)ptr;
		if (grefs_left <= XENSND_NUM_GREFS_PER_PAGE) {
			to_copy = grefs_left;
			page_dir->gref_dir_next_page = GRANT_INVALID_REF;
		} else {
			to_copy = XENSND_NUM_GREFS_PER_PAGE;
			page_dir->gref_dir_next_page = buf->grefs[i + 1];
		}
		memcpy(&page_dir->gref, &buf->grefs[cur_gref],
			to_copy * sizeof(grant_ref_t));
		ptr += XEN_PAGE_SIZE;
		grefs_left -= to_copy;
		cur_gref += to_copy;
	}
}

static int sh_buf_grant_refs(struct xenbus_device *xb_dev,
	struct sh_buf_info *buf,
	int num_pages_dir, int num_pages_vbuffer, int num_grefs)
{
	grant_ref_t priv_gref_head;
	int ret, i, j, cur_ref;
	int otherend_id;

	ret = gnttab_alloc_grant_references(num_grefs, &priv_gref_head);
	if (ret)
		return ret;

	buf->num_grefs = num_grefs;
	otherend_id = xb_dev->otherend_id;
	j = 0;

	for (i = 0; i < num_pages_dir; i++) {
		cur_ref = gnttab_claim_grant_reference(&priv_gref_head);
		if (cur_ref < 0) {
			ret = cur_ref;
			goto fail;
		}

		gnttab_grant_foreign_access_ref(cur_ref, otherend_id,
			xen_page_to_gfn(virt_to_page(buf->vdirectory +
				XEN_PAGE_SIZE * i)), 0);
		buf->grefs[j++] = cur_ref;
	}

	for (i = 0; i < num_pages_vbuffer; i++) {
		cur_ref = gnttab_claim_grant_reference(&priv_gref_head);
		if (cur_ref < 0) {
			ret = cur_ref;
			goto fail;
		}

		gnttab_grant_foreign_access_ref(cur_ref, otherend_id,
			xen_page_to_gfn(virt_to_page(buf->vbuffer +
				XEN_PAGE_SIZE * i)), 0);
		buf->grefs[j++] = cur_ref;
	}

	gnttab_free_grant_references(priv_gref_head);
	sh_buf_fill_page_dir(buf, num_pages_dir);
	return 0;

fail:
	gnttab_free_grant_references(priv_gref_head);
	return ret;
}

static int sh_buf_alloc_int_buffers(struct sh_buf_info *buf,
		int num_pages_dir, int num_pages_vbuffer, int num_grefs)
{
	buf->grefs = kcalloc(num_grefs, sizeof(*buf->grefs), GFP_KERNEL);
	if (!buf->grefs)
		return -ENOMEM;

	buf->vdirectory = kcalloc(num_pages_dir, XEN_PAGE_SIZE, GFP_KERNEL);
	if (!buf->vdirectory)
		goto fail;

	buf->vbuffer_sz = num_pages_vbuffer * XEN_PAGE_SIZE;
	buf->vbuffer = alloc_pages_exact(buf->vbuffer_sz, GFP_KERNEL);
	if (!buf->vbuffer)
		goto fail;
	return 0;

fail:
	kfree(buf->grefs);
	buf->grefs = NULL;
	kfree(buf->vdirectory);
	buf->vdirectory = NULL;
	return -ENOMEM;
}

static int sh_buf_alloc(struct xenbus_device *xb_dev,
	struct sh_buf_info *buf, unsigned int buffer_size)
{
	int num_pages_vbuffer, num_pages_dir, num_grefs;
	int ret;

	sh_buf_clear(buf);

	num_pages_vbuffer = DIV_ROUND_UP(buffer_size, XEN_PAGE_SIZE);
	/* number of pages the page directory consumes itself */
	num_pages_dir = DIV_ROUND_UP(num_pages_vbuffer,
		XENSND_NUM_GREFS_PER_PAGE);
	num_grefs = num_pages_vbuffer + num_pages_dir;

	ret = sh_buf_alloc_int_buffers(buf, num_pages_dir,
		num_pages_vbuffer, num_grefs);
	if (ret < 0)
		return ret;

	ret = sh_buf_grant_refs(xb_dev, buf,
		num_pages_dir, num_pages_vbuffer, num_grefs);
	if (ret < 0)
		return ret;

	sh_buf_fill_page_dir(buf, num_pages_dir);
	return 0;
}

static int be_on_initwait(struct drv_info *drv_info)
{
	int num_streams;
	int ret;

	drv_info->cfg_plat_data.drv_info = drv_info;
	num_streams = 0;
	ret = cfg_card(drv_info, &drv_info->cfg_plat_data, &num_streams);
	if (ret < 0)
		return ret;
	/* create event channels for all streams and publish */
	ret = evtchnl_create_all(drv_info, num_streams);
	if (ret < 0)
		return ret;
	return evtchnl_publish_all(drv_info);
}

static inline int be_on_connected(struct drv_info *drv_info)
{
	return snd_drv_init(drv_info);
}

static inline void be_on_disconnected(struct drv_info *drv_info)
{
	xenbus_drv_remove_internal(drv_info);
}

static void be_on_changed(struct xenbus_device *xb_dev,
	enum xenbus_state backend_state)
{
	struct drv_info *drv_info = dev_get_drvdata(&xb_dev->dev);
	int ret;

	printk("Backend state is %s, front is %s\n",
		xenbus_strstate(backend_state), xenbus_strstate(xb_dev->state));
	switch (backend_state) {
	case XenbusStateReconfiguring:
		/* fall through */
	case XenbusStateReconfigured:
		/* fall through */
	case XenbusStateInitialised:
		/* fall through */
		break;

	case XenbusStateInitialising:
		if (xb_dev->state == XenbusStateInitialising)
			break;

		/* recovering after backend unexpected closure */
		mutex_lock(&drv_info->mutex);
		be_on_disconnected(drv_info);
		mutex_unlock(&drv_info->mutex);
		xenbus_switch_state(xb_dev, XenbusStateInitialising);
		break;

	case XenbusStateInitWait:
		if (xb_dev->state != XenbusStateInitialising)
			break;

		mutex_lock(&drv_info->mutex);
		ret = be_on_initwait(drv_info);
		mutex_unlock(&drv_info->mutex);
		if (ret < 0) {
			xenbus_dev_fatal(xb_dev, ret,
				"initializing " XENSND_DRIVER_NAME);
			break;
		}

		xenbus_switch_state(xb_dev, XenbusStateInitialised);
		break;

	case XenbusStateConnected:
		if (xb_dev->state != XenbusStateInitialised)
			break;

		mutex_lock(&drv_info->mutex);
		ret = be_on_connected(drv_info);
		mutex_unlock(&drv_info->mutex);
		if (ret < 0) {
			xenbus_dev_fatal(xb_dev, ret,
				"connecting " XENSND_DRIVER_NAME);
			break;
		}

		xenbus_switch_state(xb_dev, XenbusStateConnected);
		break;

	case XenbusStateClosing:
		/*
		 * in this state backend starts freeing resources,
		 * so let it go into closed state first, so we can also
		 * remove ours
		 */
		break;

	case XenbusStateUnknown:
		/* fall through */
	case XenbusStateClosed:
		if (xb_dev->state == XenbusStateClosed)
			break;

		mutex_lock(&drv_info->mutex);
		be_on_disconnected(drv_info);
		mutex_unlock(&drv_info->mutex);
		xenbus_switch_state(xb_dev, XenbusStateInitialising);
		break;
	}
}

static int xenbus_drv_probe(struct xenbus_device *xb_dev,
	const struct xenbus_device_id *id)
{
	struct drv_info *drv_info;

	drv_info = devm_kzalloc(&xb_dev->dev, sizeof(*drv_info), GFP_KERNEL);
	if (!drv_info) {
		xenbus_dev_fatal(xb_dev, -ENOMEM, "allocating device memory");
		return -ENOMEM;
	}

	drv_info->xb_dev = xb_dev;
	spin_lock_init(&drv_info->io_lock);
	mutex_init(&drv_info->mutex);
	dev_set_drvdata(&xb_dev->dev, drv_info);
	xenbus_switch_state(xb_dev, XenbusStateInitialising);
	return 0;
}

static int xenbus_drv_remove(struct xenbus_device *dev)
{
	struct drv_info *drv_info = dev_get_drvdata(&dev->dev);

	xenbus_switch_state(dev, XenbusStateClosed);
	mutex_lock(&drv_info->mutex);
	xenbus_drv_remove_internal(drv_info);
	mutex_unlock(&drv_info->mutex);
	return 0;
}

static const struct xenbus_device_id xenbus_drv_ids[] = {
	{ XENSND_DRIVER_NAME },
	{ "" }
};

static struct xenbus_driver xenbus_driver = {
	.ids = xenbus_drv_ids,
	.probe = xenbus_drv_probe,
	.remove = xenbus_drv_remove,
	.otherend_changed = be_on_changed,
};

static int __init xenbus_drv_init(void)
{
	if (!xen_domain())
		return -ENODEV;

	pr_info("Initialising Xen " XENSND_DRIVER_NAME " frontend driver\n");
	return xenbus_register_frontend(&xenbus_driver);
}

static void __exit xenbus_drv_cleanup(void)
{
	pr_info("Unregistering Xen " XENSND_DRIVER_NAME " frontend driver\n");
	xenbus_unregister_driver(&xenbus_driver);
}

module_init(xenbus_drv_init);
module_exit(xenbus_drv_cleanup);

MODULE_DESCRIPTION("Xen virtual sound device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:"XENSND_DRIVER_NAME);
MODULE_SUPPORTED_DEVICE("{{ALSA,Virtual soundcard}}");
