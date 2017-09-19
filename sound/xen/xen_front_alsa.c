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

#include <linux/atomic.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>

#include <xen/xen.h>
#include <xen/xenbus.h>

#include <xen/interface/io/sndif.h>

#include "xen_front.h"
#include "xen_front_alsa.h"
#include "xen_front_cfg.h"

struct pcm_stream_info {
	int dummy;
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

static int new_pcm_instance(struct card_info *card_info,
	struct xen_front_cfg_pcm_instance *instance_cfg,
	struct pcm_instance_info *pcm_instance_info)
{
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
