/*
 * Remote processors emulator driver
 *
 * Copyright (C) 2016 EPAM Systems Inc.
 * Author: Oleksandr Tyshchenko <oleksandr_tyshchenko@epam.com>
 * based on drivers/iommu/arm-smmu.c

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG 1
#define pr_fmt(fmt) "coproc-emu: %s: " fmt, __func__

#include <linux/device.h>
#include <linux/file.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define COPROC_POWER_REG	0x10
#define CORPOC_ENABLE		(1 << 0)

struct mmio {
	void __iomem *base;
	unsigned long size;
};

struct coproc_emu_device {
	struct device *dev;

	u32 num_mmios;
	struct mmio *mmios;
	u32 num_irqs;
	unsigned int *irqs;

	struct list_head list;
};

static DEFINE_SPINLOCK(coproc_emu_devices_lock);
static LIST_HEAD(coproc_emu_devices);

static irqreturn_t coproc_emu_irq_handler(int irq, void *dev)
{
	struct coproc_emu_device *coproc_emu = dev;

	dev_err(coproc_emu->dev, "got irq %d\n", irq);

	return IRQ_HANDLED;
}

static void coproc_emu_device_power_on(struct coproc_emu_device *coproc_emu)
{
	void __iomem *base = coproc_emu->mmios[0].base;
	u32 val;

	val = readl_relaxed(base + COPROC_POWER_REG);
	val |= CORPOC_ENABLE;
	writel_relaxed(val, base + COPROC_POWER_REG);
}

static void coproc_emu_device_power_off(struct coproc_emu_device *coproc_emu)
{
	void __iomem *base = coproc_emu->mmios[0].base;
	u32 val;

	val = readl_relaxed(base + COPROC_POWER_REG);
	val &= CORPOC_ENABLE;
	writel_relaxed(val, base + COPROC_POWER_REG);
}

static int coproc_emu_device_dt_probe(struct platform_device *pdev)
{
	struct coproc_emu_device *coproc_emu;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int num_irqs, num_mmios, i, ret;

	pr_debug("entered\n");

	coproc_emu = devm_kzalloc(dev, sizeof(*coproc_emu), GFP_KERNEL);
	if (!coproc_emu) {
		dev_err(dev, "failed to allocate coproc_emu_device\n");
		return -ENOMEM;
	}
	coproc_emu->dev = dev;

	num_mmios = 0;
	while ((res = platform_get_resource(pdev, IORESOURCE_MEM, num_mmios)))
		num_mmios++;

	if (!num_mmios) {
		dev_err(dev, "failed to find at least one mmio\n");
		return -ENODEV;
	}

	dev_notice(dev, "found %d mmios\n", num_mmios);

	coproc_emu->mmios = devm_kzalloc(dev, sizeof(*coproc_emu->mmios) * num_mmios,
			GFP_KERNEL);
	if (!coproc_emu->mmios) {
		dev_err(dev, "failed to allocate %d mmios\n", num_mmios);
		return -ENOMEM;
	}

	for (i = 0; i < num_mmios; ++i) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		coproc_emu->mmios[i].base = devm_ioremap_resource(dev, res);
		if (IS_ERR(coproc_emu->mmios[i].base))
			return PTR_ERR(coproc_emu->mmios[i].base);

		coproc_emu->mmios[i].size = resource_size(res);
	}
	coproc_emu->num_mmios = num_mmios;

	dev_notice(dev, "processed %d mmios\n", num_mmios);

	num_irqs = 0;
	while ((res = platform_get_resource(pdev, IORESOURCE_IRQ, num_irqs)))
		num_irqs++;

	if (!num_irqs) {
		dev_err(dev, "failed to find at least one irq\n");
		return -ENODEV;
	}

	dev_notice(dev, "found %d irqs\n", num_irqs);

	coproc_emu->irqs = devm_kzalloc(dev, sizeof(*coproc_emu->irqs) * num_irqs,
			GFP_KERNEL);
	if (!coproc_emu->irqs) {
		dev_err(dev, "failed to allocate %d irqs\n", num_irqs);
		return -ENOMEM;
	}

	for (i = 0; i < num_irqs; ++i) {
		int irq = platform_get_irq(pdev, i);
		if (irq < 0) {
			dev_err(dev, "failed to get irq index %d\n", i);
			return -ENODEV;
		}

		coproc_emu->irqs[i] = irq;
	}
	coproc_emu->num_irqs = num_irqs;

	for (i = 0; i < num_irqs; ++i) {
		ret = request_irq(coproc_emu->irqs[i],
				  coproc_emu_irq_handler,
				  IRQF_SHARED,
				  "coproc_emu irq",
				  coproc_emu);
		if (ret) {
			dev_err(dev, "failed to request irq %d (%u)\n", i, coproc_emu->irqs[i]);
			goto out_free_irqs;
		}
	}

	dev_notice(dev, "processed %d irqs\n", num_irqs);

	INIT_LIST_HEAD(&coproc_emu->list);
	spin_lock(&coproc_emu_devices_lock);
	list_add(&coproc_emu->list, &coproc_emu_devices);
	spin_unlock(&coproc_emu_devices_lock);

	coproc_emu_device_power_on(coproc_emu);

	dev_notice(dev, "registered\n");

	return 0;

out_free_irqs:
	while (i--)
		free_irq(coproc_emu->irqs[i], coproc_emu);

	return ret;
}

static int coproc_emu_device_remove(struct platform_device *pdev)
{
	struct coproc_emu_device *curr, *coproc_emu = NULL;
	struct device *dev = &pdev->dev;
	int i;

	pr_debug("entered\n");

	spin_lock(&coproc_emu_devices_lock);
	list_for_each_entry(curr, &coproc_emu_devices, list) {
		if (curr->dev == dev) {
			coproc_emu = curr;
			list_del(&coproc_emu->list);
			break;
		}
	}
	spin_unlock(&coproc_emu_devices_lock);

	if (!coproc_emu)
		return -ENODEV;

	coproc_emu_device_power_off(coproc_emu);

	/* Wait a bit to catch last irq */
	udelay(2000);

	for (i = 0; i < coproc_emu->num_irqs; ++i)
		free_irq(coproc_emu->irqs[i], coproc_emu);

	dev_notice(dev, "unregistered\n");

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id coproc_emu_of_match[] = {
	{ .compatible = "vendor_xxx,coproc_xxx", },
	{ },
};
MODULE_DEVICE_TABLE(of, coproc_emu_of_match);
#endif

static struct platform_driver coproc_emu_driver = {
	.driver	= {
		.owner		= THIS_MODULE,
		.name		= "coproc-emu",
		.of_match_table	= of_match_ptr(coproc_emu_of_match),
	},
	.probe	= coproc_emu_device_dt_probe,
	.remove	= coproc_emu_device_remove,
};

static int __init coproc_emu_init(void)
{
	pr_debug("entered\n");

	return platform_driver_register(&coproc_emu_driver);
}

static void __exit coproc_emu_exit(void)
{
	pr_debug("entered\n");

	return platform_driver_unregister(&coproc_emu_driver);
}

module_init(coproc_emu_init);
module_exit(coproc_emu_exit);

MODULE_DESCRIPTION("Remote processors emulator driver");
MODULE_AUTHOR("EPAM Systems Inc.");
MODULE_LICENSE("GPL v2");
