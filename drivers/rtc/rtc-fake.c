/*
 * RTC fake device/driver
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>

static time64_t                     rtc_time_sec = 1474903385; /* Initial time: Sep 26, 2016 */
static struct rtc_wkalrm            rtc_alarm;
static struct platform_device *     rtc_fake_dev = NULL;


static int rtc_fake_set_mmss(struct device *dev, unsigned long secs)
{
    dev_info(dev, "%s, secs = %lu\n", __func__, secs);
    return 0;
}

static int rtc_fake_alarm_irq_enable(struct device *dev, unsigned int enable)
{
    return 0;
}

static int rtc_fake_proc(struct device *dev, struct seq_file *seq)
{
    return 0;
}

static int rtc_fake_read_time(struct device *dev, struct rtc_time *tm)
{
    rtc_time64_to_tm(ktime_get_real_seconds() + rtc_time_sec, tm);
    return 0;
}

static int rtc_fake_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
    memcpy(alrm, &rtc_alarm, sizeof(struct rtc_wkalrm));
    return 0;
}

static int rtc_fake_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
    memcpy(&rtc_alarm, alrm, sizeof(struct rtc_wkalrm));
    return 0;
}

static struct rtc_class_ops rtc_fake_ops = {
    .proc               = rtc_fake_proc,
    .read_time          = rtc_fake_read_time,
    .read_alarm         = rtc_fake_read_alarm,
    .set_alarm          = rtc_fake_set_alarm,
    .set_mmss           = rtc_fake_set_mmss,
    .alarm_irq_enable   = rtc_fake_alarm_irq_enable,
};

static ssize_t rtc_fake_irq_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "0\n");
}

static ssize_t rtc_fake_irq_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    int retval;
    struct platform_device *pdev = to_platform_device(dev);
    struct rtc_device *rtc = platform_get_drvdata(pdev);

    retval = count;
    if (strncmp(buf, "tick", 4) == 0 && rtc->pie_enabled)
        rtc_update_irq(rtc, 1, RTC_PF | RTC_IRQF);
    else if (strncmp(buf, "alarm", 5) == 0) {
        struct rtc_wkalrm alrm;
        int err = rtc_read_alarm(rtc, &alrm);

        if (!err && alrm.enabled)
            rtc_update_irq(rtc, 1, RTC_AF | RTC_IRQF);

    } else if (strncmp(buf, "update", 6) == 0 && rtc->uie_rtctimer.enabled)
        rtc_update_irq(rtc, 1, RTC_UF | RTC_IRQF);
    else
        retval = -EINVAL;

    return retval;
}

static DEVICE_ATTR(irq, S_IRUGO | S_IWUSR, rtc_fake_irq_show, rtc_fake_irq_store);

static int rtc_fake_probe(struct platform_device *pdev)
{
    int err;
    struct rtc_device *rtc;

    /* Initially alarm is disabled. */
    rtc_alarm.enabled = 0;
    rtc_alarm.pending = 0;
    rtc_alarm.time.tm_mon = -1;
    rtc_alarm.time.tm_mday = -1;
    rtc_alarm.time.tm_year = -1;
    rtc_alarm.time.tm_hour = -1;
    rtc_alarm.time.tm_min = -1;
    rtc_alarm.time.tm_sec = -1;

    /* */
    device_init_wakeup(&pdev->dev, 1);

    rtc = devm_rtc_device_register(&pdev->dev,
            dev_name(&pdev->dev), &rtc_fake_ops, THIS_MODULE);

    if (IS_ERR(rtc)) {
        return PTR_ERR(rtc);
    }

    err = device_create_file(&pdev->dev, &dev_attr_irq);
    if (err)
        dev_err(&pdev->dev, "Unable to create sysfs entry: %s\n",
            dev_attr_irq.attr.name);

    platform_set_drvdata(pdev, rtc);
    return 0;
}

static int rtc_fake_remove(struct platform_device *pdev)
{
    device_remove_file(&pdev->dev, &dev_attr_irq);
    return 0;
}

static struct platform_driver rtc_fake_driver = {
    .probe  = rtc_fake_probe,
    .remove = rtc_fake_remove,
    .driver = {
        .name = "rtc-fake",
    },
};

static int __init fake_rtc_init(void)
{
    int err;

    if ((err = platform_driver_register(&rtc_fake_driver)))
        return err;

    if ((rtc_fake_dev = platform_device_alloc("rtc-fake", 0)) == NULL) {
        err = -ENOMEM;
        goto exit_driver_unregister;
    }

    if ((err = platform_device_add(rtc_fake_dev)))
        goto exit_put;

    return 0;

exit_put:
    platform_device_put(rtc_fake_dev);

exit_driver_unregister:
    platform_driver_unregister(&rtc_fake_driver);
    return err;
}

static void __exit fake_rtc_exit(void)
{
    platform_device_unregister(rtc_fake_dev);
    platform_driver_unregister(&rtc_fake_driver);
}

MODULE_DESCRIPTION("RTC fake device driver");
MODULE_LICENSE("GPL");

module_init(fake_rtc_init);
module_exit(fake_rtc_exit);
