/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/soundwire/soundwire.h>
#include <linux/soundwire/swr-wcd.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include "swrm_registers.h"
#include "swr-wcd-ctrl.h"

#define SWR_BROADCAST_CMD_ID            0x0F
#define SWR_AUTO_SUSPEND_DELAY		3 /* delay in sec */
#define SWR_DEV_ID_MASK			0xFFFFFFFF
#define SWR_REG_VAL_PACK(data, dev, id, reg)	\
			((reg) | ((id) << 16) | ((dev) << 20) | ((data) << 24))


/* pm runtime auto suspend timer in msecs */
static int auto_suspend_timer = SWR_AUTO_SUSPEND_DELAY * 1000;
module_param(auto_suspend_timer, int,
		S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(auto_suspend_timer, "timer for auto suspend");

static int swrm_probe(struct platform_device *pdev)
{
	struct swr_mstr_ctrl *swrm;
	struct swr_ctrl_platform_data *pdata;
	struct swr_device *swr_dev, *safe;
	int ret;

	printk(KERN_ERR "%s enter,qcom,swr-wcd dev probe.\n",__func__);
	/* Allocate soundwire master driver structure */
	swrm = kzalloc(sizeof(struct swr_mstr_ctrl), GFP_KERNEL);
	if (!swrm) {
		dev_err(&pdev->dev, "%s: no memory for swr mstr controller\n",
			 __func__);
		ret = -ENOMEM;
		goto err_memory_fail;
	}
	swrm->dev = &pdev->dev;
	swrm->pdev = pdev;
	platform_set_drvdata(pdev, swrm);
	swr_set_ctrl_data(&swrm->master, swrm);
	/*pdata is from tasha_codec driver*/
	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "%s: pdata from parent is NULL\n",
			__func__);
		ret = -EINVAL;
		goto err_pdata_fail;
	}
	swrm->master.dev.parent = &pdev->dev;
	swrm->master.dev.of_node = pdev->dev.of_node;
	swrm->master.num_port = 0;
	swrm->num_enum_slaves = 0;
	swrm->rcmd_id = 0;
	swrm->wcmd_id = 0;
	swrm->slave_status = 0;
	swrm->state = SWR_MSTR_RESUME;
	init_completion(&swrm->reset);
	init_completion(&swrm->broadcast);
	mutex_init(&swrm->mlock);
	INIT_LIST_HEAD(&swrm->mport_list);
	mutex_init(&swrm->reslock);
	printk(KERN_ERR "%s: register swrm_master(qcom,swrm_wcd)\n",__func__);
	ret = swr_register_master(&swrm->master);
	if (ret) {
		dev_err(&pdev->dev, "%s: error adding swr master\n", __func__);
		goto err_mstr_fail;
	}
        
	/*register swr881x dev,cpf 2016.04.06*/
	if (pdev->dev.of_node)
		of_register_swr_devices(&swrm->master);

	/* Add devices registered with board-info as the
	   controller will be up now
	 */
	swr_master_add_boarddevices(&swrm->master);
	mutex_lock(&swrm->mlock);
	/* Enumerate slave devices */
	list_for_each_entry_safe(swr_dev, safe, &swrm->master.devices,
				 dev_list) {
		/*swr881x dev startup,cpf 2016.04.06*/
		ret = swr_startup_devices(swr_dev);
		if (ret)
			list_del(&swr_dev->dev_list);
	}
	mutex_unlock(&swrm->mlock);
	pm_runtime_set_autosuspend_delay(&pdev->dev, auto_suspend_timer);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_mark_last_busy(&pdev->dev);

	return 0;
err_mstr_fail:
err_pdata_fail:
	kfree(swrm);
err_memory_fail:
	return ret;
}

static int swrm_remove(struct platform_device *pdev)
{
	struct swr_mstr_ctrl *swrm = platform_get_drvdata(pdev);
	if (swrm->mstr_port) {
		kfree(swrm->mstr_port->port);
		kfree(swrm->mstr_port);
	}
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	swr_unregister_master(&swrm->master);
	mutex_destroy(&swrm->mlock);
	kfree(swrm);
	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int swrm_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct swr_mstr_ctrl *swrm = platform_get_drvdata(pdev);
	int ret = 0;
	struct swr_master *mstr = &swrm->master;
	struct swr_device *swr_dev;

	dev_dbg(dev, "%s: pm_runtime: resume, state:%d\n",
		__func__, swrm->state);
	mutex_lock(&swrm->reslock);
	if ((swrm->state == SWR_MSTR_PAUSE) ||
	    (swrm->state == SWR_MSTR_DOWN)) {
		swrm->state = SWR_MSTR_UP;
                //swr_dev = list_first_entry(&mstr->devices, typeof(*swr_dev), dev_list);
		list_for_each_entry(swr_dev, &mstr->devices, dev_list) {
			switch(swr_dev->addr)
			{
				case 0x20170211:
				case 0x21170213:
					ret = swr_device_up(swr_dev);
					if (ret) {
						dev_err(dev,
							"%s: failed to wakeup swr dev %d\n",
							__func__, swr_dev->dev_num);
						goto exit;
					}
					break;
				default:
					continue;
			}
		}
	}
exit:
	pm_runtime_set_autosuspend_delay(&pdev->dev, auto_suspend_timer);
	mutex_unlock(&swrm->reslock);
	return ret;
}

static bool swrm_is_port_en(struct swr_master *mstr)
{
	return !!(mstr->num_port);
}

static int swrm_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct swr_mstr_ctrl *swrm = platform_get_drvdata(pdev);
	int ret = 0;
	struct swr_master *mstr = &swrm->master;
	struct swr_device *swr_dev;

	dev_dbg(dev, "%s: pm_runtime: suspend state: %d\n",
		__func__, swrm->state);
	mutex_lock(&swrm->reslock);
	if ((swrm->state == SWR_MSTR_RESUME) ||
	    (swrm->state == SWR_MSTR_UP)) {
		if (swrm_is_port_en(&swrm->master)) {
			dev_dbg(dev, "%s swrm_pa are enabled\n", __func__);
			ret = -EBUSY;
			goto exit;
		}
		list_for_each_entry(swr_dev, &mstr->devices, dev_list) {
			ret = swr_device_down(swr_dev);
			if (ret) {
				dev_err(dev,
					"%s: failed to shutdown swr dev %d\n",
					__func__, swr_dev->dev_num);
				goto exit;
			}
		}
		swrm->state = SWR_MSTR_DOWN;
	}
exit:
	mutex_unlock(&swrm->reslock);
	return ret;
}

static int swrm_runtime_idle(struct device *dev)
{
	if (pm_runtime_autosuspend_expiration(dev)) {
		pm_runtime_autosuspend(dev);
		return -EAGAIN;
	}
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static int swrm_device_down(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct swr_mstr_ctrl *swrm = platform_get_drvdata(pdev);
	int ret = 0;
	struct swr_master *mstr = &swrm->master;
	struct swr_device *swr_dev;

	dev_dbg(dev, "%s: swrm state: %d\n", __func__, swrm->state);
	mutex_lock(&swrm->reslock);
	if ((swrm->state == SWR_MSTR_RESUME) ||
	    (swrm->state == SWR_MSTR_UP)) {
		list_for_each_entry(swr_dev, &mstr->devices, dev_list) {
			ret = swr_device_down(swr_dev);
			if (ret)
				dev_err(dev,
					"%s: failed to shutdown swr dev %d\n",
					__func__, swr_dev->dev_num);
		}
		dev_dbg(dev, "%s: Shutting down SWRM\n", __func__);
		pm_runtime_disable(dev);
		pm_runtime_set_suspended(dev);
		pm_runtime_enable(dev);
		swrm->state = SWR_MSTR_DOWN;
	}
	mutex_unlock(&swrm->reslock);
	return ret;
}

/**
 * swrm_wcd_notify - parent device can notify to soundwire master through
 * this function
 * @pdev: pointer to platform device structure
 * @id: command id from parent to the soundwire master
 * @data: data from parent device to soundwire master
 */
int swrm_wcd_notify(struct platform_device *pdev, u32 id, void *data)
{
	struct swr_mstr_ctrl *swrm;
	int ret = 0;
	struct swr_master *mstr;
	struct swr_device *swr_dev;

	if (!pdev) {
		pr_err("%s: pdev is NULL\n", __func__);
		return -EINVAL;
	}
	swrm = platform_get_drvdata(pdev);
	if (!swrm) {
		dev_err(&pdev->dev, "%s: swrm is NULL\n", __func__);
		return -EINVAL;
	}
	mstr = &swrm->master;

	dev_err(swrm->dev, "%s \n", __func__);
	switch (id) {
	case SWR_CH_MAP:
		if (!data) {
			dev_err(swrm->dev, "%s: data is NULL\n", __func__);
			ret = -EINVAL;
		}
		break;
	case SWR_DEVICE_DOWN:
		dev_err(swrm->dev, "%s: swr master down called\n", __func__);
		mutex_lock(&swrm->mlock);
		if ((swrm->state == SWR_MSTR_PAUSE) ||
		    (swrm->state == SWR_MSTR_DOWN))
			dev_err(swrm->dev, "%s: SWR master is already Down: %d\n",
				__func__, swrm->state);
		else
			swrm_device_down(&pdev->dev);
		mutex_unlock(&swrm->mlock);
		break;
	case SWR_DEVICE_UP:
		dev_err(swrm->dev, "%s: swr master up called\n", __func__);
		mutex_lock(&swrm->mlock);
		if ((swrm->state == SWR_MSTR_RESUME) ||
		    (swrm->state == SWR_MSTR_UP)) {
			dev_err(swrm->dev, "%s: SWR master is already UP: %d\n",
				__func__, swrm->state);
		} else {
			swrm->state = SWR_MSTR_DOWN;
			pm_runtime_get_sync(&pdev->dev);
			list_for_each_entry(swr_dev, &mstr->devices, dev_list) {
				ret = swr_reset_device(swr_dev);
#if 0
				ret = swr_device_up(swr_dev);
#endif
				if (ret) {
					dev_err(swrm->dev,
						"%s: failed to reset swr device %d\n",
						__func__, swr_dev->dev_num);
				}
			}
			pm_runtime_mark_last_busy(&pdev->dev);
//			pm_runtime_put_autosuspend(&pdev->dev);
		}
		mutex_unlock(&swrm->mlock);
		break;
	default:
		dev_err(swrm->dev, "%s: swr master unknown id %d\n",
			__func__, id);
		break;
	}
	return ret;
}
EXPORT_SYMBOL(swrm_wcd_notify);

#ifdef CONFIG_PM_SLEEP
static int swrm_suspend(struct device *dev)
{
	int ret = -EBUSY;
	struct platform_device *pdev = to_platform_device(dev);
	struct swr_mstr_ctrl *swrm = platform_get_drvdata(pdev);

	dev_dbg(dev, "%s: system suspend, state: %d\n", __func__, swrm->state);
	if (!pm_runtime_enabled(dev) || !pm_runtime_suspended(dev)) {
		ret = swrm_runtime_suspend(dev);
		if (!ret) {
			/*
			 * Synchronize runtime-pm and system-pm states:
			 * At this point, we are already suspended. If
			 * runtime-pm still thinks its active, then
			 * make sure its status is in sync with HW
			 * status. The three below calls let the
			 * runtime-pm know that we are suspended
			 * already without re-invoking the suspend
			 * callback
			 */
			pm_runtime_disable(dev);
			pm_runtime_set_suspended(dev);
			pm_runtime_enable(dev);
		}
	}
	if (ret == -EBUSY) {
		/*
		 * There is a possibility that some audio stream is active
		 * during suspend. We dont want to return suspend failure in
		 * that case so that display and relevant components can still
		 * go to suspend.
		 * If there is some other error, then it should be passed-on
		 * to system level suspend
		 */
		ret = 0;
	}
	return ret;
}

static int swrm_resume(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct swr_mstr_ctrl *swrm = platform_get_drvdata(pdev);

	dev_dbg(dev, "%s: system resume, state: %d\n", __func__, swrm->state);
	if (!pm_runtime_enabled(dev) || !pm_runtime_suspend(dev)) {
		ret = swrm_runtime_resume(dev);
		if (!ret) {
			pm_runtime_mark_last_busy(dev);
			pm_request_autosuspend(dev);
		}
	}
	return ret;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops swrm_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(
		swrm_suspend,
		swrm_resume
	)
	SET_RUNTIME_PM_OPS(
		swrm_runtime_suspend,
		swrm_runtime_resume,
		swrm_runtime_idle
	)
};

static struct of_device_id swrm_dt_match[] = {
	{
		.compatible = "qcom,swr-wcd",
	},
	{}
};

static struct platform_driver swr_mstr_driver = {
	.probe = swrm_probe,
	.remove = swrm_remove,
	.driver = {
		.name = SWR_WCD_NAME,
		.owner = THIS_MODULE,
		.pm = &swrm_dev_pm_ops,
		.of_match_table = swrm_dt_match,
	},
};

static int __init swrm_init(void)
{
	printk(KERN_ERR "%s: swr_mstr_driver register\n",__func__);
	return platform_driver_register(&swr_mstr_driver);
}
subsys_initcall(swrm_init);

static void __exit swrm_exit(void)
{
	platform_driver_unregister(&swr_mstr_driver);
}
module_exit(swrm_exit);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("WCD SoundWire Controller");
MODULE_ALIAS("platform:swr-wcd");
