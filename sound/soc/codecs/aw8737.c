/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/bitops.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>
#include <linux/soundwire/soundwire.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include "wsa881x.h"
#include "wsa881x-temp-sensor.h"

struct wsa_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *wsa_spkr_sus;
	struct pinctrl_state *wsa_spkr_act;
};

enum {
	SWR_DAC_PORT,
	SWR_COMP_PORT,
	SWR_BOOST_PORT,
	SWR_VISENSE_PORT,
};

struct swr_port {
	u8 port_id;
	u8 ch_mask;
	u32 ch_rate;
	u8 num_ch;
};

enum {
	WSA881X_DEV_DOWN,
	WSA881X_DEV_UP,
};

/*
 * Private data Structure for wsa881x. All parameters related to
 * WSA881X codec needs to be defined here.
 */
struct wsa881x_priv {
	struct regmap *regmap;
	struct device *dev;
	struct swr_device *swr_slave;
	struct snd_soc_codec *codec;
	bool comp_enable;
	bool boost_enable;
	bool visense_enable;
	struct swr_port port[WSA881X_MAX_SWR_PORTS];
	struct wsa_pinctrl_info pinctrl_info;
	int pd_gpio;
	struct wsa881x_tz_priv tz_pdata;
	int bg_cnt;
	int clk_cnt;
	int version;
	struct mutex bg_lock;
	struct mutex res_lock;
	struct snd_info_entry *entry;
	struct snd_info_entry *version_entry;
	int state;
	struct delayed_work ocp_ctl_work;
};

#define WSA881X_VERSION_ENTRY_SIZE 27

static ssize_t wsa881x_codec_version_read(struct snd_info_entry *entry,
			       void *file_private_data, struct file *file,
			       char __user *buf, size_t count, loff_t pos)
{
	struct wsa881x_priv *wsa881x;
	char buffer[WSA881X_VERSION_ENTRY_SIZE];
	int len;

	wsa881x = (struct wsa881x_priv *) entry->private_data;
	if (!wsa881x) {
		pr_err("%s: wsa881x priv is null\n", __func__);
		return -EINVAL;
	}

	if (WSA881X_IS_2_0(wsa881x->version))
		len = snprintf(buffer, sizeof(buffer),
			       "WSA881X-SOUNDWIRE_2_0\n");
	else
		len = snprintf(buffer, sizeof(buffer),
			       "WSA881X-SOUNDWIRE_1_0\n");

	return simple_read_from_buffer(buf, count, &pos, buffer, len);
}

static struct snd_info_entry_ops wsa881x_codec_info_ops = {
	.read = wsa881x_codec_version_read,
};

/*
 * wsa881x_codec_info_create_codec_entry - creates wsa881x module
 * @codec_root: The parent directory
 * @codec: Codec instance
 *
 * Creates wsa881x module and version entry under the given
 * parent directory.
 *
 * Return: 0 on success or negative error code on failure.
 */
int wsa881x_codec_info_create_codec_entry(struct snd_info_entry *codec_root,
					  struct snd_soc_codec *codec)
{
	struct snd_info_entry *version_entry;
	struct wsa881x_priv *wsa881x;
	struct snd_soc_card *card;
	char name[80];

	if (!codec_root || !codec)
		return -EINVAL;

	wsa881x = snd_soc_codec_get_drvdata(codec);
	card = codec->card;
	snprintf(name, sizeof(name), "%s.%x", "wsa881x",
		 (u32)wsa881x->swr_slave->addr);

	wsa881x->entry = snd_register_module_info(codec_root->module,
						  (const char *)name,
						  codec_root);
	if (!wsa881x->entry) {
		dev_dbg(codec->dev, "%s: failed to create wsa881x entry\n",
			__func__);
		return -ENOMEM;
	}

	version_entry = snd_info_create_card_entry(card->snd_card,
						   "version",
						   wsa881x->entry);
	if (!version_entry) {
		dev_dbg(codec->dev, "%s: failed to create wsa881x version entry\n",
			__func__);
		return -ENOMEM;
	}

	version_entry->private_data = wsa881x;
	version_entry->size = WSA881X_VERSION_ENTRY_SIZE;
	version_entry->content = SNDRV_INFO_CONTENT_DATA;
	version_entry->c.ops = &wsa881x_codec_info_ops;

	if (snd_info_register(version_entry) < 0) {
		snd_info_free_entry(version_entry);
		return -ENOMEM;
	}
	wsa881x->version_entry = version_entry;

	return 0;
}
EXPORT_SYMBOL(wsa881x_codec_info_create_codec_entry);

#if 1
static int wsa881x_swr_up(struct swr_device *pdev);
static int wsa881x_swr_down(struct swr_device *pdev);
#endif 

static int aw8737_get_switch_value(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = wsa881x->comp_enable;
	return 0;
}

static int aw8737_set_switch_value(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_codec_get_drvdata(codec);
	int value = ucontrol->value.integer.value[0];
	struct swr_master *master = NULL;

	if (wsa881x == NULL){
		pr_err("%s: wsa881x priv is null\n", __func__);
		return -EINVAL;
	}
	
	master = wsa881x->swr_slave->master;
	if(!master){
		printk(KERN_ERR "%s: master is NULL\n",__func__);
		return -EINVAL;
	}

	dev_err(codec->dev, "%s: Spkr Switch  enable current %d, new %d\n",
		 __func__, wsa881x->comp_enable, value);
	wsa881x->comp_enable = value;
	if(value != 0)
		master->num_port = 4;	
	else
		master->num_port = 0;

	return 0;
}

#if 1
static int aw8737_set_pa_switch(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_codec_get_drvdata(codec);
	int value = ucontrol->value.integer.value[0];

	if (wsa881x == NULL){
		pr_err("%s: wsa881x priv is null\n", __func__);
		return -EINVAL;
	}	
	dev_err(codec->dev, "%s: Spkr Switch  enable current %d, new %d\n",
		 __func__, wsa881x->comp_enable, value);
	
	wsa881x->comp_enable = value;
	if(value == true)
		wsa881x_swr_up(wsa881x->swr_slave);
	else
		wsa881x_swr_down(wsa881x->swr_slave);

	return 0;
}
static int aw8737_get_pa_switch(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = wsa881x->comp_enable;
	return 0;
}

static struct snd_kcontrol_new aw8737_snd_controls_gpio31[] = {
	SOC_SINGLE_EXT("SWR DAC_Port Switch", SND_SOC_NOPM, 0, 1, 0,
		aw8737_get_switch_value, aw8737_set_switch_value),
	SOC_SINGLE_EXT("GPIO31 Switch", SND_SOC_NOPM, 0, 1, 0,
		aw8737_get_pa_switch, aw8737_set_pa_switch),
};
static struct snd_kcontrol_new aw8737_snd_controls_gpio32[] = {
	SOC_SINGLE_EXT("SWR DAC_Port Switch", SND_SOC_NOPM, 0, 1, 0,
		aw8737_get_switch_value, aw8737_set_switch_value),
	SOC_SINGLE_EXT("GPIO32 Switch", SND_SOC_NOPM, 0, 1, 0,
		aw8737_get_pa_switch, aw8737_set_pa_switch),
};
static struct snd_kcontrol_new aw8737_snd_controls_gpio36[] = {
	SOC_SINGLE_EXT("SWR DAC_Port Switch", SND_SOC_NOPM, 0, 1, 0,
		aw8737_get_switch_value, aw8737_set_switch_value),
	SOC_SINGLE_EXT("GPIO36 Switch", SND_SOC_NOPM, 0, 1, 0,
		aw8737_get_pa_switch, aw8737_set_pa_switch),
};
static struct snd_kcontrol_new aw8737_snd_controls_gpio104[] = {
	SOC_SINGLE_EXT("SWR DAC_Port Switch", SND_SOC_NOPM, 0, 1, 0,
		aw8737_get_switch_value, aw8737_set_switch_value),
	SOC_SINGLE_EXT("GPIO104 Switch", SND_SOC_NOPM, 0, 1, 0,
		aw8737_get_pa_switch, aw8737_set_pa_switch),
};
#endif


int wsa881x_set_channel_map(struct snd_soc_codec *codec, u8 *port, u8 num_port,
				unsigned int *ch_mask, unsigned int *ch_rate)
{
	struct wsa881x_priv *wsa881x = snd_soc_codec_get_drvdata(codec);
	int i;

	if (!port || !ch_mask || !ch_rate ||
		(num_port > WSA881X_MAX_SWR_PORTS)) {
		dev_err(codec->dev,
			"%s: Invalid port=%p, ch_mask=%p, ch_rate=%p\n",
			__func__, port, ch_mask, ch_rate);
		return -EINVAL;
	}
	for (i = 0; i < num_port; i++) {
		wsa881x->port[i].port_id = port[i];
		wsa881x->port[i].ch_mask = ch_mask[i];
		wsa881x->port[i].ch_rate = ch_rate[i];
		wsa881x->port[i].num_ch = __sw_hweight8(ch_mask[i]);
	}
	return 0;
}
EXPORT_SYMBOL(wsa881x_set_channel_map);

static void wsa881x_init(struct snd_soc_codec *codec)
{

	struct wsa881x_priv *wsa881x = snd_soc_codec_get_drvdata(codec);
	/*modify cpf 16.04.06*/
	wsa881x->version = WSA881X_2_0; 
}

static int wsa881x_probe(struct snd_soc_codec *codec)
{
	struct wsa881x_priv *wsa881x = snd_soc_codec_get_drvdata(codec);
	struct swr_device *dev;
	int ret = 0;

	if (!wsa881x)
		return -EINVAL;
        printk(KERN_ERR "%s enter\n",__func__);
	dev = wsa881x->swr_slave;
	wsa881x->codec = codec;
	codec->control_data = NULL;
	wsa881x_init(codec);
	wsa881x->state = WSA881X_DEV_UP;

	if (dev->dev_num % 2 == 0) {
		snprintf(wsa881x->tz_pdata.name, sizeof(wsa881x->tz_pdata.name), "%s.%x", "wsatz", (u8)dev->addr);
		wsa881x_init_thermal(&wsa881x->tz_pdata);
	} else {
		dev_err(&dev->dev, "%s not registering thermal zone for uneven devnum(%d)\n",__func__,dev->dev_num);
	}
	return ret;
}

static int wsa881x_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_wsa881x[] = {
	{
		.probe = wsa881x_probe,
		.remove = wsa881x_remove,
		.controls = aw8737_snd_controls_gpio31,
		.num_controls = ARRAY_SIZE(aw8737_snd_controls_gpio31),
	},
	{
		.probe = wsa881x_probe,
		.remove = wsa881x_remove,
		.controls = aw8737_snd_controls_gpio32,
		.num_controls = ARRAY_SIZE(aw8737_snd_controls_gpio32),
	},
	{
		.probe = wsa881x_probe,
		.remove = wsa881x_remove,
		.controls = aw8737_snd_controls_gpio36,
		.num_controls = ARRAY_SIZE(aw8737_snd_controls_gpio36),
	},
	{
		.probe = wsa881x_probe,
		.remove = wsa881x_remove,
		.controls = aw8737_snd_controls_gpio104,
		.num_controls = ARRAY_SIZE(aw8737_snd_controls_gpio104),
	},
};

static int wsa881x_swr_startup(struct swr_device *swr_dev)
{
	int ret = 0;
	struct wsa881x_priv *wsa881x;

	wsa881x = swr_get_dev_data(swr_dev);
	if (!wsa881x) {
		dev_err(&swr_dev->dev, "%s: wsa881x is NULL\n", __func__);
		return -EINVAL;
	}

	/*
	 * Add 5msec delay to provide sufficient time for
	 * soundwire auto enumeration of slave devices as
	 * as per HW requirement.
	 */
	usleep_range(5000, 5010);
	switch(swr_dev->addr)
	{
		case 0x20170211:
			swr_dev->dev_num = 0;
			break;
		case 0x20170212:
			swr_dev->dev_num = 1;
			break;
		case 0x21170213:
			swr_dev->dev_num = 2;
			break;
		case 0x21170214:
		default:
			swr_dev->dev_num = 3;
			break;
	}
	dev_err(&swr_dev->dev, "%s get devnum(%d)\n",__func__,swr_dev->dev_num);
	wsa881x->regmap = NULL;

	printk(KERN_ERR "%s:qcom,wsa881x codec register\n",__func__);

	ret = snd_soc_register_codec(&swr_dev->dev, &soc_codec_dev_wsa881x[swr_dev->dev_num],
				     NULL, 0);
	if (ret) {
		dev_err(&swr_dev->dev, "%s: Codec registration failed\n",
			__func__);
		goto err;
	}

err:
	return ret;
}

static int wsa881x_gpio_ctrl(struct wsa881x_priv *wsa881x, bool enable)
{
	if (wsa881x->pd_gpio < 0) {
		dev_err(wsa881x->dev, "%s: gpio is not valid %d\n",
			__func__, wsa881x->pd_gpio);
		return -EINVAL;
	}
	printk(KERN_ERR "%s: gpio(%d) is set to (%d)\n",
					__func__, wsa881x->pd_gpio,enable);
	gpio_direction_output(wsa881x->pd_gpio, enable);
	return 0;
}

static int wsa881x_gpio_init(struct swr_device *pdev)
{
	int ret = 0;
	struct wsa881x_priv *wsa881x;

	wsa881x = swr_get_dev_data(pdev);
	if (!wsa881x) {
		dev_err(&pdev->dev, "%s: wsa881x is NULL\n", __func__);
		return -EINVAL;
	}
	dev_err(&pdev->dev, "%s: gpio %d request with name %s\n",
		__func__, wsa881x->pd_gpio, dev_name(&pdev->dev));
	ret = gpio_request(wsa881x->pd_gpio, dev_name(&pdev->dev));
	if (ret) {
		if (ret == -EBUSY) {
			/* GPIO was already requested */
			dev_err(&pdev->dev,
				 "%s: gpio %d is already set to high\n",
				 __func__, wsa881x->pd_gpio);
			ret = 0;
		} else {
			dev_err(&pdev->dev, "%s: Failed to request gpio %d, err: %d\n",
				__func__, wsa881x->pd_gpio, ret);
		}
	}
	return ret;
}

static int wsa881x_pinctrl_init(struct wsa881x_priv *wsa881x,
						struct swr_device *pdev)
{
	struct pinctrl *pinctrl;

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
			pr_err("%s: Unable to get pinctrl handle\n",
					__func__);
			return -EINVAL;
	}
	wsa881x->pinctrl_info.pinctrl = pinctrl;

	wsa881x->pinctrl_info.wsa_spkr_act = pinctrl_lookup_state(pinctrl,
							"wsa_spkr_sd_act");
	if (IS_ERR(wsa881x->pinctrl_info.wsa_spkr_act)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}
	wsa881x->pinctrl_info.wsa_spkr_sus = pinctrl_lookup_state(pinctrl,
		"wsa_spkr_sd_sus");
	if (IS_ERR(wsa881x->pinctrl_info.wsa_spkr_sus)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}
	return 0;
}

static int wsa881x_swr_probe(struct swr_device *pdev)
{
	int ret = 0;
	struct wsa881x_priv *wsa881x;

	printk(KERN_ERR"%s: qcom,wsa881x\n",__func__);
	wsa881x = devm_kzalloc(&pdev->dev, sizeof(struct wsa881x_priv),
			    GFP_KERNEL);
	if (!wsa881x) {
		dev_err(&pdev->dev, "%s: cannot create memory for wsa881x\n",
			__func__);
		return -ENOMEM;
	}
	swr_set_dev_data(pdev, wsa881x);

	wsa881x->swr_slave = pdev;

	ret = wsa881x_pinctrl_init(wsa881x, pdev);
	if (ret < 0) {
		wsa881x->pd_gpio = of_get_named_gpio(pdev->dev.of_node,
				"qcom,spkr-sd-n-gpio", 0);
		if (wsa881x->pd_gpio < 0) {
			dev_err(&pdev->dev, "%s: %s property is not found %d\n",
					__func__, "qcom,spkr-sd-n-gpio",
					wsa881x->pd_gpio);
			return -EINVAL;
		}
		dev_err(&pdev->dev, "%s: reset gpio %d\n", __func__,
				wsa881x->pd_gpio);
		ret = wsa881x_gpio_init(pdev);
		if (ret)
			goto err;
	        gpio_direction_output(wsa881x->pd_gpio, false);
		/*wsa881x_gpio_ctrl(wsa881x, true);*/
	} else {
		ret = pinctrl_select_state(wsa881x->pinctrl_info.pinctrl,
				wsa881x->pinctrl_info.wsa_spkr_act);
		if (ret) {
			dev_dbg(&pdev->dev, "%s: pinctrl act failed for wsa\n",
					__func__);
		}
	}
	return 0;

err:
	return ret;
}

static int wsa881x_swr_remove(struct swr_device *pdev)
{
	struct wsa881x_priv *wsa881x;

	wsa881x = swr_get_dev_data(pdev);

	snd_soc_unregister_codec(&pdev->dev);
	if (wsa881x->pd_gpio)
		gpio_free(wsa881x->pd_gpio);
	swr_set_dev_data(pdev, NULL);
	return 0;
}

static int wsa881x_swr_up(struct swr_device *pdev)
{
	int ret = 0;
	struct wsa881x_priv *wsa881x;

	wsa881x = swr_get_dev_data(pdev);
	if (!wsa881x) {
		dev_err(&pdev->dev, "%s: wsa881x is NULL\n", __func__);
		return -EINVAL;
	}
	/*ret = wsa881x_gpio_ctrl(wsa881x, true);*/
	if (wsa881x->pd_gpio < 0) {
		dev_err(wsa881x->dev, "%s: gpio is not valid %d\n",
			__func__, wsa881x->pd_gpio);
		ret = -EINVAL;
	}
	gpio_direction_output(wsa881x->pd_gpio, 0);
	udelay(2);
        printk(KERN_ERR "gpio reset status --- 0\n");
	gpio_direction_output(wsa881x->pd_gpio, 1);
	udelay(2);
	printk(KERN_ERR "gpio first rising edge\n");
	gpio_direction_output(wsa881x->pd_gpio, 0);
	udelay(2);
        printk(KERN_ERR "gpio first trailing edge\n");
	gpio_direction_output(wsa881x->pd_gpio, 1);
	udelay(2);
        printk(KERN_ERR "gpio second rising edge\n");
	if (ret)
		dev_err(&pdev->dev, "%s: Failed to enable gpio\n", __func__);
	else
		wsa881x->state = WSA881X_DEV_UP;

	return ret;
}

static int wsa881x_swr_down(struct swr_device *pdev)
{
	struct wsa881x_priv *wsa881x;
	int ret;

	wsa881x = swr_get_dev_data(pdev);
	if (!wsa881x) {
		dev_err(&pdev->dev, "%s: wsa881x is NULL\n", __func__);
		return -EINVAL;
	}
	cancel_delayed_work_sync(&wsa881x->ocp_ctl_work);
	ret = wsa881x_gpio_ctrl(wsa881x, false);
	if (ret)
		dev_err(&pdev->dev, "%s: Failed to disable gpio\n", __func__);
	else
		wsa881x->state = WSA881X_DEV_DOWN;

	return ret;
}

static int wsa881x_swr_reset(struct swr_device *pdev)
{
	struct wsa881x_priv *wsa881x;
	
	wsa881x = swr_get_dev_data(pdev);
	if (!wsa881x) {
		dev_err(&pdev->dev, "%s: wsa881x is NULL\n", __func__);
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int wsa881x_swr_suspend(struct device *dev)
{
	dev_dbg(dev, "%s: system suspend\n", __func__);
	return 0;
}

static int wsa881x_swr_resume(struct device *dev)
{
	struct wsa881x_priv *wsa881x = swr_get_dev_data(to_swr_device(dev));

	if (!wsa881x) {
		dev_err(dev, "%s: wsa881x private data is NULL\n", __func__);
		return -EINVAL;
	}
	dev_dbg(dev, "%s: system resume\n", __func__);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops wsa881x_swr_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(wsa881x_swr_suspend, wsa881x_swr_resume)
};

static const struct swr_device_id wsa881x_swr_id[] = {
	{"wsa881x", 0},
	{}
};

static struct of_device_id wsa881x_swr_dt_match[] = {
	{
		.compatible = "qcom,wsa881x",
	},
	{}
};

static struct swr_driver wsa881x_codec_driver = {
	.driver = {
		.name = "wsa881x",
		.owner = THIS_MODULE,
		.pm = &wsa881x_swr_pm_ops,
		.of_match_table = wsa881x_swr_dt_match,
	},
	.probe = wsa881x_swr_probe,
	.remove = wsa881x_swr_remove,
	.id_table = wsa881x_swr_id,
	.device_up = wsa881x_swr_up,
	.device_down = wsa881x_swr_down,
	.reset_device = wsa881x_swr_reset,
	.startup = wsa881x_swr_startup,
};

static int __init wsa881x_codec_init(void)
{
	return swr_driver_register(&wsa881x_codec_driver);
}

static void __exit wsa881x_codec_exit(void)
{
	swr_driver_unregister(&wsa881x_codec_driver);
}

module_init(wsa881x_codec_init);
module_exit(wsa881x_codec_exit);

MODULE_DESCRIPTION("WSA881x Codec driver");
MODULE_LICENSE("GPL v2");
