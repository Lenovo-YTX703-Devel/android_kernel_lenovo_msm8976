/*
 * BQ27541 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/BQ27541.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 * http://www.ti.com/product/bq27425-g1
 */
#define DEBUG
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <asm/unaligned.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power/bq27541_battery.h>

#define DRIVER_VERSION			"1.1.0"
/* Bq27541 standard data commands */
#define BQ27541_REG_CNTL		0x00
#define BQ27541_REG_AR			0x02
#define BQ27541_REG_ARTTE		0x04
#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_NAC			0x0C
#define BQ27541_REG_FAC			0x0e
#define BQ27541_REG_RM			0x10
#define BQ27541_REG_FCC			0x12
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_TTE			0x16
#define BQ27541_REG_TTF			0x18
#define BQ27541_REG_SI			0x1a
#define BQ27541_REG_STTE		0x1c
#define BQ27541_REG_MLI			0x1e
#define BQ27541_REG_MLTTE		0x20
#define BQ27541_REG_AE			0x22
#define BQ27541_REG_AP			0x24
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_SOH			0x28
#define BQ27541_REG_CC			0x2a
#define BQ27541_REG_SOC			0x2c
#define BQ27541_REG_NIC			0x2e
#define BQ27541_REG_ICR			0x30
#define BQ27541_REG_LOGIDX		0x32
#define BQ27541_REG_LOGBUF		0x34

#define BQ27541_FLAG_SOCF		BIT(1)
#define BQ27541_FLAG_OTC		BIT(15)
#define BQ27541_FLAG_FC			BIT(9)
#define BQ27541_FLAG_DSG		BIT(0)
#define BQ27541_FLAG_SOC1		BIT(2)
#define BQ27541_FLAG_CHGS   BIT(8)

/* Control subcommands */
#define BQ27541_SUBCMD_CTNL_STATUS  0x0000
#define BQ27541_SUBCMD_DEVCIE_TYPE  0x0001
#define BQ27541_SUBCMD_FW_VER  0x0002
#define BQ27541_SUBCMD_HW_VER  0x0003
#define BQ27541_SUBCMD_DF_CSUM  0x0004
#define BQ27541_SUBCMD_PREV_MACW   0x0007
#define BQ27541_SUBCMD_CHEM_ID   0x0008
#define BQ27541_SUBCMD_BD_OFFSET   0x0009
#define BQ27541_SUBCMD_INT_OFFSET  0x000a
#define BQ27541_SUBCMD_CC_VER   0x000b
#define BQ27541_SUBCMD_OCV  0x000c
#define BQ27541_SUBCMD_BAT_INS   0x000d
#define BQ27541_SUBCMD_BAT_REM   0x000e
#define BQ27541_SUBCMD_SET_HIB   0x0011
#define BQ27541_SUBCMD_CLR_HIB   0x0012
#define BQ27541_SUBCMD_SET_SLP   0x0013
#define BQ27541_SUBCMD_CLR_SLP   0x0014
#define BQ27541_SUBCMD_FCT_RES   0x0015
#define BQ27541_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27541_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27541_SUBCMD_SEALED   0x0020
#define BQ27541_SUBCMD_ENABLE_IT    0x0021
#define BQ27541_SUBCMD_DISABLE_IT   0x0023
#define BQ27541_SUBCMD_CAL_MODE  0x0040
#define BQ27541_SUBCMD_RESET   0x0041
#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN   (-2731)
#define BQ27541_INIT_DELAY   ((HZ)*1)
#define ODMM_CHARGING_STATUS

#define BATT_DBG(format,...) do{ printk("[ODM_BATT]");\
						printk(format, ## __VA_ARGS__);	\
			}while(0) 
struct bq27541_device_info;
struct bq27541_access_methods {
	int (*read)(struct bq27541_device_info *di, u8 reg, bool single);
	int (*write)(struct bq27541_device_info *di, u8 reg,
		     void *value, bool single);
};

enum bq27541_chip { BQ27541, BQ27500, BQ27425};

struct bq27541_reg_cache {
	int temperature;
	int charge_full;
	int cycle_count;
	int capacity;
	int flags;
	int power_avg;
	int health;
};

struct bq27541_reg_no_compare_cache 
{
	int energy;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int remain;
	int batt_vol;
	int current_now;
	int nac;
};

struct bq27541_device_info {
	struct device 		*dev;
	int			id;
	enum bq27541_chip	chip;

	struct bq27541_reg_cache cache;
	struct bq27541_reg_no_compare_cache no_cmp_cache;
	int charge_design_full;
	int irq;
	struct gpio_edge_desc *gpio_wakeup;

	unsigned long last_update;
	struct delayed_work work;

	struct power_supply	bat;

	struct bq27541_access_methods bus;

	struct mutex lock;
	bool		b_suspended;
};

static enum power_supply_property bq27541_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,	
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
};
#ifndef ODMM_CHARGING_TEST
#define ODMM_CHARGING_TEST

static bool enable_charging = true;
static bool enable_test = false;
#endif
//#define TEMP_REGION_DEBUG
#ifdef TEMP_REGION_DEBUG
static int smbchg_temp_debug= 250;

module_param_named(
	temp_debug, smbchg_temp_debug, int, S_IRUSR | S_IWUSR
);

static int smbchg_temp_debug_interval = 20;
static int cur_count = 0;

module_param_named(
	temp_debug_interval, smbchg_temp_debug_interval, int, S_IRUSR | S_IWUSR
);

static int smbchg_temp_debug_auto = 0;

module_param_named(
	temp_debug_auto, smbchg_temp_debug_auto, int, S_IRUSR | S_IWUSR
);

static int smbchg_temp_debug_auto_up = 1;

module_param_named(
	temp_debug_auto_up, smbchg_temp_debug_auto_up, int, S_IRUSR | S_IWUSR
);
#endif


static unsigned int poll_interval = 1;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
				"0 disables polling");

#ifndef  ODMM_CHARGING_STATUS
bool bq24196_is_charging();
#endif


/*
 * Common code for BQ27541 devices
*/

static inline int bq27541_read(struct bq27541_device_info *di, u8 reg,
		bool single)
{
	return di->bus.read(di, reg, single);
}

#if 0
static int bq27541_write(struct bq27541_device_info *di, u8 reg,
			     void *val, bool single)
{
	return di->bus.write(di, reg, val, single);
}
#endif

#ifdef ODMM_CHARGING_TEST
int bq27541_config_charging_status(bool start_test,bool start_charging)
{
	enable_test = start_test;
	if(start_test) 
			enable_charging = start_charging;
	return 0;
}
EXPORT_SYMBOL(bq27541_config_charging_status);
#endif

/*
 * StateOfCharge( )
 * Return the battery State-of-Charge
 * Or < 0 if something fails.
*/
static int bq27541_battery_read_rsoc(struct bq27541_device_info *di)
{
	int rsoc;
	rsoc = bq27541_read(di, BQ27541_REG_SOC, false);
	if (rsoc < 0)
		dev_dbg(di->dev, "error reading relative State-of-Charge\n");
	return rsoc;
}

/* 
	* RemainingCapacity( ) 
	* Return the battery remaining capacity
  * Or < 0 if something fails. 
*/
static int bq27541_remaining_capacity(struct bq27541_device_info *di)
{
	int cap = 0;

	cap = bq27541_read(di, BQ27541_REG_RM, false);
	if (cap < 0) {
		dev_err(di->dev, "error reading remaining capacity.\n");
		return cap;
	}
	
	return cap;
}
/*
 * NomAvailableCapacity( )
 * Return the battery Nominal available capaciy in mAh
 * Or < 0 if something fails.
*/
static int bq27541_battery_read_nac(struct bq27541_device_info *di)
{
	int nac;

	nac = bq27541_read(di, BQ27541_REG_NAC, false);
	if (nac < 0) {
		dev_dbg(di->dev, "error reading charge register %02x: %d\n",
			BQ27541_REG_NAC, nac);
	}
	
	return nac;
}
/*
 * FilteredRM()
 * Return the battery Available energy in mWh
 * Or < 0 if something fails.
*/
static int bq27541_battery_read_energy(struct bq27541_device_info *di)
{
	int ae;

	ae = bq27541_read(di, BQ27541_REG_AE, false);
	if (ae < 0) {
		dev_dbg(di->dev, "error reading available energy\n");
		return ae;
	}

	return ae;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
*/
static int bq27541_battery_read_temperature(struct bq27541_device_info *di)
{
	int temp;

	temp = bq27541_read(di, BQ27541_REG_TEMP, false);
	if (temp < 0) {
		dev_err(di->dev, "error reading temperature\n");
		return -200;
	}

	temp = (temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN);
	return temp;
}

/*
 * AverageCurrent( )
 * Return the battery average current in mA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
*/
static int bq27541_average_current(struct bq27541_device_info *di)
{
	int curr;

	curr = bq27541_read(di, BQ27541_REG_AI, false);
	if (curr < 0) {
		dev_err(di->dev, "error reading current\n");
	}

	return (int)((s16)curr);
}

/*
 * Voltage( )
 * Return the battery Voltage in millivolts
 * Or < 0 if something fails.
*/
static int bq27541_battery_voltage(struct bq27541_device_info *di)
{
	int volt;

	volt = bq27541_read(di, BQ27541_REG_VOLT, false);
	if (volt < 0) {
		dev_err(di->dev, "error reading voltage\n");
	}

	return volt;
}


/*
 * CycleCount( )
 * Return the battery Cycle count total
 * Or < 0 if something fails.
*/
static int bq27541_battery_read_cyct(struct bq27541_device_info *di)
{
	int cyct;

	cyct = bq27541_read(di, BQ27541_REG_CC, false);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
*/
static int bq27541_battery_read_time(struct bq27541_device_info *di, u8 reg)
{
	int tval;

	tval = bq27541_read(di, reg, false);
	if (tval < 0) {
		dev_dbg(di->dev, "error reading time register %02x: %d\n",
			reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	return tval * 60;
}

/*
 * Read a power avg register.
 * Return < 0 if something fails.
 */
static int bq27541_battery_read_pwr_avg(struct bq27541_device_info *di, u8 reg)
{
	int tval;

	tval = bq27541_read(di, reg, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading power avg rgister  %02x: %d\n",
			reg, tval);
		return tval;
	}
	
	return tval;
}

/*
 * Flags( )
 * Read flag register.
 * Return < 0 if something fails.
*/
static int bq27541_battery_read_health(struct bq27541_device_info *di)
{
	int tval;

	tval = bq27541_read(di, BQ27541_REG_FLAGS, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading flag register:%d\n", tval);
		return tval;
	}

	if (tval & BQ27541_FLAG_SOCF)
		tval = POWER_SUPPLY_HEALTH_DEAD;
	else if (tval & BQ27541_FLAG_OTC)
		tval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		tval = POWER_SUPPLY_HEALTH_GOOD;
	return tval;
}

static void bq27541_update(struct bq27541_device_info *di)
{
	struct bq27541_reg_cache cache = {0, };
	struct bq27541_reg_no_compare_cache no_cmp_cache = {0, };
	int val = 0;
	cache.flags = bq27541_read(di, BQ27541_REG_FLAGS, false);
	if (cache.flags >= 0) {
		cache.capacity = bq27541_battery_read_rsoc(di);
		cache.health = bq27541_battery_read_health(di);
	    cache.temperature = bq27541_battery_read_temperature(di);
		cache.cycle_count = bq27541_battery_read_cyct(di);
	    cache.power_avg = bq27541_battery_read_pwr_avg(di, BQ27541_REG_AP);
		
		val = bq27541_read(di, BQ27541_REG_AI, false);
		no_cmp_cache.energy = bq27541_battery_read_energy(di);
		no_cmp_cache.remain = bq27541_remaining_capacity(di);
		no_cmp_cache.time_to_empty = bq27541_battery_read_time(di,BQ27541_REG_TTE);
		no_cmp_cache.time_to_empty_avg = bq27541_battery_read_time(di,BQ27541_REG_TTECP);
		no_cmp_cache.time_to_full = bq27541_battery_read_time(di,BQ27541_REG_TTF);
		no_cmp_cache.batt_vol= bq27541_battery_voltage(di);
		no_cmp_cache.current_now = bq27541_average_current(di);
		no_cmp_cache.nac = bq27541_battery_read_nac(di);
#ifdef TEMP_REGION_DEBUG
		if(smbchg_temp_debug_auto){
			cur_count += poll_interval;
			if(cur_count >= smbchg_temp_debug_interval){
				cur_count = 0;
				if(smbchg_temp_debug_auto_up){
					smbchg_temp_debug+= 10;
				} else {
					smbchg_temp_debug -= 10;
				}
				if(smbchg_temp_debug < -100){
					smbchg_temp_debug = -100;
					smbchg_temp_debug_auto_up = 1;
				} else if(smbchg_temp_debug > 600){
					smbchg_temp_debug = 600;
					if(smbchg_temp_debug_auto > 1){
						smbchg_temp_debug_auto_up = 0;
					}
				}
			}
		}
		cache.temperature = smbchg_temp_debug;
#endif
	}

	if (memcmp(&di->cache, &cache, sizeof(cache)) != 0) {
		di->cache = cache;
		power_supply_changed(&di->bat);
	}

	di->no_cmp_cache = no_cmp_cache;

	di->last_update = jiffies;
}

static void bq27541_battery_poll(struct work_struct *work)
{
	struct bq27541_device_info *di =
		container_of(work, struct bq27541_device_info, work.work);
    
	bq27541_update(di);

	if (poll_interval > 0) {
		schedule_delayed_work(&di->work, msecs_to_jiffies(poll_interval * 1000));
	}
}
#if 0
static int bq27541_battery_status(struct bq27541_device_info *di,
	union power_supply_propval *val)
{
	int status, curr,soc;
#ifdef ODMM_CHARGING_TEST
	if(!enable_test)
	{
#endif// ODMM_CHARGING_TEST

	if (di->cache.flags & BQ27541_FLAG_FC){
		if(bq24196_is_charging()) {
			soc = bq27541_battery_read_rsoc(di);
			if(soc == 100){
				status = POWER_SUPPLY_STATUS_FULL;
			}else{
				status = POWER_SUPPLY_STATUS_CHARGING;
			}
		}else{
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
	}

#ifndef  ODMM_CHARGING_STATUS
		else if(!bq24196_is_charging()) 
#else
		else if(di->cache.flags & BQ27541_FLAG_DSG)
#endif

{
		status = POWER_SUPPLY_STATUS_DISCHARGING;
}
	else
		{
		status = POWER_SUPPLY_STATUS_CHARGING;
	}
	val->intval = status;
		
#ifdef ODMM_CHARGING_TEST
	}
	else
	{
		if(enable_charging){
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			BATT_DBG("%s ,[TEST]status charging.\n",__func__);

		}
		else{
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			BATT_DBG("%s ,[TEST]status discharging.\n",__func__);

		}

		if (di->cache.flags & BQ27541_FLAG_FC){
			status = POWER_SUPPLY_STATUS_FULL;
			BATT_DBG("%s ,[TEST]status full.\n",__func__);
		}

	}
#endif

	return 0;
}
#endif

static int bq27541_battery_status(struct bq27541_device_info *di,
	union power_supply_propval *val)
{
	int status;

	if (di->cache.flags & BQ27541_FLAG_FC)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (di->cache.flags & BQ27541_FLAG_CHGS)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else if (power_supply_am_i_supplied(&di->bat))
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		status = POWER_SUPPLY_STATUS_DISCHARGING;

	val->intval = status;

	return 0;
}

static int bq27541_battery_capacity_level(struct bq27541_device_info *di,
	union power_supply_propval *val)
{
	int level;

	
	if (di->cache.flags & BQ27541_FLAG_FC)
		level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (di->cache.flags & BQ27541_FLAG_SOC1)
		level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (di->cache.flags & BQ27541_FLAG_SOCF)
		level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

	val->intval = level;

	return 0;
}

static int bq27541_simple_value(int value,
	union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

#define to_bq27541_device_info(x) container_of((x), \
				struct bq27541_device_info, bat);

static int bq27541_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27541_device_info *di = to_bq27541_device_info(psy);
	if(di->b_suspended){
		pr_err("bq27541_battery_get_property b_suspended!!!\n");
	}
	
	mutex_lock(&di->lock);
	if (time_is_before_jiffies(di->last_update + msecs_to_jiffies(poll_interval * 1000)) 
			&& !di->b_suspended) {
		cancel_delayed_work_sync(&di->work);
		bq27541_battery_poll(&di->work.work);
	}
	mutex_unlock(&di->lock);

	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27541_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq27541_simple_value(di->no_cmp_cache.batt_vol, val);
		val->intval = val->intval * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->cache.flags < 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val ->intval = di->no_cmp_cache.current_now * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = bq27541_simple_value(di->cache.capacity, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = bq27541_battery_capacity_level(di, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if(di->cache.flags < 0)		//mzy add
			val ->intval = -200;
		else
			val ->intval = di->cache.temperature;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27541_simple_value(di->no_cmp_cache.time_to_empty, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27541_simple_value(di->no_cmp_cache.time_to_empty_avg, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27541_simple_value(di->no_cmp_cache.time_to_full, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27541_simple_value(di->no_cmp_cache.nac, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = bq27541_simple_value(di->cache.cycle_count, val);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = bq27541_simple_value(di->no_cmp_cache.energy, val);
		break;
	case POWER_SUPPLY_PROP_POWER_AVG:
		ret = bq27541_simple_value(di->cache.power_avg, val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bq27541_simple_value(di->cache.health, val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int bq27541_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	return 0;
}

static char *bq27541_supplicants[] = {
	"battery",
	"bcl",
	"fg_adc"
};

static void bq27541_external_power_changed(struct power_supply *psy)
{
	struct bq27541_device_info *di = to_bq27541_device_info(psy);

	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, 0);
}

static int bq27541_power_supply_init(struct bq27541_device_info *di)
{
	int ret;
	di->bat.name = "bms";
	di->bat.type = POWER_SUPPLY_TYPE_BMS;
	di->bat.properties = bq27541_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27541_battery_props);
	di->bat.get_property = bq27541_battery_get_property;
    di->bat.set_property = bq27541_battery_set_property;
    di->bat.supplied_to = bq27541_supplicants;
	di->bat.num_supplicants = ARRAY_SIZE(bq27541_supplicants);
	di->bat.external_power_changed = bq27541_external_power_changed;

	ret = power_supply_register(di->dev, &di->bat);
	
	if (ret) {
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;
}

static void bq27541_powersupply_unregister(struct bq27541_device_info *di)
{
	/*
	 * power_supply_unregister call bq27541_battery_get_property which
	 * call bq27541_battery_poll.
	 * Make sure that bq27541_battery_poll will not call
	 * schedule_delayed_work again after unregister (which cause OOPS).
	 */
	poll_interval = 0;

	cancel_delayed_work_sync(&di->work);

	power_supply_unregister(&di->bat);

	mutex_destroy(&di->lock);
}


/* i2c specific code */
/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static int bq27541_read_i2c(struct bq27541_device_info *di, u8 reg, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if(ret == 0){
		printk(KERN_ERR "bq27541_read_i2c ret = zero!!!\n");
	}
	if (ret < 0)
		return ret;

	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];

	return ret;
}

static int bq27541_write_i2c(struct bq27541_device_info *di, u8 reg,
			     void *val, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	unsigned char data[3];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	data[0] = reg;
	if (!single) {
		memcpy(&data[1], (char *)val, 2);
		ret = i2c_master_send(client, data, 3);
	} else {
		memcpy(&data[1], (char *)val, 1);
		ret = i2c_master_send(client, data, 2);
	}

	return ret;
}

static int bq27541_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27541_device_info *di;
	int num;
	int retval = 0;

	/* Get new ID for the new battery device */
	mutex_lock(&battery_mutex);
	num = idr_alloc(&battery_id, client, 0, 0, GFP_KERNEL);
	mutex_unlock(&battery_mutex);
	if (num < 0)
		return num;

	//name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	name = kasprintf(GFP_KERNEL, "bq27541_battery");
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	//di = kzalloc(sizeof(*di), GFP_KERNEL);
    di = devm_kzalloc(&client->dev, sizeof(struct bq27541_device_info), GFP_KERNEL);
	if (di == NULL) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
    
	di->id = num;
	di->dev = &client->dev;

	if(bq27541_read_i2c(di, BQ27541_REG_FLAGS, false) < 0)	//mzy add
	{
		pr_err("Failed to detect bq27541, device may be absent\n");
		retval = -ENODEV;
		goto batt_failed_3;
	}
	
	di->bus.read = &bq27541_read_i2c;
	di->bus.write = &bq27541_write_i2c;
	i2c_set_clientdata(client, di);

	mutex_init(&di->lock);
	INIT_DELAYED_WORK(&di->work, bq27541_battery_poll);
	di->b_suspended = true;
	retval = bq27541_power_supply_init(di);
	if (retval)
		goto batt_failed_3;
	
	bq27541_update(di);

	di->b_suspended = false;

	if (poll_interval > 0) {
		schedule_delayed_work(&di->work, msecs_to_jiffies(poll_interval * 1000));
	}

	device_init_wakeup(di->dev, 1);
	
	return 0;

batt_failed_3:
	devm_kfree(di->dev, di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27541_battery_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	bq27541_powersupply_unregister(di);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	devm_kfree(di->dev, di);

	return 0;
}

static struct of_device_id bq27541_match_table[] = {
    { .compatible = "qcom,bq27541-battery",},
    { },
};
static const struct i2c_device_id bq27541_id[] = {
	{"bq27541", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, bq27541_id);

static int bq27541_battery_suspend(struct device *dev)
{
	struct bq27541_device_info *di = dev_get_drvdata(dev);
	//printk(KERN_ERR "bq27541_battery_suspend\n");
	di->b_suspended = true;
	cancel_delayed_work_sync(&di->work);
	
	return 0;
}

static int bq27541_battery_resume(struct device *dev)
{
	struct bq27541_device_info *di = dev_get_drvdata(dev);
	di->b_suspended = false;
	//printk(KERN_ERR "bq27541_battery_resume\n");
	bq27541_battery_poll(&di->work.work);
	
	return 0;
}

static const struct dev_pm_ops bq27541_pm_ops = {
	.suspend = bq27541_battery_suspend,
	.resume = bq27541_battery_resume,
};

static struct i2c_driver bq27541_battery_driver = {
	.driver = {
		.name = "bq27541-battery",
        .owner = THIS_MODULE,
        .of_match_table	= bq27541_match_table,
        .pm = &bq27541_pm_ops,
	},
	.probe = bq27541_battery_probe,
	.remove = bq27541_battery_remove,
	.id_table = bq27541_id,
};
#if 0
module_i2c_driver(bq27541_battery_driver);

#else
static inline int bq27541_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27541 i2c driver\n");

	return ret;
}

static inline void bq27541_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}

static int __init bq27541_battery_init(void)
{
	int ret;

	ret = bq27541_battery_i2c_init();
	
	if (ret)
		return ret;
	
	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	bq27541_battery_i2c_exit();
}
module_exit(bq27541_battery_exit);
#endif

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("bq27541 battery monitor driver");

