#ifndef __LINUX_BQ25892_CHARGER_H__
#define __LINUX_BQ25892_CHARGER_H__

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/mutex.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/workqueue.h>

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	SOC	= BIT(3),
};

struct bq25892_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct bq25892_charger {
	struct i2c_client	*client;
	struct device		*dev;

	bool			recharge_disabled;
	int			recharge_mv;
	bool			iterm_disabled;
	int			iterm_ma;
	int			vfloat_mv;
	int			chg_present;
	int			fake_battery_soc;
	bool			chg_autonomous_mode;
	bool			disable_apsd;
	bool			using_pmic_therm;
	bool			jeita_supported;
	bool			battery_missing;
	const char		*bms_psy_name;
	bool			resume_completed;
	bool			irq_waiting;
	struct delayed_work	chg_remove_work;

	/* status tracking */
	bool			batt_full;
	bool			batt_hot;
	bool			batt_cold;
	bool			batt_warm;
	bool			batt_cool;

	int			charging_disabled_status;
	int			usb_suspended_status;
	int			fastchg_current_max_ma;
	int			workaround_flags;

	int			parallel_pin_polarity_setting;
	bool			parallel_charger;
	bool			parallel_charger_present;
	bool			bms_controlled_charging;
	bool			apsd_rerun;
	bool			usbin_ov;
	bool			chg_remove_work_scheduled;
	bool			force_hvdcp_2p0;

	/* psy */
	struct power_supply	*usb_psy;
	int			usb_psy_ma;
	struct power_supply	*bms_psy;
	struct power_supply	batt_psy;
	struct power_supply	parallel_psy;

	struct bq25892_regulator	otg_vreg;
	struct mutex		irq_complete;

	struct dentry		*debug_root;
	u32			peek_poke_address;

	/* adc_tm parameters */
	struct qpnp_vadc_chip	*vadc_dev;
	struct qpnp_adc_tm_chip	*adc_tm_dev;
	struct qpnp_adc_tm_btm_param	adc_param;

	/* jeita parameters */
	int			batt_hot_decidegc;
	int			batt_cold_decidegc;
	int			batt_warm_decidegc;
	int			batt_cool_decidegc;
	int			batt_missing_decidegc;
	unsigned int		batt_warm_ma;
	unsigned int		batt_warm_mv;
	unsigned int		batt_cool_ma;
	unsigned int		batt_cool_mv;

	unsigned int			psel_gpio;		//mzy add 20160415
	//lrh
	struct mutex			adcc_lock_mutex;
	unsigned long			adcc_last_jiffies;
	bool					adcc_processing;
	struct mutex			cpc_lock_mutex;
	bool					cpc_processing;
	int					max_ilim_ma;
	struct mutex			pm_lock;
	int					wake_reasons;
};

struct smb_irq_info {
	const char		*name;
	int (*smb_irq)(struct bq25892_charger *chip, u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct smb_irq_info	irq_info[4];
};


struct battery_status {
	bool			batt_hot;
	bool			batt_warm;
	bool			batt_cool;
	bool			batt_cold;
	bool			batt_present;
};

enum {
	BATT_HOT = 0,
	BATT_WARM,
	BATT_NORMAL,
	BATT_COOL,
	BATT_COLD,
	BATT_MISSING,
	BATT_STATUS_MAX,
};

enum {
	POWER_SUPPLY_CHECKING = 0,
	POWER_SUPPLY_STANDARD,
	POWER_SUPPLY_NOT_STANDARD,
};

#endif