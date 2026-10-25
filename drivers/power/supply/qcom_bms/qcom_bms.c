// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Unified Battery/Charger Manager
 *
 * Monolithic driver that aggregates the Qualcomm fuel gauge, PMIC charger,
 * and optional direct chargers into user-facing battery and USB power supplies.
 *
 * Copyright (C) 2026
 */

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>

#include "qcom_bms.h"

#define QCOM_BMS_PSY_NAME "qcom-battery"
#define QCOM_BMS_USB_PSY_NAME "qcom-usb"

/* Direct charge is attempted down to this VBAT (uV); the PMIC USBIN path
 * does not provide the deferred precharge below it on nabu. */
#define QCOM_BMS_DC_MIN_VBAT_UV 3000000

/* 直充在线时，它自己的大电流会把 VBUS 压下去，tcpm 因此短暂误报 offline。
 * 若此时立刻关直充，VBUS 回升 → PD 重协商 → 又开，形成约 3.3 秒周期的
 * 震荡，电池永远进不去稳态（实测 input_current_limit 一直是 0、电流持续
 * 为负）。所以在直充仍在 SWITCHING 期间给 cc_online 一个粘滞窗口。 */
#define QCOM_BMS_CC_STICKY_MS 5000

/* Charger FULL/INHIBIT claims are only credible near the float voltage. */
#define QCOM_BMS_MIN_FULL_VBAT_UV 4000000

/*
 * 而且还得看电流。大电流充电时端电压 = 开路电压 + I·R，4.9A 在 ~90mΩ 上能
 * 抬出 0.44V —— 12% 电量、真实开路 3.6V 的电池，端电压会显示 4.05V，轻松
 * 越过上面的 4.0V 门槛，于是被误判成"已充满"（DMS 就显示充满，而电量还在
 * 涨）。FULL 只可能在 CV 阶段末期成立，那时电流已经掉到截止值附近，所以
 * 加一条电流上限：0.1C ≈ 870mA。
 */
#define QCOM_BMS_FULL_CURRENT_UA 900000

/* 净放电超过这个值就认定在放电，不再等充电器状态（拔线后它滞后很久）。 */
#define QCOM_BMS_DISCHARGE_CURRENT_UA 100000

/*
 * 直充要"真的在出力"才让 PMIC 让位。500mA 是个宽松下限：正常直充一走就是
 * 安培级，空转时是 0，两者不会混淆。
 */
#define QCOM_BMS_DC_MIN_DELIVER_UA 500000

/*
 * 低于这个 VBAT 就不让直充参与。
 *
 * 实测低电量（~11%、3.6V）时 LN8000 的 ADC 会冻住（IIN 读数恒为
 * 58680uA 不变）、op_mode 缓存在 SWITCHING 但完全没有输出，同时
 * PMIC 也只剩百来毫安 —— 两条路互相干扰，插着 9V/3A 电池仍在掉。
 * 这种情况下让 PMIC 独占 VBUS 更可靠：它有 3A ICL + 1.95A FCC。
 */
/*
 * 3.7V 时禁用直充这条规则是基于"直充在低 VBAT 下空转"的错误结论写的。
 * 真正原因是 ln8000_has_fault() 把已 disable 的 IIN_OC 状态位也算作故障，
 * 导致直充被误关。那个 bug 修掉后，低 VBAT 下直充应该能正常接管，所以
 * 门槛压到 3.2V（仅作为电池真正危险时的兜底，不再是常规路径）。
 */
#define QCOM_BMS_DC_DISABLE_BELOW_UV 3200000

struct qcom_bms_fg {
	struct device *dev;
	struct device_link *link;
	const struct qcom_bms_fg_ops *ops;
	void *priv;
	unsigned int active_calls;
	struct completion idle;
	bool unregistering;
};

struct qcom_bms_charger {
	struct device *dev;
	struct device_link *link;
	const struct qcom_bms_charger_ops *ops;
	void *priv;
	unsigned int active_calls;
	struct completion idle;
	bool unregistering;
};

struct qcom_bms_fg_ref {
	struct qcom_bms_fg *backend;
	const struct qcom_bms_fg_ops *ops;
	void *priv;
};

struct qcom_bms_charger_ref {
	struct qcom_bms_charger *backend;
	const struct qcom_bms_charger_ops *ops;
	void *priv;
};

struct qcom_bms_authenticator {
	struct device *dev;
	struct device_link *link;
	bool authenticated;
};

struct qcom_bms_info {
	struct device *dev;
	struct power_supply *bms_psy;
	struct power_supply *usb_psy;

	struct mutex ops_lock;
	struct qcom_bms_fg fg;
	struct qcom_bms_charger pmic_chg;
	struct qcom_bms_charger direct_chg;
	struct qcom_bms_authenticator auth[QCOM_BMS_AUTH_MAX];
	u32 required_auth_mask;
	bool authenticated;

	struct work_struct status_work;

	/* tcpm source psy (CC-driven attach/detach) drives direct-charge control. */
	struct notifier_block psy_nb;
	struct power_supply *tcpm_psy;
	struct device_node *tcpm_np;
	bool pmic_usbin_suspended;

	/* 最近一次 tcpm 报 online=1 的 jiffies，用于直充期间的粘滞判断 */
	unsigned long cc_online_last;

	/* 上一次 evaluate 时充电器是否在位，用于检测"刚插入"这个边沿 */
	bool cc_was_online;

	int status;
	bool charge_done;
	bool input_present;
};

static struct qcom_bms_info *g_bms;
static DEFINE_MUTEX(qcom_bms_mutex);

/*
 * Grab the coordinator with ops_lock held; returns NULL if it has not probed
 * yet (or has been removed).  The caller releases with mutex_unlock(&info->ops_lock).
 */
static struct qcom_bms_info *qcom_bms_lock(void)
{
	struct qcom_bms_info *info;

	mutex_lock(&qcom_bms_mutex);
	info = g_bms;
	if (info)
		mutex_lock(&info->ops_lock);
	mutex_unlock(&qcom_bms_mutex);

	return info;
}

static bool qcom_bms_fg_acquire_locked(struct qcom_bms_info *info,
				       struct qcom_bms_fg_ref *ref)
{
	struct qcom_bms_fg *fg = &info->fg;

	if (!fg->ops || fg->unregistering)
		return false;

	if (!fg->active_calls++)
		reinit_completion(&fg->idle);
	ref->backend = fg;
	ref->ops = fg->ops;
	ref->priv = fg->priv;

	return true;
}

static void qcom_bms_fg_release(struct qcom_bms_info *info,
				struct qcom_bms_fg_ref *ref)
{
	mutex_lock(&info->ops_lock);
	if (!--ref->backend->active_calls)
		complete_all(&ref->backend->idle);
	mutex_unlock(&info->ops_lock);
}

static bool qcom_bms_charger_acquire_locked(struct qcom_bms_charger *charger,
					    struct qcom_bms_charger_ref *ref)
{
	if (!charger->ops || charger->unregistering)
		return false;

	if (!charger->active_calls++)
		reinit_completion(&charger->idle);
	ref->backend = charger;
	ref->ops = charger->ops;
	ref->priv = charger->priv;

	return true;
}

static void qcom_bms_charger_release(struct qcom_bms_info *info,
				     struct qcom_bms_charger_ref *ref)
{
	mutex_lock(&info->ops_lock);
	if (!--ref->backend->active_calls)
		complete_all(&ref->backend->idle);
	mutex_unlock(&info->ops_lock);
}

static int qcom_bms_psy_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct qcom_bms_info *info = power_supply_get_drvdata(psy);
	struct qcom_bms_fg_ref fg = {};
	int ret;

	mutex_lock(&info->ops_lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = info->status;
		mutex_unlock(&info->ops_lock);
		return 0;
	case POWER_SUPPLY_PROP_AUTHENTIC:
		val->intval = info->authenticated;
		mutex_unlock(&info->ops_lock);
		return 0;
	default:
		break;
	}

	if (!qcom_bms_fg_acquire_locked(info, &fg)) {
		mutex_unlock(&info->ops_lock);
		return -ENODATA;
	}
	mutex_unlock(&info->ops_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		ret = fg.ops->get_technology ?
			      fg.ops->get_technology(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = fg.ops->get_capacity ?
			      fg.ops->get_capacity(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = fg.ops->get_capacity_level ?
			      fg.ops->get_capacity_level(fg.priv,
							 &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = fg.ops->get_current ?
			      fg.ops->get_current(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = fg.ops->get_voltage ?
			      fg.ops->get_voltage(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		ret = fg.ops->get_power ?
			      fg.ops->get_power(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		ret = fg.ops->get_voltage_min_design ?
			      fg.ops->get_voltage_min_design(fg.priv,
							     &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		ret = fg.ops->get_voltage_max_design ?
			      fg.ops->get_voltage_max_design(fg.priv,
							     &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = fg.ops->get_charge_full_design ?
			      fg.ops->get_charge_full_design(fg.priv,
							     &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = fg.ops->get_charge_full ?
			      fg.ops->get_charge_full(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = fg.ops->get_charge_now ?
			      fg.ops->get_charge_now(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		ret = fg.ops->get_energy_full_design ?
			      fg.ops->get_energy_full_design(fg.priv,
							     &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		ret = fg.ops->get_energy_full ?
			      fg.ops->get_energy_full(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = fg.ops->get_energy_now ?
			      fg.ops->get_energy_now(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = fg.ops->get_cycle_count ?
			      fg.ops->get_cycle_count(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = fg.ops->get_health ?
			      fg.ops->get_health(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = fg.ops->get_present ?
			      fg.ops->get_present(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = fg.ops->get_temp ?
			      fg.ops->get_temp(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		ret = fg.ops->get_scope ?
			      fg.ops->get_scope(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		ret = fg.ops->get_manufacturer ?
			      fg.ops->get_manufacturer(fg.priv, &val->strval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		ret = fg.ops->get_model_name ?
			      fg.ops->get_model_name(fg.priv, &val->strval) :
			      -ENODATA;
		break;
	default:
		ret = -EINVAL;
	}

	qcom_bms_fg_release(info, &fg);
	return ret;
}

static int qcom_bms_psy_set_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     const union power_supply_propval *val)
{
	struct qcom_bms_info *info = power_supply_get_drvdata(psy);
	struct qcom_bms_fg_ref fg = {};
	int ret;

	mutex_lock(&info->ops_lock);
	if (!qcom_bms_fg_acquire_locked(info, &fg)) {
		mutex_unlock(&info->ops_lock);
		return -ENODATA;
	}
	mutex_unlock(&info->ops_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = fg.ops->set_charge_full ?
			      fg.ops->set_charge_full(fg.priv, val->intval) :
			      -EINVAL;
		break;
	default:
		ret = -EINVAL;
	}

	qcom_bms_fg_release(info, &fg);
	return ret;
}

static int qcom_bms_psy_property_is_writeable(struct power_supply *psy,
					      enum power_supply_property psp)
{
	struct qcom_bms_info *info = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&info->ops_lock);
	if (!info->fg.ops) {
		mutex_unlock(&info->ops_lock);
		return 0;
	}

	ret = psp == POWER_SUPPLY_PROP_CHARGE_FULL &&
	      info->fg.ops->set_charge_full;
	mutex_unlock(&info->ops_lock);

	return ret;
}

static const enum power_supply_property qcom_bms_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_AUTHENTIC,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static const struct power_supply_desc qcom_bms_psy_desc = {
	.name = QCOM_BMS_PSY_NAME,
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = qcom_bms_psy_props,
	.num_properties = ARRAY_SIZE(qcom_bms_psy_props),
	.get_property = qcom_bms_psy_get_property,
	.set_property = qcom_bms_psy_set_property,
	.property_is_writeable = qcom_bms_psy_property_is_writeable,
	.no_thermal = true,
};

static bool qcom_bms_charger_is_online(struct qcom_bms_charger_ref *charger)
{
	int online;

	return charger->ops && charger->ops->get_online &&
	       !charger->ops->get_online(charger->priv, &online) && online;
}

static int qcom_bms_usb_psy_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_bms_info *info = power_supply_get_drvdata(psy);
	struct qcom_bms_charger_ref pmic = {};
	struct qcom_bms_charger_ref direct = {};
	bool have_direct;
	bool have_pmic;
	bool direct_online;
	bool pmic_online;
	int ret = 0;

	mutex_lock(&info->ops_lock);
	have_direct =
		qcom_bms_charger_acquire_locked(&info->direct_chg, &direct);
	have_pmic = qcom_bms_charger_acquire_locked(&info->pmic_chg, &pmic);
	mutex_unlock(&info->ops_lock);

	direct_online = have_direct && qcom_bms_charger_is_online(&direct);
	pmic_online = have_pmic && qcom_bms_charger_is_online(&pmic);

	/* Prefer the direct charger only while it is actively online. */
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = direct_online || pmic_online;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (direct_online && direct.ops->get_status)
			ret = direct.ops->get_status(direct.priv, &val->intval);
		else if (pmic_online && pmic.ops->get_status)
			ret = pmic.ops->get_status(pmic.priv, &val->intval);
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (direct_online && direct.ops->get_current_now)
			ret = direct.ops->get_current_now(direct.priv,
							  &val->intval);
		else if (have_pmic && pmic.ops->get_current_now)
			ret = pmic.ops->get_current_now(pmic.priv,
							&val->intval);
		else
			ret = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (direct_online && direct.ops->get_voltage_now)
			ret = direct.ops->get_voltage_now(direct.priv,
							  &val->intval);
		else if (have_pmic && pmic.ops->get_voltage_now)
			ret = pmic.ops->get_voltage_now(pmic.priv,
							&val->intval);
		else
			ret = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		if (direct_online) {
			val->intval = POWER_SUPPLY_USB_TYPE_DCP;
		} else if (pmic_online && pmic.ops->get_usb_type) {
			ret = pmic.ops->get_usb_type(pmic.priv, &val->intval);
		} else {
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (direct_online && direct.ops->get_input_current_limit)
			ret = direct.ops->get_input_current_limit(direct.priv,
								  &val->intval);
		else if (have_pmic && pmic.ops->get_input_current_limit)
			ret = pmic.ops->get_input_current_limit(pmic.priv,
								&val->intval);
		else
			ret = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (direct_online && direct.ops->get_health)
			ret = direct.ops->get_health(direct.priv, &val->intval);
		else if (pmic_online && pmic.ops->get_health)
			ret = pmic.ops->get_health(pmic.priv, &val->intval);
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	default:
		ret = -EINVAL;
	}

	if (have_pmic)
		qcom_bms_charger_release(info, &pmic);
	if (have_direct)
		qcom_bms_charger_release(info, &direct);

	return ret;
}

static const enum power_supply_property qcom_bms_usb_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,      POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW, POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_USB_TYPE,    POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_HEALTH,      POWER_SUPPLY_PROP_SCOPE,
};

static const struct power_supply_desc qcom_bms_usb_psy_desc = {
	.name = QCOM_BMS_USB_PSY_NAME,
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP),
	.properties = qcom_bms_usb_psy_props,
	.num_properties = ARRAY_SIZE(qcom_bms_usb_psy_props),
	.get_property = qcom_bms_usb_psy_get_property,
};

static void qcom_bms_update_status(struct qcom_bms_info *info)
{
	struct qcom_bms_charger_ref direct = {};
	struct qcom_bms_charger_ref pmic = {};
	struct qcom_bms_fg_ref fg = {};
	int pmic_status = POWER_SUPPLY_STATUS_UNKNOWN;
	int direct_status = POWER_SUPPLY_STATUS_UNKNOWN;
	int fg_current = 0;
	int vbat_uv = 0;
	int online;
	int new_status;
	bool have_direct;
	bool have_pmic;
	bool have_fg;
	bool direct_online = false;
	bool pmic_online = false;
	bool fg_current_valid = false;
	bool fg_voltage_known = false;
	bool charge_done;
	bool input_present;
	bool changed;

	mutex_lock(&info->ops_lock);
	have_fg = qcom_bms_fg_acquire_locked(info, &fg);
	have_pmic = qcom_bms_charger_acquire_locked(&info->pmic_chg, &pmic);
	have_direct =
		qcom_bms_charger_acquire_locked(&info->direct_chg, &direct);
	mutex_unlock(&info->ops_lock);

	if (have_fg && fg.ops->get_current)
		fg_current_valid = !fg.ops->get_current(fg.priv, &fg_current);

	if (have_fg && fg.ops->get_voltage &&
	    !fg.ops->get_voltage(fg.priv, &vbat_uv))
		fg_voltage_known = vbat_uv > 0;

	if (have_pmic) {
		if (pmic.ops->get_status)
			pmic.ops->get_status(pmic.priv, &pmic_status);
		if (pmic.ops->get_online &&
		    !pmic.ops->get_online(pmic.priv, &online))
			pmic_online = online;
	}

	if (have_direct) {
		if (direct.ops->get_status)
			direct.ops->get_status(direct.priv, &direct_status);
		if (direct.ops->get_online &&
		    !direct.ops->get_online(direct.priv, &online))
			direct_online = online;
	}

	input_present = pmic_online || direct_online;
	/* At deep discharge the PMIC charger parks in INHIBIT (reported as
	 * FULL) while the direct charger is below its rate, so the battery
	 * drains on the cable.  A FULL claim is only credible near the float
	 * voltage; otherwise report the true discharging state.
	 */
	charge_done = (pmic_status == POWER_SUPPLY_STATUS_FULL ||
		       direct_status == POWER_SUPPLY_STATUS_FULL) &&
		      (!fg_voltage_known ||
		       vbat_uv >= QCOM_BMS_MIN_FULL_VBAT_UV) &&
		      (!fg_current_valid ||
		       fg_current < QCOM_BMS_FULL_CURRENT_UA);

	/* Charger state is authoritative; FG current is only a fallback. */
	if (charge_done) {
		new_status = POWER_SUPPLY_STATUS_FULL;
	} else if (!input_present ||
		   (fg_current_valid && fg_current < -QCOM_BMS_DISCHARGE_CURRENT_UA)) {
		/*
		 * 电流明显为负就一定是放电，放在充电器状态之前判断。
		 * PMIC 的 status 在拔线后会滞后很久（实测一分钟还报 CHARGING），
		 * 若按旧顺序先看 pmic_status，拔线后界面会一直显示"充电中"。
		 */
		new_status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else if (direct_status == POWER_SUPPLY_STATUS_CHARGING ||
		   pmic_status == POWER_SUPPLY_STATUS_CHARGING) {
		new_status = POWER_SUPPLY_STATUS_CHARGING;
	} else if (pmic_status == POWER_SUPPLY_STATUS_DISCHARGING ||
		   (fg_current_valid && fg_current < 0)) {
		new_status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else if (fg_current_valid && fg_current > 0)
		new_status = POWER_SUPPLY_STATUS_CHARGING;
	else
		new_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	mutex_lock(&info->ops_lock);
	changed = info->status != new_status ||
		  info->charge_done != charge_done ||
		  info->input_present != input_present;
	info->status = new_status;
	info->charge_done = charge_done;
	info->input_present = input_present;
	mutex_unlock(&info->ops_lock);

	if (changed && have_fg && fg.ops->charging_state_changed)
		fg.ops->charging_state_changed(fg.priv, new_status, charge_done,
					       input_present);

	if (have_direct)
		qcom_bms_charger_release(info, &direct);
	if (have_pmic)
		qcom_bms_charger_release(info, &pmic);
	if (have_fg)
		qcom_bms_fg_release(info, &fg);

	if (changed) {
		power_supply_changed(info->bms_psy);
		power_supply_changed(info->usb_psy);
	}
}

/* tcpm source psy changed (CC attach/detach) -> re-evaluate direct charge. */
static int qcom_bms_psy_notifier(struct notifier_block *nb, unsigned long event,
				 void *data)
{
	struct qcom_bms_info *info =
		container_of(nb, struct qcom_bms_info, psy_nb);
	struct power_supply *psy = data;

	if (event == PSY_EVENT_PROP_CHANGED && info->tcpm_np &&
	    psy->dev.of_node == info->tcpm_np)
		queue_work(system_freezable_wq, &info->status_work);

	return NOTIFY_OK;
}

/*
 * Decide whether the direct charger should be enabled.  The coordinator owns
 * this decision (driven by the real charger connection state from the tcpm CC
 * psy); the LN8000 only self-disables on a hardware fault / I2C error.  This is
 * the complement of qcom_bms_update_status(): that aggregates state, this acts
 * on it.
 */
static void qcom_bms_evaluate_direct_charge(struct qcom_bms_info *info)
{
	struct qcom_bms_charger_ref direct = {};
	struct qcom_bms_charger_ref pmic = {};
	struct qcom_bms_fg_ref fg = {};
	union power_supply_propval val;
	bool have_direct, have_pmic, have_fg;
	bool direct_online = false;
	bool cc_online = false;
	bool want_on;
	int health = POWER_SUPPLY_HEALTH_GOOD;
	int vbat_uv = QCOM_BMS_DC_MIN_VBAT_UV;
	int online;
	int ret;

	/* Resolve the tcpm source psy lazily (tcpm may probe after us). */
	if (!info->tcpm_psy && info->tcpm_np) {
		info->tcpm_psy = power_supply_get_by_reference(
			dev_fwnode(info->dev), "qcom,tcpm-psy");
		if (IS_ERR(info->tcpm_psy))
			info->tcpm_psy = NULL;
	}
	/* tcpm not probed yet; the psy notifier re-triggers us when it
	 * registers.  cc_online stays false (safe).
	 */
	if (!info->tcpm_psy)
		return;

	mutex_lock(&info->ops_lock);
	have_direct =
		qcom_bms_charger_acquire_locked(&info->direct_chg, &direct);
	have_pmic = qcom_bms_charger_acquire_locked(&info->pmic_chg, &pmic);
	have_fg = qcom_bms_fg_acquire_locked(info, &fg);
	mutex_unlock(&info->ops_lock);

	if (!have_direct || !direct.ops->set_charging_enabled ||
	    !direct.ops->get_online || !direct.ops->get_health) {
		if (have_pmic && pmic.ops->set_usbin_suspend &&
		    info->pmic_usbin_suspended) {
			pmic.ops->set_usbin_suspend(pmic.priv, false);
			info->pmic_usbin_suspended = false;
		}
		goto out;
	}

	if (!direct.ops->get_online(direct.priv, &online))
		direct_online = online;
	direct.ops->get_health(direct.priv, &health);
	if (have_fg && fg.ops->get_voltage)
		fg.ops->get_voltage(fg.priv, &vbat_uv);

	ret = power_supply_get_property(info->tcpm_psy,
					POWER_SUPPLY_PROP_ONLINE, &val);
	cc_online = !ret && val.intval;

	if (cc_online)
		info->cc_online_last = jiffies;

	/*
	 * 插入瞬间强制 resume 一次 PMIC。
	 *
	 * PMIC 的 input current limit 只在 smb_status_change_work 里写，而那个
	 * work 是由 set_usbin_suspend(false) 调度的。如果开机时没插电，段 C
	 * 会先把 pmic_usbin_suspended 刷成 false，之后插电就再也没有理由调用
	 * resume —— ICL 永远停在 0（实测 icl=0、电池 -1.8A 放电）。
	 *
	 * 所以这里只在 cc_online 由 0 变 1 的那个边沿补一次。边沿触发是幂等的
	 * 一次写入，不会像无条件调用那样把 mod_delayed_work 和 I2C 刷爆。
	 */
	if (cc_online && !info->cc_was_online && have_pmic &&
	    pmic.ops->set_usbin_suspend) {
		if (pmic.ops->set_usbin_suspend(pmic.priv, false))
			dev_warn(info->dev,
				 "failed to resume PMIC USBIN on plug-in\n");
		else
			info->pmic_usbin_suspended = false;
	}
	info->cc_was_online = cc_online;

	/*
	 * 直充正在 SWITCHING 说明 VIN 确实还在（op_mode 要求输入存在），
	 * 此时 tcpm 报 offline 多半是自己的大电流把 VBUS 压低了 —— 不是拔线。
	 * 容忍一个窗口，让直充把电流稳下来；超时仍 offline 则按真断开处理。
	 */
	if (!cc_online && direct_online &&
	    time_before(jiffies,
			info->cc_online_last + msecs_to_jiffies(QCOM_BMS_CC_STICKY_MS))) {
		cc_online = true;
		dev_dbg(info->dev,
			"tcpm offline while direct charge is switching, holding\n");
	}

	want_on = cc_online && READ_ONCE(info->authenticated) &&
		  health == POWER_SUPPLY_HEALTH_GOOD &&
		  vbat_uv >= QCOM_BMS_DC_MIN_VBAT_UV;

	/*
	 * 低 VBAT 时不让直充上场：它在这个区段会半死不活地占着 SWITCHING
	 * 状态却不输出，反而拖垮整条充电链路。让 PMIC 独自承担，等电压
	 * 爬过阈值再交还给直充。
	 */
	if (vbat_uv < QCOM_BMS_DC_DISABLE_BELOW_UV && want_on) {
		want_on = false;
		dev_info_ratelimited(info->dev,
				     "low VBAT (%d uV): keeping PMIC-only, direct charge withheld\n",
				     vbat_uv);
	}

	/*
	 * 低电压窗口：直充（LN8000）因为 VBAT 低于门槛被禁用，这是对的 ——
	 * 它不支持 3V 以下的电芯。但此时必须让 PMIC 的 USBIN 通路接手，
	 * 由 PM8150B 的硬件 trickle/pre-charge 状态机用小电流把电压拉回来。
	 *
	 * 去重依赖 info->pmic_usbin_suspended，它在上电时被初始化为 true
	 * （见 probe），所以第一次进来一定会 resume 一次、把 ICL 写下去；
	 * 之后该字段变成 false，不会再重复写，避免 mod_delayed_work 把
	 * I2C 打爆。
	 */
	/*
	 * 低 VBAT 时确保 PMIC 醒着（它才写得到 ICL）。
	 *
	 * 这里必须去重：set_usbin_suspend(false) 内部会 mod_delayed_work，
	 * 而本函数被 psy notifier 高频调用。曾经写成无条件调用，结果 work
	 * 被反复调度、I2C 被打爆（usb_type 读回 EAGAIN、icl 在 0/250mA 之间
	 * 震荡）。所以只在"确实处于挂起状态"时写一次。
	 *
	 * 门槛用 DC_DISABLE_BELOW_UV（3.2V，仅电池真正危险时的兜底）。
	 */
	if (!want_on && cc_online && have_pmic &&
	    vbat_uv < QCOM_BMS_DC_DISABLE_BELOW_UV &&
	    info->pmic_usbin_suspended &&
	    pmic.ops->set_usbin_suspend) {
		if (pmic.ops->set_usbin_suspend(pmic.priv, false))
			dev_warn_once(info->dev,
				      "failed to resume PMIC USBIN at low VBAT\n");
		else if (info->pmic_usbin_suspended)
			dev_info(info->dev,
				 "low VBAT (%d uV): resuming PMIC USBIN for pre-charge\n",
				 vbat_uv);
		info->pmic_usbin_suspended = false;
	}

	if (want_on && !direct_online) {
		ret = direct.ops->set_charging_enabled(direct.priv, true);
		if (!ret)
			dev_info(info->dev,
				 "charger connected, enabling direct charge\n");
		else if (ret != -EAGAIN)
			dev_warn(info->dev,
				 "failed to enable direct charge: %d\n", ret);
	} else if (!want_on && direct_online) {
		dev_info(
			info->dev,
			"charger disconnected or not ready, disabling direct charge\n");
		ret = direct.ops->set_charging_enabled(direct.priv, false);
		if (ret)
			dev_warn(info->dev,
				 "failed to disable direct charge: %d\n", ret);
	}

	/*
	 * Re-read the actual direct-charge state (the LN8000 may have
	 * self-disabled on a fault/RCP trip, or the enable may have been refused
	 * during the cooldown).  The coordinator owns the PMIC USBIN handover:
	 * suspend the PMIC USBIN path iff direct charge is actively online, so a
	 * self-disable (or a refused enable) restores PMIC charging.
	 */
	if (direct.ops->get_online(direct.priv, &online))
		direct_online = false;
	else
		direct_online = online;

	if (have_pmic && pmic.ops->set_usbin_suspend) {
		/*
		 * 判据是"直充真的在出力"，而不是"直充被启用了"。
		 *
		 * direct_online 只等于 chg_en && op_mode == SWITCHING —— 那是芯片
		 * 内部状态，不保证有电流送出。低电量时实测直充报 SWITCHING、电池
		 * 却仍在放电；若据此挂起 PMIC，两条路同时不给电，插着 9V/3A 的
		 * 充电器电量照掉。所以这里读直充的实际输出电流，超过阈值才算它
		 * 接管了 VBUS，PMIC 才让位；否则 PMIC 保持开启兜底。
		 */
		int direct_i = 0;
		bool direct_delivering = false;

		if (direct_online && direct.ops->get_current_now &&
		    !direct.ops->get_current_now(direct.priv, &direct_i))
			direct_delivering = direct_i > QCOM_BMS_DC_MIN_DELIVER_UA;

		bool want_pmic_susp = direct_delivering;

		if (want_pmic_susp != info->pmic_usbin_suspended) {
			ret = pmic.ops->set_usbin_suspend(pmic.priv,
							  want_pmic_susp);
			if (ret)
				dev_warn(info->dev,
					 "failed to %s PMIC USBIN: %d\n",
					 want_pmic_susp ? "suspend" : "resume",
					 ret);
			else
				info->pmic_usbin_suspended = want_pmic_susp;
		}
	}

out:
	if (have_pmic)
		qcom_bms_charger_release(info, &pmic);
	if (have_direct)
		qcom_bms_charger_release(info, &direct);
	if (have_fg)
		qcom_bms_fg_release(info, &fg);
}

static void qcom_bms_status_work(struct work_struct *work)
{
	struct qcom_bms_info *info =
		container_of(work, struct qcom_bms_info, status_work);

	qcom_bms_update_status(info);
	qcom_bms_evaluate_direct_charge(info);
}

static void qcom_bms_refresh_authentication(struct qcom_bms_info *info)
{
	struct qcom_bms_charger_ref direct = {};
	struct qcom_bms_charger_ref pmic = {};
	u32 authenticated_mask = 0;
	bool have_direct = false;
	bool have_pmic = false;
	bool authenticated;
	bool changed;
	int i;
	int ret;

	mutex_lock(&info->ops_lock);
	for (i = 0; i < QCOM_BMS_AUTH_MAX; i++) {
		if (info->auth[i].dev && info->auth[i].authenticated)
			authenticated_mask |= BIT(i);
	}

	authenticated = (authenticated_mask & info->required_auth_mask) ==
			info->required_auth_mask;
	changed = info->authenticated != authenticated;
	info->authenticated = authenticated;
	if (changed) {
		have_pmic =
			qcom_bms_charger_acquire_locked(&info->pmic_chg, &pmic);
		have_direct = qcom_bms_charger_acquire_locked(&info->direct_chg,
							      &direct);
	}
	mutex_unlock(&info->ops_lock);

	if (!changed)
		return;

	if (have_pmic && pmic.ops->set_authenticated) {
		ret = pmic.ops->set_authenticated(pmic.priv, authenticated);
		if (ret)
			dev_warn(
				info->dev,
				"failed to update PMIC authentication state: %d\n",
				ret);
	}
	if (have_direct && direct.ops->set_authenticated) {
		ret = direct.ops->set_authenticated(direct.priv, authenticated);
		if (ret)
			dev_warn(
				info->dev,
				"failed to update direct charger authentication state: %d\n",
				ret);
	}

	if (have_direct)
		qcom_bms_charger_release(info, &direct);
	if (have_pmic)
		qcom_bms_charger_release(info, &pmic);

	dev_info(info->dev, "battery authentication %s (mask %#x/%#x)\n",
		 authenticated ? "passed" : "failed", authenticated_mask,
		 info->required_auth_mask);
	power_supply_changed(info->bms_psy);

	/* Authentication gates direct charge; re-evaluate on change. */
	queue_work(system_freezable_wq, &info->status_work);
}

int qcom_bms_register_fg(struct device *dev, const struct qcom_bms_fg_ops *ops,
			 void *priv)
{
	struct qcom_bms_info *info;
	int ret;

	if (!dev || !ops)
		return -EINVAL;

	info = qcom_bms_lock();
	if (!info)
		return -EPROBE_DEFER;

	if (info->fg.dev || info->fg.unregistering) {
		ret = -EBUSY;
		goto out_unlock;
	}

	info->fg.link = device_link_add(dev, info->dev, DL_FLAG_STATELESS);
	if (!info->fg.link) {
		dev_err(dev, "failed to link fuel gauge to BMS\n");
		ret = -EPROBE_DEFER;
		goto out_unlock;
	}

	info->fg.dev = dev;
	info->fg.ops = ops;
	info->fg.priv = priv;
	mutex_unlock(&info->ops_lock);

	qcom_bms_notify_changed(dev);
	return 0;

out_unlock:
	mutex_unlock(&info->ops_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_bms_register_fg);

int qcom_bms_register_charger(struct device *dev,
			      enum qcom_bms_charger_role role,
			      const struct qcom_bms_charger_ops *ops,
			      void *priv)
{
	struct qcom_bms_charger_ref ref = {};
	struct qcom_bms_info *info;
	struct qcom_bms_charger *charger;
	bool authenticated;
	int ret;

	if (!dev || !ops)
		return -EINVAL;
	if (role != QCOM_BMS_CHARGER_PMIC && role != QCOM_BMS_CHARGER_DIRECT)
		return -EINVAL;

	info = qcom_bms_lock();
	if (!info)
		return -EPROBE_DEFER;

	charger = role == QCOM_BMS_CHARGER_PMIC ? &info->pmic_chg :
						  &info->direct_chg;
	if (charger->dev || charger->unregistering) {
		ret = -EBUSY;
		goto out_unlock;
	}

	charger->link = device_link_add(dev, info->dev, DL_FLAG_STATELESS);
	if (!charger->link) {
		dev_err(dev, "failed to link charger to BMS\n");
		ret = -EPROBE_DEFER;
		goto out_unlock;
	}

	charger->dev = dev;
	charger->ops = ops;
	charger->priv = priv;
	authenticated = info->authenticated;
	qcom_bms_charger_acquire_locked(charger, &ref);
	mutex_unlock(&info->ops_lock);

	if (ref.ops->set_authenticated) {
		ret = ref.ops->set_authenticated(ref.priv, authenticated);
		if (ret)
			dev_warn(
				dev,
				"failed to set initial authentication state: %d\n",
				ret);
	}
	qcom_bms_charger_release(info, &ref);

	qcom_bms_notify_changed(dev);
	return 0;

out_unlock:
	mutex_unlock(&info->ops_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_bms_register_charger);

void qcom_bms_unregister_fg(struct device *dev)
{
	struct qcom_bms_info *info;
	struct device_link *link;

	info = qcom_bms_lock();
	if (!info)
		return;

	if (info->fg.dev != dev) {
		mutex_unlock(&info->ops_lock);
		return;
	}

	link = info->fg.link;
	info->fg.dev = NULL;
	info->fg.link = NULL;
	info->fg.ops = NULL;
	info->fg.priv = NULL;
	info->fg.unregistering = true;
	mutex_unlock(&info->ops_lock);

	wait_for_completion(&info->fg.idle);
	if (link)
		device_link_del(link);

	mutex_lock(&info->ops_lock);
	info->fg.unregistering = false;
	mutex_unlock(&info->ops_lock);

	queue_work(system_freezable_wq, &info->status_work);
}
EXPORT_SYMBOL_GPL(qcom_bms_unregister_fg);

void qcom_bms_unregister_charger(struct device *dev)
{
	struct qcom_bms_info *info;
	struct qcom_bms_charger *charger;
	struct device_link *link;

	info = qcom_bms_lock();
	if (!info)
		return;

	if (info->pmic_chg.dev == dev) {
		charger = &info->pmic_chg;
	} else if (info->direct_chg.dev == dev) {
		charger = &info->direct_chg;
	} else {
		mutex_unlock(&info->ops_lock);
		return;
	}

	link = charger->link;
	charger->dev = NULL;
	charger->link = NULL;
	charger->ops = NULL;
	charger->priv = NULL;
	charger->unregistering = true;
	mutex_unlock(&info->ops_lock);

	wait_for_completion(&charger->idle);
	if (link)
		device_link_del(link);

	mutex_lock(&info->ops_lock);
	charger->unregistering = false;
	mutex_unlock(&info->ops_lock);

	queue_work(system_freezable_wq, &info->status_work);
}
EXPORT_SYMBOL_GPL(qcom_bms_unregister_charger);

int qcom_bms_register_authenticator(struct device *dev,
				    enum qcom_bms_auth_role role)
{
	struct qcom_bms_authenticator *auth;
	struct qcom_bms_info *info;
	int ret;

	if (!dev || role < 0 || role >= QCOM_BMS_AUTH_MAX)
		return -EINVAL;

	info = qcom_bms_lock();
	if (!info)
		return -EPROBE_DEFER;

	auth = &info->auth[role];
	if (auth->dev) {
		ret = -EBUSY;
		goto out_unlock;
	}

	auth->link = device_link_add(dev, info->dev, DL_FLAG_STATELESS);
	if (!auth->link) {
		ret = -EPROBE_DEFER;
		goto out_unlock;
	}

	auth->dev = dev;
	mutex_unlock(&info->ops_lock);

	return 0;

out_unlock:
	mutex_unlock(&info->ops_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_bms_register_authenticator);

void qcom_bms_unregister_authenticator(struct device *dev)
{
	struct qcom_bms_info *info;
	struct device_link *link = NULL;
	int i;

	info = qcom_bms_lock();
	if (!info)
		return;

	for (i = 0; i < QCOM_BMS_AUTH_MAX; i++) {
		if (info->auth[i].dev != dev)
			continue;

		link = info->auth[i].link;
		memset(&info->auth[i], 0, sizeof(info->auth[i]));
		break;
	}
	mutex_unlock(&info->ops_lock);

	if (!link)
		return;

	device_link_del(link);
	qcom_bms_refresh_authentication(info);
}
EXPORT_SYMBOL_GPL(qcom_bms_unregister_authenticator);

void qcom_bms_authentication_changed(struct device *dev, bool authenticated)
{
	struct qcom_bms_info *info;
	int i;

	info = qcom_bms_lock();
	if (!info)
		return;

	for (i = 0; i < QCOM_BMS_AUTH_MAX; i++) {
		if (info->auth[i].dev != dev)
			continue;

		info->auth[i].authenticated = authenticated;
		break;
	}
	mutex_unlock(&info->ops_lock);

	if (i < QCOM_BMS_AUTH_MAX)
		qcom_bms_refresh_authentication(info);
}
EXPORT_SYMBOL_GPL(qcom_bms_authentication_changed);

void qcom_bms_notify_changed(struct device *dev)
{
	struct qcom_bms_info *info;
	bool registered;

	info = qcom_bms_lock();
	if (!info)
		return;

	registered = info->fg.dev == dev || info->pmic_chg.dev == dev ||
		     info->direct_chg.dev == dev;
	if (registered)
		queue_work(system_freezable_wq, &info->status_work);
	mutex_unlock(&info->ops_lock);
}
EXPORT_SYMBOL_GPL(qcom_bms_notify_changed);

static int qcom_bms_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = {};
	struct qcom_bms_info *info;
	int ret;

	mutex_lock(&qcom_bms_mutex);
	if (g_bms) {
		mutex_unlock(&qcom_bms_mutex);
		return -EBUSY;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		mutex_unlock(&qcom_bms_mutex);
		return -ENOMEM;
	}

	info->dev = &pdev->dev;
	info->status = POWER_SUPPLY_STATUS_UNKNOWN;
	/*
	 * PM8150B 上电时 USBIN 通路默认是挂起的。这里必须假定"已挂起"，
	 * 否则去重逻辑（"状态变化才写"）永远不会触发第一次 resume，
	 * PMIC 从头到尾不会被配置，input current limit 停在 0 —— 实测表现
	 * 为插着 9V/3A 充电器、icl=0、电池 -1.8A 放电。
	 *
	 * 反过来也不能无条件写：set_usbin_suspend() 内部会 mod_delayed_work，
	 * 本函数被 psy notifier 高频调用，无脑调用会把 I2C 打爆（usb_type
	 * 读回 EAGAIN、icl 在 0/250mA 之间震荡）。
	 *
	 * 初值 true + 现有去重，正好让第一次走 resume、之后不再重复。
	 */
	info->pmic_usbin_suspended = true;
	of_property_read_u32(pdev->dev.of_node, "qcom,required-authenticators",
			     &info->required_auth_mask);
	info->required_auth_mask &= GENMASK(QCOM_BMS_AUTH_MAX - 1, 0);
	info->authenticated = !info->required_auth_mask;
	mutex_init(&info->ops_lock);
	init_completion(&info->fg.idle);
	complete_all(&info->fg.idle);
	init_completion(&info->pmic_chg.idle);
	complete_all(&info->pmic_chg.idle);
	init_completion(&info->direct_chg.idle);
	complete_all(&info->direct_chg.idle);
	INIT_WORK(&info->status_work, qcom_bms_status_work);
	psy_cfg.drv_data = info;
	psy_cfg.fwnode = dev_fwnode(&pdev->dev);

	info->bms_psy = devm_power_supply_register(
		&pdev->dev, &qcom_bms_psy_desc, &psy_cfg);
	if (IS_ERR(info->bms_psy)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(info->bms_psy),
				    "failed to register %s\n",
				    QCOM_BMS_PSY_NAME);
		mutex_unlock(&qcom_bms_mutex);
		return ret;
	}

	info->usb_psy = devm_power_supply_register(
		&pdev->dev, &qcom_bms_usb_psy_desc, &psy_cfg);
	if (IS_ERR(info->usb_psy)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(info->usb_psy),
				    "failed to register %s\n",
				    QCOM_BMS_USB_PSY_NAME);
		mutex_unlock(&qcom_bms_mutex);
		return ret;
	}

	g_bms = info;
	platform_set_drvdata(pdev, info);
	mutex_unlock(&qcom_bms_mutex);

	dev_info(
		&pdev->dev,
		"Qualcomm BMS coordinator registered (required auth mask %#x)\n",
		info->required_auth_mask);

	/*
	 * Watch the tcpm source psy for CC-driven attach/detach - the real
	 * charger-connection signal (VBUS-based detection is fooled by the SC
	 * back-feeding VBUS to ~2*VBAT on unplug).  Resolved lazily; tcpm may
	 * probe after us.
	 */
	info->tcpm_np = of_parse_phandle(pdev->dev.of_node, "qcom,tcpm-psy", 0);
	if (!info->tcpm_np)
		dev_warn(
			&pdev->dev,
			"qcom,tcpm-psy not specified; direct charge will not engage\n");
	info->psy_nb.notifier_call = qcom_bms_psy_notifier;
	power_supply_reg_notifier(&info->psy_nb);

	return 0;
}

static void qcom_bms_remove(struct platform_device *pdev)
{
	struct qcom_bms_info *info;

	mutex_lock(&qcom_bms_mutex);
	info = g_bms;
	g_bms = NULL;
	mutex_unlock(&qcom_bms_mutex);

	power_supply_unreg_notifier(&info->psy_nb);
	cancel_work_sync(&info->status_work);
	if (info->tcpm_psy)
		power_supply_put(info->tcpm_psy);
	of_node_put(info->tcpm_np);
}

static int qcom_bms_resume(struct device *dev)
{
	struct qcom_bms_info *info = dev_get_drvdata(dev);

	/* Re-evaluate: charging state may have changed during sleep. */
	queue_work(system_freezable_wq, &info->status_work);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(qcom_bms_pm, NULL, qcom_bms_resume);

static const struct of_device_id qcom_bms_of_match[] = {
	{ .compatible = "qcom,pm8150b-bms" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_bms_of_match);

static struct platform_driver qcom_bms_driver = {
	.driver = {
		.name = "qcom-bms",
		.of_match_table = qcom_bms_of_match,
		.pm = pm_sleep_ptr(&qcom_bms_pm),
	},
	.probe = qcom_bms_probe,
	.remove = qcom_bms_remove,
};
module_platform_driver(qcom_bms_driver);

MODULE_AUTHOR("TwinbornPlate75 <3342733415@qq.com>");
MODULE_DESCRIPTION("Qualcomm Unified Battery/Charger Manager");
MODULE_LICENSE("GPL");
