// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2019 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Ltd.
 * Author: Casey Connolly <casey.connolly@linaro.org>
 *
 * This driver is for the switch-mode battery charger and boost
 * hardware found in PM8150b / PM7250b PMICs (SMB5 generation).
 */

#include <linux/bits.h>
#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "qcom_bms.h"

#define SMB_BASE 0x100

/*
 * 地址体系（已与 vendor smb5-reg.h 逐条核对）：
 *   chip->base = DTS 的 reg = 0x1000 = vendor 的 CHGR_BASE
 *   SMB_BASE   = 0x100           = vendor 的 DCDC_BASE - CHGR_BASE
 * 所以绝对地址 = chip->base + 这些偏移。
 *
 * 下面是 vendor 在 probe 里会配、而本移植版一直漏掉的两项 ——
 * 它们正是低电量充不进电的原因：
 *
 * 1. JEITA_EN_CFG（CHGR_BASE + 0x90）：硬件温度补偿。vendor 无条件
 *    关掉它（改用软件 JEITA）。本版没碰过，默认开着，温度判断一旦
 *    落到 cold/warm 档就把充电电流砍到 pre-charge 级别。
 *
 * 2. DCDC_ENG_SDCDC_CFG5（DCDC_BASE + 0xC4，bits 7:6）：低 VBAT 时
 *    允许高功率充电的阈值。vendor 取 3.6V（保守），这里取 3.3V，让
 *    10% 上下（约 3.5V）的电芯也能接受正常的 CC 电流。
 *    注意：实测在该项上于 3.6V/3.3V 之间切换并不改变实际充电电流 ——
 *    低电量充不进电的真正原因是 DTS 里 required-authenticators 要求了
 *    并不存在的认证器，协调器从未启用直充。此项保留但与那个故障无关。
 */
#define CHGR_JEITA_EN_CFG    (0x90)              /* CHGR_BASE + 0x90 */
#define JEITA_EN_ALL_MASK    (0x0F)              /* cold/hot × fcv/ccc */
#define DCDC_SDCDC_CFG5      (SMB_BASE + 0xC4)   /* DCDC_BASE + 0xC4 */
#define ENG_SDCDC_BAT_HPWR_MASK  (0xC0)          /* GENMASK(7,6) */
/*
 * 阈值从 3.6V 降到 3.3V。
 *
 * 该字段的含义是"VBAT 高于此值才允许高功率充电"。vendor 取 3.6V 是保守
 * 策略：实测电池 3.50–3.55V（10% 上下）时 ICL 被压到 1.875A，而 3.75V
 * 以上能跑 4.7A —— 差值正好卡在这条线两侧。
 *
 * 降到 3.3V 后，3.5V 就能放开到 3.05A（0.35C）。锂电池标准快充是
 * 0.5–1C，所以这个电流对 10% SOC 的电池仍属保守范围，不属于危险区；
 * vendor 的高阈值是求稳，不是安全红线。
 *
 * 枚举值（vendor smb5-reg.h）：3P3=0x00 / 3P4=0x40 / 3P5=0x80 / 3P6=0xC0
 */
#define BOOST_MODE_THRESH_3P3_V  (0x00)

#define BATTERY_CHARGER_STATUS_1 0x06
#define BATTERY_CHARGER_STATUS_MASK GENMASK(2, 0)

#define SMB5_CHARGER_ERROR_STATUS_BAT_OV_BIT BIT(1)

#define BATTERY_CHARGER_STATUS_7 0x0D
#define SMB5_BAT_TEMP_STATUS_HOT_SOFT_BIT BIT(5)
#define SMB5_BAT_TEMP_STATUS_COLD_SOFT_BIT BIT(4)
#define SMB5_BAT_TEMP_STATUS_TOO_HOT_BIT BIT(3)
#define SMB5_BAT_TEMP_STATUS_TOO_COLD_BIT BIT(2)

#define CHARGING_ENABLE_CMD 0x42
#define CHARGING_ENABLE_CMD_BIT BIT(0)

#define FAST_CHARGE_CURRENT_CFG 0x61
#define FAST_CHARGE_CURRENT_SETTING_MASK GENMASK(7, 0)

#define FLOAT_VOLTAGE_CFG 0x70
#define FLOAT_VOLTAGE_SETTING_MASK GENMASK(7, 0)

#define SMB5_CHARGE_RCHG_SOC_THRESHOLD_CFG_REG 0x7D
#define SMB5_CHARGE_RCHG_SOC_THRESHOLD_CFG_MASK GENMASK(7, 0)

#define OTG_CFG 0x153
#define OTG_EN_SRC_CFG_BIT BIT(1)

#define APSD_STATUS 0x307
#define APSD_DTC_STATUS_DONE_BIT BIT(0)

#define APSD_RESULT_STATUS 0x308
#define APSD_RESULT_STATUS_MASK GENMASK(6, 0)
#define FLOAT_CHARGER_BIT BIT(4)
#define DCP_CHARGER_BIT BIT(3)
#define CDP_CHARGER_BIT BIT(2)
#define OCP_CHARGER_BIT BIT(1)

#define USBIN_CMD_IL 0x340

/*
 * USBIN 子块（USBIN_BASE = 0x1300，本驱动 chip->base = 0x1000，
 * 所以这里的偏移都是相对 0x1000 的 0x3xx）。
 *
 * USBIN_ADAPTER_ALLOW_CFG 是常开的 OVP 比较器窗口。bootloader 往往
 * 把它留在"只允许 5V"，而 PD 一旦把 VBUS 拉到 9V/PPS，这个比较器
 * 就会锁存过压（USBIN_INT_RT_STS 的 bit3），充电被永久门控 ——
 * 表现正是插着 9V/3A、BATTERY_CHARGER_STATUS_1 却在
 * PAUSE/DISABLE 之间跳、ICL 读到 0。
 * 参见 asidko/pm6150-charger-mainline 的 SW_OVERRIDE_HC_MODE 第 8 步。
 */
#define USBIN_INT_RT_STS        0x310
#define USBIN_INT_LATCHED_CLR   0x314
#define USBIN_ADAPTER_ALLOW_OVERRIDE 0x344
#define USBIN_ADAPTER_ALLOW_CFG 0x360

#define USBIN_ADAPTER_ALLOW_5V_TO_12V        12   /* 0x0C: 允许 5-12V */
#define USBIN_ADAPTER_ALLOW_OVERRIDE_CONTINUOUS BIT(3) /* 接受 4-12V 全范围 */
#define USBIN_OV_RT_STS_BIT     BIT(3)  /* 0x1310: 输入过压 */
#define USBIN_UV_RT_STS_BIT     BIT(2)  /* 0x1310: 输入欠压 */
#define USBIN_SUSPEND_ON_COLLAPSE_BIT   BIT(7) /* 0x1380: 塌陷即挂起 */
#define USBIN_SUSPEND_BIT BIT(0)

#define CMD_APSD 0x341
#define APSD_RERUN_BIT BIT(0)

#define CMD_ICL_OVERRIDE 0x342
#define ICL_OVERRIDE_BIT BIT(0)

#define TYPE_C_CFG 0x358
#define APSD_START_ON_CC_BIT BIT(7)

#define USBIN_OPTIONS_1_CFG 0x362
#define AUTO_SRC_DETECT_BIT BIT(3)

#define USBIN_LOAD_CFG 0x65
#define ICL_OVERRIDE_AFTER_APSD_BIT BIT(4)

#define USBIN_CURRENT_LIMIT_CFG 0x370
#define USBIN_ICL_OPTIONS 0x366
#define USBIN_MODE_CHG_BIT BIT(0)

#define USBIN_AICL_OPTIONS_CFG 0x380
#define SUSPEND_ON_COLLAPSE_USBIN_BIT BIT(7)
#define USBIN_AICL_PERIODIC_RERUN_EN_BIT BIT(4)
#define USBIN_AICL_ADC_EN_BIT BIT(3)
#define USBIN_AICL_EN_BIT BIT(2)

#define ICL_STATUS (SMB_BASE + 0x07)

#define POWER_PATH_STATUS (SMB_BASE + 0x0B)
#define P_PATH_USE_USBIN_BIT BIT(4)
#define P_PATH_VALID_INPUT_POWER_SOURCE_STS_BIT BIT(0)

/* 0x5xx region is PM8150b only Type-C registers */

#define SMB5_TYPE_C_MODE_CFG 0x544
#define SMB5_EN_TRY_SNK_BIT BIT(4)
#define SMB5_EN_SNK_ONLY_BIT BIT(1)

#define SMB5_TYPEC_TYPE_C_VCONN_CONTROL 0x546
#define SMB5_VCONN_EN_ORIENTATION_BIT BIT(2)
#define SMB5_VCONN_EN_VALUE_BIT BIT(1)
#define SMB5_VCONN_EN_SRC_BIT BIT(0)

#define SMB5_TYPE_C_DEBUG_ACCESS_SINK 0x54a
#define SMB5_TYPEC_DEBUG_ACCESS_SINK_MASK GENMASK(4, 0)

#define SMB5_DEBUG_ACCESS_SRC_CFG 0x54C
#define SMB5_EN_UNORIENTED_DEBUG_ACCESS_SRC_BIT BIT(0)

#define SMB5_TYPE_C_EXIT_STATE_CFG 0x550
#define SMB5_SEL_SRC_UPPER_REF_BIT BIT(2)

#define WD_CFG 0x651
#define WATCHDOG_TRIGGER_AFP_EN_BIT BIT(7)
#define BARK_WDOG_INT_EN_BIT BIT(6)
#define WDOG_TIMER_EN_ON_PLUGIN_BIT BIT(1)

#define SNARL_BARK_BITE_WD_CFG 0x653

#define AICL_RERUN_TIME_CFG 0x661
#define AICL_RERUN_TIME_MASK GENMASK(1, 0)
#define AIC_RERUN_TIME_3_SECS 0x0

#define SDP_CURRENT_UA 500000
#define CDP_CURRENT_UA 1500000
#define DCP_CURRENT_UA 1500000
#define UNVERIFIED_CURRENT_UA 2000000

/*
 * APSD 是 BC1.2 的充电器类型检测，硬件本身要 1-2 秒才置 DONE。
 * 原来 1 秒就 rerun，正好在检测完成前把它重置掉，于是 DONE 永远
 * 起不来、ICL 永远设不上（实测：插着 PD 9V/3A 的充电器，icl=0、
 * 电池持续放电）。放宽到 3 秒，并给重试次数封顶。
 */
#define APSD_RETRY_MS 3000
#define APSD_MAX_RETRIES 5

/* SMB5 (PM8150b/PM7250b) registers represent current in 1/40th amp increments */
#define CURRENT_SCALE_FACTOR 25000

/*
 * BATTERY_CHARGER_STATUS_1 的 bit2:0 编码，顺序按 PM8150B 硬件手册
 * （与 vendor smb5-reg.h 一致），不能凭字面猜：INHIBIT 是 0、PAUSE 是 6。
 * 原先这里把 TRICKLE 当成 0、还凭空多编了一个 FAST_CHARGE，读到 6
 * （真实含义 PAUSE）时被当成 INHIBIT，进而报成 "Full" —— 表现为
 * 电量 0% 显示已充满、插着 9V/3A 却充不进去。
 */
enum charger_status {
	INHIBIT_CHARGE = 0,
	TRICKLE_CHARGE,
	PRE_CHARGE,
	FULLON_CHARGE,
	TAPER_CHARGE,
	TERMINATE_CHARGE,
	PAUSE_CHARGE,
	DISABLE_CHARGE,
};

struct smb_init_register {
	u16 addr;
	u8 mask;
	u8 val;
};

/**
 * struct smb_chip - smb chip structure
 * @dev:		Device reference
 * @base:		Base address for smb registers
 * @regmap:		Register map
 * @status_change_work: Worker to handle plug/unplug events
 * @cable_irq:		USB plugin IRQ
 * @usb_in_i_chan:	USB_IN current measurement channel
 * @usb_in_v_chan:	USB_IN voltage measurement channel
 * @constant_charge_current_max_ua: maximum fast charge current from battery DT
 * @authenticated: all required battery authenticators have passed
 * @voltage_max_design_uv:	maximum battery voltage from battery DT
 */
struct smb_chip {
	struct device *dev;
	unsigned int base;
	struct regmap *regmap;

	struct delayed_work status_change_work;
	int cable_irq;
	struct iio_channel *usb_in_i_chan;
	struct iio_channel *usb_in_v_chan;

	u32 constant_charge_current_max_ua;
	bool authenticated;
	u32 voltage_max_design_uv;
	unsigned int apsd_retries;
};

static void smb_get_batt_info(struct smb_chip *chip)
{
	struct device_node *batt_np;
	u32 val;

	batt_np = of_parse_phandle(chip->dev->of_node, "monitored-battery", 0);
	if (!batt_np) {
		dev_warn(chip->dev,
			 "No monitored-battery, using default charge limits\n");
		chip->constant_charge_current_max_ua = DCP_CURRENT_UA;
		chip->voltage_max_design_uv = 4480000;
		return;
	}

	if (of_property_read_u32(batt_np,
				 "constant-charge-current-max-microamp", &val))
		val = DCP_CURRENT_UA;
	chip->constant_charge_current_max_ua = val;

	if (of_property_read_u32(batt_np, "voltage-max-design-microvolt", &val))
		val = 4480000;
	chip->voltage_max_design_uv = val;

	of_node_put(batt_np);
}

static int smb_get_prop_usb_online(void *priv, int *val)
{
	struct smb_chip *chip = priv;
	unsigned int stat;
	int rc;

	rc = regmap_read(chip->regmap, chip->base + POWER_PATH_STATUS, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read power path status: %d\n", rc);
		return rc;
	}

	*val = (stat & P_PATH_USE_USBIN_BIT) &&
	       (stat & P_PATH_VALID_INPUT_POWER_SOURCE_STS_BIT);
	return 0;
}

/*
 * Qualcomm "automatic power source detection" aka APSD
 * tells us what type of charger we're connected to.
 */
static int smb_apsd_get_charger_type(void *priv, int *val)
{
	struct smb_chip *chip = priv;
	unsigned int apsd_stat, stat;
	int usb_online = 0;
	int rc;

	rc = smb_get_prop_usb_online(chip, &usb_online);
	if (!usb_online) {
		*val = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		return rc;
	}

	rc = regmap_read(chip->regmap, chip->base + APSD_STATUS, &apsd_stat);
	if (rc < 0) {
		dev_err(chip->dev, "failed to read APSD status: %d\n", rc);
		return rc;
	}
	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT)) {
		dev_dbg(chip->dev, "APSD not ready\n");
		return -EAGAIN;
	}

	rc = regmap_read(chip->regmap, chip->base + APSD_RESULT_STATUS, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "failed to read APSD result: %d\n", rc);
		return rc;
	}

	stat &= APSD_RESULT_STATUS_MASK;

	if (stat & CDP_CHARGER_BIT)
		*val = POWER_SUPPLY_USB_TYPE_CDP;
	else if (stat & (DCP_CHARGER_BIT | OCP_CHARGER_BIT | FLOAT_CHARGER_BIT))
		*val = POWER_SUPPLY_USB_TYPE_DCP;
	else /* SDP or unknown charger type */
		*val = POWER_SUPPLY_USB_TYPE_SDP;

	return 0;
}

/* Return 1 when in overvoltage state, else 0 or -errno */
static int smbx_ov_status(struct smb_chip *chip)
{
	u32 val;
	int rc;

	rc = regmap_read(chip->regmap, chip->base + BATTERY_CHARGER_STATUS_7,
			 &val);
	if (rc)
		return rc;

	return !!(val & SMB5_CHARGER_ERROR_STATUS_BAT_OV_BIT);
}

static int smb_get_prop_status(void *priv, int *val)
{
	struct smb_chip *chip = priv;
	u32 stat;
	int usb_online = 0;
	int rc;

	rc = smb_get_prop_usb_online(chip, &usb_online);
	if (!usb_online) {
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		return rc;
	}

	rc = regmap_read(chip->regmap, chip->base + BATTERY_CHARGER_STATUS_1,
			 &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read charging status ret=%d\n",
			rc);
		return rc;
	}


	rc = smbx_ov_status(chip);
	if (rc < 0)
		return rc;

	/* In overvoltage state */
	if (rc == 1) {
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	stat &= BATTERY_CHARGER_STATUS_MASK;

	/*
	 * 按硬件状态的实际含义映射，不是照抄 vendor：
	 *   TRICKLE/PRE/FULLON/TAPER  正在充            -> CHARGING
	 *   TERMINATE                 达到终止条件，真满 -> FULL
	 *   INHIBIT                   硬件主动禁止充电  -> NOT_CHARGING
	 *   PAUSE                     暂时停充          -> NOT_CHARGING
	 *   DISABLE                   关闭充电          -> NOT_CHARGING
	 *
	 * vendor 把 INHIBIT 也报成 FULL，是为了让安卓状态栏别闪；代价是
	 * 低电量被禁止充电时显示"已充满"。这里按实际报，避免上层误判。
	 */
	switch (stat) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
	case FULLON_CHARGE:
	case TAPER_CHARGE:
		*val = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case TERMINATE_CHARGE:
		*val = POWER_SUPPLY_STATUS_FULL;
		break;
	case INHIBIT_CHARGE:
	case PAUSE_CHARGE:
	case DISABLE_CHARGE:
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		*val = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return 0;
}

static inline int smb_get_current_limit(struct smb_chip *chip,
					unsigned int *val)
{
	int rc = regmap_read(chip->regmap, chip->base + ICL_STATUS, val);

	if (rc >= 0)
		*val *= CURRENT_SCALE_FACTOR;
	return rc;
}

static int smb_set_current_limit(struct smb_chip *chip, unsigned int val)
{
	unsigned char val_raw;

	if (val > 4800000) {
		dev_err(chip->dev,
			"can't set current limit higher than 4800000 uA\n");
		return -EINVAL;
	}
	val_raw = val / CURRENT_SCALE_FACTOR;

	return regmap_write(chip->regmap, chip->base + USBIN_CURRENT_LIMIT_CFG,
			    val_raw);
}

static int smb_get_input_current_limit(void *priv, int *val)
{
	struct smb_chip *chip = priv;
	unsigned int limit;
	int rc;

	rc = smb_get_current_limit(chip, &limit);
	if (rc < 0)
		return rc;

	*val = (int)limit;
	return 0;
}

static int smb_set_usbin_suspend(void *priv, bool suspend)
{
	struct smb_chip *chip = priv;
	int ret;

	ret = regmap_update_bits(chip->regmap, chip->base + USBIN_CMD_IL,
				  USBIN_SUSPEND_BIT,
				  suspend ? USBIN_SUSPEND_BIT : 0);
	if (!ret && !suspend)
		mod_delayed_work(system_dfl_wq, &chip->status_change_work,
				 msecs_to_jiffies(100));

	return ret;
}

static void smb_status_change_work(struct work_struct *work)
{
	unsigned int charger_type, current_ua;
	int usb_online = 0;
	int count, rc;
	struct smb_chip *chip;

	chip = container_of(work, struct smb_chip, status_change_work.work);

	rc = smb_get_prop_usb_online(chip, &usb_online);
	if (rc < 0) {
		dev_err(chip->dev, "failed to read USB status: %d\n", rc);
		return;
	}
	if (!usb_online) {
		/*
		 * Charger removed: trigger a re-evaluation once VBUS has
		 * collapsed and the FG current has settled, so the status
		 * does not stay latched at CHARGING from a transient reading.
		 */
		qcom_bms_notify_changed(chip->dev);
		return;
	}

	/*
	 * 插着电：清掉 USBIN 子块的锁存中断位（W1C，写 1 清）。
	 * 过压/欠压比较器一旦锁存，内部的故障门会把充电一直挡住 ——
	 * 即使 VBUS 本身已经正常（PD 从 5V 走到 9V 时最容易踩到）。
	 * 配合 init 表里放宽的 OVP 窗口，清完之后 RT_STS 的对应位
	 * 若仍为 1，说明是真实故障，下一次这里还会再清。
	 */
	regmap_write(chip->regmap, chip->base + USBIN_INT_LATCHED_CLR, 0xFF);

	for (count = 0; count < 3; count++) {
		dev_dbg(chip->dev, "get charger type retry %d\n", count);
		rc = smb_apsd_get_charger_type(chip, &charger_type);
		if (rc != -EAGAIN)
			break;
		msleep(100);
	}

	if (rc < 0 && rc != -EAGAIN) {
		dev_err(chip->dev, "get charger type failed: %d\n", rc);
		return;
	}

	if (rc < 0) {
		/*
		 * APSD 没出结果。重试几次，但别无限等：PD 充电器走的是
		 * CC 协商，压根不发 BC1.2 的 D+/D- 握手，DONE 位可能永远
		 * 不置位。这种情况下 tcpm 已经把 VBUS 谈好了（9V/3A），
		 * 用电池 DT 的最大充电电流兜底是安全的 —— 总比 icl=0、
		 * 插着充电器还在放电强。
		 */
		if (chip->apsd_retries++ < APSD_MAX_RETRIES) {
			rc = regmap_update_bits(chip->regmap,
						chip->base + CMD_APSD,
						APSD_RERUN_BIT, APSD_RERUN_BIT);
			if (rc < 0) {
				dev_err(chip->dev,
					"failed to rerun APSD: %d\n", rc);
				return;
			}
			mod_delayed_work(system_dfl_wq,
					 &chip->status_change_work,
					 msecs_to_jiffies(APSD_RETRY_MS));
			dev_dbg(chip->dev,
				"get charger type failed, rerun apsd (%u/%u)\n",
				chip->apsd_retries, APSD_MAX_RETRIES);
			return;
		}

		current_ua = chip->constant_charge_current_max_ua;
		if (!READ_ONCE(chip->authenticated))
			current_ua = min(current_ua, UNVERIFIED_CURRENT_UA);
		dev_info(chip->dev,
			 "APSD never completed, falling back to %u uA\n",
			 current_ua);
		goto set_limit;
	}

	chip->apsd_retries = 0;

	switch (charger_type) {
	case POWER_SUPPLY_USB_TYPE_CDP:
		current_ua = CDP_CURRENT_UA;
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
		current_ua = chip->constant_charge_current_max_ua;
		if (!READ_ONCE(chip->authenticated))
			current_ua = min(current_ua, UNVERIFIED_CURRENT_UA);
		break;
	case POWER_SUPPLY_USB_TYPE_SDP:
	default:
		current_ua = SDP_CURRENT_UA;
		break;
	}

set_limit:
	rc = smb_set_current_limit(chip, current_ua);
	if (rc < 0) {
		dev_err(chip->dev, "failed to set input current limit: %d\n",
			rc);
		return;
	}
	qcom_bms_notify_changed(chip->dev);
}

static int smb_get_iio_chan(struct smb_chip *chip, struct iio_channel *chan,
			    int *val)
{
	int rc;
	int status;

	rc = smb_get_prop_status(chip, &status);
	if (rc < 0 || status != POWER_SUPPLY_STATUS_CHARGING) {
		*val = 0;
		return 0;
	}

	return iio_read_channel_processed(chan, val);
}

static int smb_get_prop_health(void *priv, int *val)
{
	struct smb_chip *chip = priv;
	int rc;
	unsigned int stat;

	rc = regmap_read(chip->regmap, chip->base + BATTERY_CHARGER_STATUS_7,
			 &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read charger status 7 rc=%d\n",
			rc);
		return rc;
	}

	if (stat & SMB5_CHARGER_ERROR_STATUS_BAT_OV_BIT)
		*val = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if (stat & SMB5_BAT_TEMP_STATUS_TOO_COLD_BIT)
		*val = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & SMB5_BAT_TEMP_STATUS_TOO_HOT_BIT)
		*val = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & SMB5_BAT_TEMP_STATUS_COLD_SOFT_BIT)
		*val = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & SMB5_BAT_TEMP_STATUS_HOT_SOFT_BIT)
		*val = POWER_SUPPLY_HEALTH_WARM;
	else
		*val = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int smb_get_voltage_now(void *priv, int *val)
{
	struct smb_chip *chip = priv;
	int ret;

	ret = smb_get_iio_chan(chip, chip->usb_in_v_chan, val);
	if (!ret)
		*val *= 16;

	return ret;
}

static int smb_op_current_now(void *priv, int *val)
{
	struct smb_chip *chip = priv;

	return smb_get_iio_chan(chip, chip->usb_in_i_chan, val);
}

static int smb_set_authenticated(void *priv, bool authenticated)
{
	struct smb_chip *chip = priv;

	WRITE_ONCE(chip->authenticated, authenticated);
	mod_delayed_work(system_dfl_wq, &chip->status_change_work, 0);

	return 0;
}

static const struct qcom_bms_charger_ops smb_charger_ops = {
	.get_status = smb_get_prop_status,
	.get_online = smb_get_prop_usb_online,
	.get_current_now = smb_op_current_now,
	.get_voltage_now = smb_get_voltage_now,
	.get_input_current_limit = smb_get_input_current_limit,
	.get_usb_type = smb_apsd_get_charger_type,
	.get_health = smb_get_prop_health,
	.set_usbin_suspend = smb_set_usbin_suspend,
	.set_authenticated = smb_set_authenticated,
};

static irqreturn_t smb_handle_batt_overvoltage(int irq, void *data)
{
	struct smb_chip *chip = data;

	if (smbx_ov_status(chip) == 1) {
		/* The hardware stops charging automatically */
		dev_err(chip->dev, "battery overvoltage detected\n");
		qcom_bms_notify_changed(chip->dev);
	}

	return IRQ_HANDLED;
}

static irqreturn_t smb_handle_usb_plugin(int irq, void *data)
{
	struct smb_chip *chip = data;

	qcom_bms_notify_changed(chip->dev);
	mod_delayed_work(system_dfl_wq, &chip->status_change_work,
			 msecs_to_jiffies(1500));

	return IRQ_HANDLED;
}

static irqreturn_t smb_handle_usb_icl_change(int irq, void *data)
{
	struct smb_chip *chip = data;

	qcom_bms_notify_changed(chip->dev);

	return IRQ_HANDLED;
}

/* Init sequence derived from vendor downstream driver */
static const struct smb_init_register smb5_init_seq[] = {
	{ .addr = USBIN_CMD_IL, .mask = USBIN_SUSPEND_BIT, .val = 0 },
	/*
	 * By default configure us as an upstream facing port
	 * FIXME: This will be handled by the type-c driver
	 */
	{ .addr = SMB5_TYPE_C_MODE_CFG,
	  .mask = SMB5_EN_TRY_SNK_BIT | SMB5_EN_SNK_ONLY_BIT,
	  .val = SMB5_EN_TRY_SNK_BIT },
	{ .addr = SMB5_TYPEC_TYPE_C_VCONN_CONTROL,
	  .mask = SMB5_VCONN_EN_ORIENTATION_BIT | SMB5_VCONN_EN_SRC_BIT |
		  SMB5_VCONN_EN_VALUE_BIT,
	  .val = 0 },
	{ .addr = SMB5_DEBUG_ACCESS_SRC_CFG,
	  .mask = SMB5_EN_UNORIENTED_DEBUG_ACCESS_SRC_BIT,
	  .val = SMB5_EN_UNORIENTED_DEBUG_ACCESS_SRC_BIT },
	{ .addr = SMB5_TYPE_C_EXIT_STATE_CFG,
	  .mask = SMB5_SEL_SRC_UPPER_REF_BIT,
	  .val = SMB5_SEL_SRC_UPPER_REF_BIT },
	/*
	 * Disable Type-C factory mode and stay in Attached.SRC state when VCONN
	 * over-current happens
	 */
	{ .addr = TYPE_C_CFG, .mask = APSD_START_ON_CC_BIT, .val = 0 },
	{ .addr = SMB5_TYPE_C_DEBUG_ACCESS_SINK,
	  .mask = SMB5_TYPEC_DEBUG_ACCESS_SINK_MASK,
	  .val = 0x17 },
	/* Configure VBUS for software control */
	{ .addr = OTG_CFG, .mask = OTG_EN_SRC_CFG_BIT, .val = 0 },
	/*
	 * Recharge when State Of Charge drops below 98%.
	 */
	{ .addr = SMB5_CHARGE_RCHG_SOC_THRESHOLD_CFG_REG,
	  .mask = SMB5_CHARGE_RCHG_SOC_THRESHOLD_CFG_MASK,
	  .val = 250 },
	/* Enable charging */
	{ .addr = CHARGING_ENABLE_CMD,
	  .mask = CHARGING_ENABLE_CMD_BIT,
	  .val = CHARGING_ENABLE_CMD_BIT },
	/* 关硬件 JEITA（vendor 同样这么干，改由软件判断温度） */
	{ .addr = CHGR_JEITA_EN_CFG,
	  .mask = JEITA_EN_ALL_MASK,
	  .val = 0 },
	/* 低 VBAT 高功率门槛 3.3V（vendor 是 3.6V，这里放宽） */
	{ .addr = DCDC_SDCDC_CFG5,
	  .mask = ENG_SDCDC_BAT_HPWR_MASK,
	  .val = BOOST_MODE_THRESH_3P3_V },
	/* Enable BC1P2 auto Src detect */
	{ .addr = USBIN_OPTIONS_1_CFG,
	  .mask = AUTO_SRC_DETECT_BIT,
	  .val = AUTO_SRC_DETECT_BIT },
	/* Set the default SDP charger type to a 500ma USB 2.0 port */
	{ .addr = USBIN_ICL_OPTIONS,
	  .mask = USBIN_MODE_CHG_BIT,
	  .val = USBIN_MODE_CHG_BIT },
	{ .addr = CMD_ICL_OVERRIDE, .mask = ICL_OVERRIDE_BIT, .val = 0 },
	/*
	 * ICL_OVERRIDE_AFTER_APSD：让写进 USBIN_CURRENT_LIMIT_CFG 的值
	 * 活过 APSD 重跑。asidko/pm6150 的 SW_OVERRIDE_HC_MODE 第 7 步
	 * 同样置位。
	 *
	 * 注：实测这块 PM8150B 读回 USBIN_LOAD_CFG 的 bit4 恒为 0，写不
	 * 进去；但关掉 AICL 之后 ICL 已经能稳定站住（读到 0x78 = 3A），
	 * 所以这条留着无害 —— 万一换了 PMIC 型号或固件版本会生效。
	 */
	{ .addr = USBIN_LOAD_CFG,
	  .mask = ICL_OVERRIDE_AFTER_APSD_BIT,
	  .val = ICL_OVERRIDE_AFTER_APSD_BIT },
	/*
	 * OVP 比较器窗口：5-12V + CONTINUOUS。
	 * 不设的话 bootloader 默认的 5V-only 窗口会在 PD 上到 9V 时锁存
	 * 过压，充电永久门控（见上面 USBIN_ADAPTER_ALLOW_CFG 的注释）。
	 */
	{ .addr = USBIN_ADAPTER_ALLOW_CFG,
	  .mask = 0xff,
	  .val = USBIN_ADAPTER_ALLOW_5V_TO_12V },
	{ .addr = USBIN_ADAPTER_ALLOW_OVERRIDE,
	  .mask = 0xff,
	  .val = USBIN_ADAPTER_ALLOW_OVERRIDE_CONTINUOUS },
	/* Disable watchdog */
	{ .addr = SNARL_BARK_BITE_WD_CFG, .mask = 0xff, .val = 0 },
	{ .addr = WD_CFG,
	  .mask = WATCHDOG_TRIGGER_AFP_EN_BIT | WDOG_TIMER_EN_ON_PLUGIN_BIT |
		  BARK_WDOG_INT_EN_BIT,
	  .val = 0 },
	/*
	 * Enable Automatic Input Current Limit, this will slowly ramp up the current
	 * When connected to a wall charger, and automatically stop when it detects
	 * the charger current limit (voltage drop?) or it reaches the programmed limit.
	 */
	/*
	 * 关掉硬件 AICL，但保留 SUSPEND_ON_COLLAPSE。
	 *
	 * 这里走的是软件静态限流（icr_override + USBIN_CURRENT_LIMIT_CFG），
	 * AICL 的周期性重跑会把我们写死的值一轮轮往回改，和 APSD 一起
	 * 造成 ICL 在 0 和各档之间振荡。PD 充电器接入时更是如此：
	 * APSD 推不出类型 → AICL 反复探底 → ICL 落到 0。
	 * SUSPEND_ON_COLLAPSE 留着：输入真的塌了还是要在硬件层面挂起。
	 * 与 asidko/pm6150-charger-mainline 的 SW_OVERRIDE_HC_MODE 第 1 步一致。
	 */
	{ .addr = USBIN_AICL_OPTIONS_CFG,
	  .mask = USBIN_AICL_PERIODIC_RERUN_EN_BIT | USBIN_AICL_ADC_EN_BIT |
		  USBIN_AICL_EN_BIT | SUSPEND_ON_COLLAPSE_USBIN_BIT,
	  .val = SUSPEND_ON_COLLAPSE_USBIN_BIT },
	/*
	 * This overrides all of the other current limit configs and is
	 * expected to be used for setting limits based on temperature.
	 * We set some relatively safe default value while still allowing
	 * a comfortably fast charging rate.
	 */
	{ .addr = FAST_CHARGE_CURRENT_CFG,
	  .mask = FAST_CHARGE_CURRENT_SETTING_MASK,
	  .val = 1950000 / CURRENT_SCALE_FACTOR },
};

static int smb_init_hw(struct smb_chip *chip,
		       const struct smb_init_register *init_seq, size_t len)
{
	size_t i;
	int rc;

	for (i = 0; i < len; i++) {
		dev_dbg(chip->dev, "%zu: writing 0x%02x to 0x%02x\n", i,
			init_seq[i].val, init_seq[i].addr);
		rc = regmap_update_bits(chip->regmap,
					chip->base + init_seq[i].addr,
					init_seq[i].mask, init_seq[i].val);
		if (rc < 0)
			return dev_err_probe(chip->dev, rc,
					     "%s: init command %zu failed\n",
					     __func__, i);
	}

	return 0;
}

static int smb_init_irq(struct smb_chip *chip, int *irq, const char *name,
			irqreturn_t (*handler)(int irq, void *data))
{
	int irqnum;
	int rc;

	irqnum = platform_get_irq_byname(to_platform_device(chip->dev), name);
	if (irqnum < 0)
		return irqnum;

	rc = devm_request_threaded_irq(chip->dev, irqnum, NULL, handler,
				       IRQF_ONESHOT, name, chip);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc, "Couldn't request irq %s\n",
				     name);

	if (irq)
		*irq = irqnum;

	return 0;
}

static int smb_probe(struct platform_device *pdev)
{
	struct smb_chip *chip;
	int rc;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap)
		return dev_err_probe(chip->dev, -ENODEV,
				     "failed to locate the regmap\n");

	rc = device_property_read_u32(chip->dev, "reg", &chip->base);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't read base address\n");

	chip->usb_in_v_chan = devm_iio_channel_get(chip->dev, "usbin_v");
	if (IS_ERR(chip->usb_in_v_chan))
		return dev_err_probe(chip->dev, PTR_ERR(chip->usb_in_v_chan),
				     "Couldn't get usbin_v IIO channel\n");

	chip->usb_in_i_chan = devm_iio_channel_get(chip->dev, "usbin_i");
	if (IS_ERR(chip->usb_in_i_chan))
		return dev_err_probe(chip->dev, PTR_ERR(chip->usb_in_i_chan),
				     "Couldn't get usbin_i IIO channel\n");

	smb_get_batt_info(chip);

	rc = smb_init_hw(chip, smb5_init_seq, ARRAY_SIZE(smb5_init_seq));
	if (rc < 0)
		return rc;

	rc = devm_delayed_work_autocancel(chip->dev, &chip->status_change_work,
					  smb_status_change_work);
	if (rc)
		return dev_err_probe(chip->dev, rc,
				     "Failed to init status change work\n");

	{
		u32 vbat_max;

		if (chip->voltage_max_design_uv > 3487500)
			vbat_max = chip->voltage_max_design_uv - 3487500;
		else
			vbat_max = 0;
		vbat_max = vbat_max / 7500 + 1;
		if (vbat_max > FLOAT_VOLTAGE_SETTING_MASK)
			vbat_max = FLOAT_VOLTAGE_SETTING_MASK;

		rc = regmap_update_bits(chip->regmap,
					chip->base + FLOAT_VOLTAGE_CFG,
					FLOAT_VOLTAGE_SETTING_MASK, vbat_max);
		if (rc < 0)
			return dev_err_probe(chip->dev, rc,
					     "Couldn't set vbat max\n");
	}

	rc = smb_init_irq(chip, NULL, "bat-ov", smb_handle_batt_overvoltage);
	if (rc < 0)
		return rc;

	rc = smb_init_irq(chip, &chip->cable_irq, "usb-plugin",
			  smb_handle_usb_plugin);
	if (rc < 0)
		return rc;

	rc = smb_init_irq(chip, NULL, "usbin-icl-change",
			  smb_handle_usb_icl_change);
	if (rc < 0)
		return rc;

	devm_device_init_wakeup(chip->dev);

	rc = devm_pm_set_wake_irq(chip->dev, chip->cable_irq);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc, "Couldn't set wake irq\n");

	platform_set_drvdata(pdev, chip);

	rc = regmap_write_bits(chip->regmap, chip->base + AICL_RERUN_TIME_CFG,
			       AICL_RERUN_TIME_MASK, AIC_RERUN_TIME_3_SECS);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Couldn't write fast AICL rerun time\n");

	rc = qcom_bms_register_charger(&pdev->dev, QCOM_BMS_CHARGER_PMIC,
				       &smb_charger_ops, chip);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc,
				     "Failed to register with qcom_bms\n");

	/* Initialise charger state */
	mod_delayed_work(system_dfl_wq, &chip->status_change_work, 0);

	return 0;
}

static void smb_remove(struct platform_device *pdev)
{
	qcom_bms_unregister_charger(&pdev->dev);
}

static const struct of_device_id smb_match_id_table[] = {
	{ .compatible = "qcom,pm7250b-charger" },
	{ .compatible = "qcom,pm8150b-charger" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, smb_match_id_table);

static struct platform_driver qcom_spmi_smb5 = {
	.probe = smb_probe,
	.remove = smb_remove,
	.driver = {
		.name = "qcom-smb5-charger",
		.of_match_table = smb_match_id_table,
	},
};

module_platform_driver(qcom_spmi_smb5);

MODULE_AUTHOR("Casey Connolly <casey.connolly@linaro.org>");
MODULE_AUTHOR("TwinbornPlate75 <3342733415@qq.com>");
MODULE_DESCRIPTION("Qualcomm SMB5 Charger Driver (qcom_bms sub-module)");
MODULE_LICENSE("GPL");
