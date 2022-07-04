/****************************************************************************
 *
 * Copyright (c) 2014 - 2021 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/smc.h>

#include "mif_reg_S5E5515.h"
#include "pmu_cal.h"
#include "ka_patch.h"

#if defined(CONFIG_ARCH_EXYNOS) || defined(CONFIG_ARCH_EXYNOS9)
#include <linux/soc/samsung/exynos-soc.h>
#endif

struct regmap *pre_boot_wlbt_regmap;
struct regmap *boot_cfg;
struct platform_device *pre_boot_wlbt_pdev;

static int pre_boot_wlbt_module_probe(struct platform_device *pdev);
static int pre_boot_wlbt_module_remove(struct platform_device *pdev);
static int pre_boot_wlbt_register_irq(struct platform_device *pdev);

struct {
	int irq_num;
	int flags;
	atomic_t irq_disabled_cnt;
} wlbt_irq;

static void set_wlbt_irq(struct platform_device *pdev)
{
	struct resource *irq_res;
	int irqtag;

	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	irqtag = 0;
	wlbt_irq.irq_num = irq_res->start;
	wlbt_irq.flags = (irq_res->flags & IRQF_TRIGGER_MASK);

	atomic_set(&wlbt_irq.irq_disabled_cnt, 0);
};

static void cfg_req_irq_clear_pending(void)
{
	int irq;
	int ret;
	bool pending = 0;

	irq = wlbt_irq.irq_num;
	ret = irq_get_irqchip_state(irq, IRQCHIP_STATE_PENDING, &pending);

	if (!ret) {
		if (pending == 1) {
			pending = 0;
			ret = irq_set_irqchip_state(irq, IRQCHIP_STATE_PENDING,
						    pending);
		}
	}
}

static int pre_boot_wlbt_init_phandle(struct device *dev)
{
	pre_boot_wlbt_regmap = syscon_regmap_lookup_by_phandle(
		dev->of_node, "samsung,pre_boot_wlbt_pmu-syscon-phandle");
	boot_cfg = syscon_regmap_lookup_by_phandle(
		dev->of_node, "samsung,boot_cfg-syscon-phandle");
	if (IS_ERR(pre_boot_wlbt_regmap) || IS_ERR(boot_cfg))
		return -EINVAL;
	else
		return 0;
}

static const struct of_device_id pre_boot_wlbt[] = {
	{ .compatible = "samsung,pre_boot_wlbt" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pre_boot_wlbt);

static struct platform_driver pre_boot_wlbt_driver =
	{ .probe = pre_boot_wlbt_module_probe,
	  .remove = pre_boot_wlbt_module_remove,
	  .driver = {
		  .name = "pre_boot_wlbt",
		  .owner = THIS_MODULE,
		  .pm = NULL,
		  .of_match_table = of_match_ptr(pre_boot_wlbt),
	  } };

module_platform_driver(pre_boot_wlbt_driver);

static int pre_boot_wlbt_module_remove(struct platform_device *pdev)
{
	return 0;
}

/* WARNING! this function might be
 * running in IRQ context if CONFIG_SCSC_WLBT_CFG_REQ_WQ is not defined */
static void set_pre_boot_wlbt_regs(void)
{
	/*s32 ret = 0;*/
	u64 ret64 = 0;
	const u64 EXYNOS_WLBT = 0x1;
	unsigned int ka_addr = PMU_BOOT_RAM_START;
	uint32_t *ka_patch_addr;
	uint32_t *ka_patch_addr_end;
	size_t ka_patch_len;
	unsigned int val;
	unsigned long	timeout;
	int i;

	ka_patch_addr = &ka_patch[0];
	ka_patch_addr_end = ka_patch_addr + ARRAY_SIZE(ka_patch);
	ka_patch_len = ARRAY_SIZE(ka_patch);

	/* Set TZPC to non-secure mode */
	ret64 = exynos_smc(SMC_CMD_CONN_IF, (EXYNOS_WLBT << 32) | 0, 0, 0);

#define CHECK(x)                                                               \
	do {                                                                   \
		int retval = (x);                                              \
		if (retval < 0) {                                              \
			pr_err("%s failed at L%d", __FUNCTION__, __LINE__);    \
			goto cfg_error;                                        \
		}                                                              \
	} while (0)

	/* PMU boot patch */
	CHECK(regmap_write(boot_cfg, PMU_BOOT, PMU_BOOT_AP_ACC));
	i = 0;

	while (ka_patch_addr < ka_patch_addr_end) {
		CHECK(regmap_write(boot_cfg, ka_addr, *ka_patch_addr));
		ka_addr += (unsigned int)sizeof(unsigned int);
		ka_patch_addr++;
		i++;
	}

	/* Notify PMU of configuration done */
	CHECK(regmap_write(boot_cfg, PMU_BOOT, PMU_BOOT_PMU_ACC));

	/* BOOT_CFG_ACK */
	CHECK(regmap_write(boot_cfg, PMU_BOOT_ACK, PMU_BOOT_COMPLETE));

	timeout = jiffies + msecs_to_jiffies(500);
	do {
		regmap_read(boot_cfg, WB2AP_MAILBOX, &val);
		if (val == 0xA1) {
			pr_info("Pre-boot success 0x%x\n", val);
			break;
		}
	} while (time_before(jiffies, timeout));

cfg_error:
	return;
}

static void pre_boot_wlbt_booting(void)
{
	pr_info("wlbt_pre_boot start\n");
	pre_boot_wlbt_register_irq(pre_boot_wlbt_pdev);
	pmu_cal_progress(pmucal_wlbt.init, pmucal_wlbt.init_size);
	enable_irq(wlbt_irq.irq_num);
}

static int pre_boot_wlbt_module_probe(struct platform_device *pdev)
{
	int ret;
	pr_info("pre_boot_wlbt_module_probe\n");
	ret = pre_boot_wlbt_init_phandle(&pdev->dev);

	if (ret)
		return ret;

	set_wlbt_irq(pdev);
	pre_boot_wlbt_pdev = pdev;
	pr_info("pre_boot_wlbt init_phandle done\n");
	pre_boot_wlbt_booting();
	return 0;
}
irqreturn_t pre_boot_wlbt_cfg_req_isr(int irq, void *data)
{
	/* mask the irq */
	disable_irq_nosync(wlbt_irq.irq_num);
	pr_info("pre_boot_wlbt : cfg_req interrupt handle\n");

	set_pre_boot_wlbt_regs();

	return IRQ_HANDLED;
}

static int pre_boot_wlbt_register_irq(struct platform_device *pdev)
{
	int err;

	/* Mark as WLBT in reset before enabling IRQ to guard against spurious IRQ */
	smp_wmb(); /* commit before irq */

	/* Register WB2AP_CFG_REQ irq */
	/* clean CFG_REQ PENDING interrupt. */
	cfg_req_irq_clear_pending();

	err = devm_request_irq(&pdev->dev, wlbt_irq.irq_num,
			       pre_boot_wlbt_cfg_req_isr, wlbt_irq.flags,
			       "boot_cfg", NULL);
	if (IS_ERR_VALUE((unsigned long)err)) {
		pr_info("pre_boot_wlbt irq register fail\n");
		err = -ENODEV;
		return err;
	}
	pr_info("pre_boot_wlbt irq register success\n");
	/* Leave disabled until ready to handle */
	disable_irq_nosync(wlbt_irq.irq_num);

	return 0;
}
