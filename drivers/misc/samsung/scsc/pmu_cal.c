/****************************************************************************
 *
 * Copyright (c) 2014 - 2021 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#include "pmu_cal.h"
#include "mif_reg_S5E5515.h"

/*  structure of pmucal_data */
/*	struct pmucal_data{
 *		int accesstype;
 *		bool pmureg;
 *		int  sfr;
 *		int field;
 *		int value;
 *
 *	};
 */

extern enable_hwbypass;

static char * pmu_cal_getsfr(int sfr){
    /* translate string to sfr addr */
    switch(sfr){
		case WLBT_CTRL_NS:
			return "WLBT_CTRL_NS";
		case WLBT_CTRL_S:
			return "WLBT_CTRL_S";
		case SYSTEM_OUT:
			return "SYSTEM_OUT";
		case WLBT_CONFIGURATION:
			return "CONFIGURATION";
		case WLBT_STATUS:
			return "WLBT_STATUS";
		case WLBT_IN:
			return "WLBT_IN";
		case VGPIO_TX_MONITOR:
			return "VGPIO_TX_MONITOR";
		case WLBT_INT_TYPE:
			return "WLBT_INT_TYPE";
		default:
			return NULL;
	}
}

#if 0
static char * pmu_cal_getfield(int sfr){
    /* translate string to sfr addr */
    switch(sfr){
		case WLBT_CTRL_NS:
			return "WLBT_CTRL_NS";
		case WLBT_CTRL_S:
			return "WLBT_CTRL_S";
		case SYSTEM_OUT:
			return "SYSTEM_OUT";
		case WLBT_CONFIGURATION:
			return "CONFIGURATION";
		case WLBT_STATUS:
			return "WLBT_STATUS";
		case WLBT_IN:
			return "WLBT_IN";
		case VGPIO_TX_MONITOR:
			return "VGPIO_TX_MONITOR";
		default:
			return NULL;
	}
}
#endif

static int pmu_cal_write(struct pmucal_data data){
	char * target_sfr;
	int value;
	int field;
	int ret;
	int val;

	/* set value */
	value = data.value << data.field;
	field = 1 << data.field;

	/* translate sfr addr to string */
	target_sfr = pmu_cal_getsfr(data.sfr);
	if(!target_sfr)
		return -EINVAL;

	/* write reg */
	ret = regmap_update_bits(pre_boot_wlbt_regmap, data.sfr, field, value);
	if(ret < 0){
		return ret;
	}
	regmap_read(pre_boot_wlbt_regmap, data.sfr, &val);
	return 0;

}

static int pmu_cal_read(struct pmucal_data data){

	unsigned long	timeout;
	char * target_sfr;
	int val = 0;
	int value;
	int chk;

	/* translate sfr addr to string */
	target_sfr = pmu_cal_getsfr(data.sfr);
	if(!target_sfr) return -EINVAL;

	/* set value */
	value = data.value << data.field;

	timeout = jiffies + msecs_to_jiffies(500);
	do {
		regmap_read(pre_boot_wlbt_regmap, data.sfr, &val);
		val &= (u32)value;
		if(!value) chk = !val;
		else chk = val;
		if (chk) {
			goto done;
		}
	} while (time_before(jiffies, timeout));

	regmap_read(pre_boot_wlbt_regmap, data.sfr, &val);
	return -EINVAL;

done:
	return 0;

}

static int pmu_cal_atomic(struct pmucal_data data){
	char * target_sfr;
	int sfr;
	int ret;
	int val;

	/* set value */
	sfr = data.sfr | 0xc000;

	/* translate sfr addr to string */
	target_sfr = pmu_cal_getsfr(data.sfr);
	if(!target_sfr) return -EINVAL;

	/* write reg */
	ret = regmap_write_bits(pre_boot_wlbt_regmap, sfr, data.value, data.value);
	if(ret < 0){
		return ret;
	}
	regmap_read(pre_boot_wlbt_regmap, data.sfr, &val);
	return 0;
}

int pmu_cal_progress(struct pmucal_data *pmu_data, int pmucal_data_size){
	int ret;
	int i =0;

	for(i =0; i < pmucal_data_size; i++){
		switch (pmu_data[i].accesstype){
			case PMUCAL_WRITE:
				ret = pmu_cal_write(pmu_data[i]);
				break;
			case PMUCAL_DELAY:
				udelay(pmu_data[i].value);
				break;
			case PMUCAL_READ:
				ret = pmu_cal_read(pmu_data[i]);
				break;
			case PMUCAL_ATOMIC:
				ret = pmu_cal_atomic(pmu_data[i]);
				break;
			default:
				return -EINVAL;
		}
	}
	return 0;
}
EXPORT_SYMBOL(pmu_cal_progress);

