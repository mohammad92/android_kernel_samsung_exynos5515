#ifndef _PMU_CAL_H_
#define _PMU_CAL_H_

#include "mif_reg_S5E5515.h"
#include "linux/regmap.h"

#define MAX_NAME_SIZE 20

extern struct regmap *pre_boot_wlbt_regmap;

struct pmucal {
	struct pmucal_data* init;
	struct pmucal_data* reset_assert;
	struct pmucal_data* reset_release;
	int init_size;
	int reset_assert_size;
	int reset_release_size;
};
struct pmucal_data{
	int accesstype;
	bool bypass;
	bool pmureg;
	int  sfr;
	int field;
	int value;
};

enum PMUCAL_ACCESS_TYPE{
	PMUCAL_WRITE,
	PMUCAL_DELAY,
	PMUCAL_READ,
	PMUCAL_ATOMIC,
};

extern struct pmucal pmucal_wlbt;
int pmu_cal_progress(struct pmucal_data *pmu_data, int pmucal_data_size);

#endif
