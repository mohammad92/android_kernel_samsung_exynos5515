/*
 *  Copyright (C) 2012, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */
#include "../ssp.h"

#define	VENDOR		"TI"
#define	CHIP_ID		"OPT3007"

#define OPT3007_REG_EXPONENT(n)		((n) >> 12)
#define OPT3007_REG_MANTISSA(n)		((n) & 0xfff)
#define ChCoef		500 // 50.0


/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/
static s32 get_lsb_size(uint8_t e)
{
	s32 lsb_size = 0;

	switch (e) {
	case 0:
		lsb_size = 1;// 0.01f;
		break;
	case 1:
		lsb_size = 2;//0.02f;
		break;
	case 2:
		lsb_size = 4;//0.04f;
		break;
	case 3:
		lsb_size = 8;//0.08f;
		break;
	case 4:
		lsb_size = 16;//0.16f;
		break;
	case 5:
		lsb_size = 32;//0.32f;
		break;
	case 6:
		lsb_size = 64;//0.64f;
		break;
	case 7:
		lsb_size = 128;//1.28f;
		break;
	case 8:
		lsb_size = 256;//2.56f;
		break;
	case 9:
		lsb_size = 512;//5.12f;
		break;
	case 10:
		lsb_size = 1024;//10.24f;
		break;
	case 11:
		lsb_size = 2048;//20.48f;
		break;
	default:
		lsb_size = 0;
		break;
	}
	return lsb_size;
}

s32 light_get_lux(uint16_t m, uint8_t e)
{
	static s32 lux;
	static s32 lsb_size;

	lsb_size = (s32)get_lsb_size(e);

	lux = (s32)((lsb_size * m * ChCoef) / 1000); /* lsb size 0.01, ChCoed 0.1 = 1000 */

	return lux;
}

static ssize_t light_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t light_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_ID);
}

static ssize_t light_lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->lux);
}

static ssize_t light_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	u16 result_r = (u16)(OPT3007_REG_MANTISSA(data->buf[LIGHT_SENSOR].rdata));
	u8 result_e = (u8)(OPT3007_REG_EXPONENT(data->buf[LIGHT_SENSOR].rdata));

	return snprintf(buf, PAGE_SIZE, "%u %u\n", result_r, result_e);
}

static DEVICE_ATTR(vendor, S_IRUGO, light_vendor_show, NULL);
static DEVICE_ATTR(name, S_IRUGO, light_name_show, NULL);
static DEVICE_ATTR(lux, S_IRUGO, light_lux_show, NULL);
static DEVICE_ATTR(raw_data, S_IRUGO, light_data_show, NULL);

static struct device_attribute *light_attrs[] = {
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_lux,
	&dev_attr_raw_data,
	NULL,
};

void initialize_opt3007_light_factorytest(struct ssp_data *data)
{
	sensors_register(data->light_device, data, light_attrs, "light_sensor");
}

void remove_opt3007_light_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->light_device, light_attrs);
}
