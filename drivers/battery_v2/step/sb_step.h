/*
 * sb_step.h
 * Samsung Mobile Battery Step Charging Header
 *
 * Copyright (C) 2022 Samsung Electronics, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SB_STEP_H
#define __SB_STEP_H __FILE__

#include <linux/err.h>

struct sb_step;
struct device_node;

struct sb_step_data {
	unsigned	volt_avg : 14;
	unsigned	cable_type : 8;
	unsigned	status : 4;
	unsigned	rsv1 : 32;
	unsigned	rsv2 : 6;
};

static inline struct sb_step_data set_step_data(int volt_avg, int cable_type, int status)
{
	struct sb_step_data st_data;

	st_data.volt_avg = volt_avg;
	st_data.cable_type = cable_type;
	st_data.status = status;

	return st_data;
}

#define SB_STEP_DISABLE	(-3700)
#if defined(CONFIG_SB_STEP)
struct sb_step *sb_step_create(struct device_node *np);
int sb_step_init(struct sb_step *st);
int sb_step_monitor(struct sb_step *st, struct sb_step_data data);
#else
static inline struct sb_step *sb_step_create(struct device_node *np)
{ return ERR_PTR(SB_STEP_DISABLE); }
static inline int sb_step_init(struct sb_step *st)
{ return SB_STEP_DISABLE; }
static inline int sb_step_monitor(struct sb_step *st, struct sb_step_data data)
{ return SB_STEP_DISABLE; }
#endif

#endif /* __SB_STEP_H */

