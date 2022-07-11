/*
 * Samsung debugging features for Samsung's SoC's.
 *
 * Copyright (c) 2014-2019 Samsung Electronics Co., Ltd.
 *      http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/* keys are grouped by size */
static char key32[][MAX_ITEM_KEY_LEN] = {
	"ID", "KTIME", "BIN", "RR",
	"SMP", "PCB", "SMD",
	"LEV", "ASB", "PSITE", "DDRID", "RST",
	"INFO2", "INFO3", "PWR",
	"PWROFF", "PINT1", "PINT2", "PINT5", "PINT6",
	"PSTS1", "PSTS2", "RSTCNT", "FPMU"
};

static char key64[][MAX_ITEM_KEY_LEN] = {
	"BAT", "FAULT",	"EPD",
	"MOCP", "SOCP", "ODR",
};

static char key256[][MAX_ITEM_KEY_LEN] = {
	"KLG", "BUS", "PANIC", "PC", "LR",
	"BUG", "ESR", "SMU", "FREQ",
	"AUD", "UP", "DOWN", "WDGC", "DSSBUS",
};

static char key1024[][MAX_ITEM_KEY_LEN] = {
	"CPU0", "CPU1", "CPU2", "CPU3", "CPU4",
	"CPU5", "CPU6", "CPU7", "MFC", "STACK",
	"FPMU", "REGS", "FPMUMSG",
};

/* keys are grouped by sysfs node */
static char akeys[][MAX_ITEM_KEY_LEN] = {
	"ID", "KTIME", "RR", "ODR", "RST", "FAULT",
	"BUG", "PC", "LR", "STACK", "PANIC",
	"RSTCNT", "SMU", "BUS", "DSSBUS",
	"UP", "DOWN", "WDGC", "EPD",
	"BAT", "ASB", "KLG", "DDRID",
	"ESR", "FREQ", "SMP", "BIN", "LEV",
	"PCB", "SMD", "FPMU",
};

static char bkeys[][MAX_ITEM_KEY_LEN] = {
	"ID", "RR", "PSITE",
	"MOCP", "SOCP", "INFO2", "INFO3",
	"PWR", "PWROFF", "PINT1",
	"PINT2", "PINT5", "PINT6", "PSTS1", "PSTS2",
};

static char ckeys[][MAX_ITEM_KEY_LEN] = {
	"ID", "RR", "CPU0", "CPU1", "CPU2",
	"CPU3", "CPU4", "CPU5", "CPU6", "CPU7",
};

static char fkeys[][MAX_ITEM_KEY_LEN] = {
	"ID", "RR", "FPMUMSG",
};

static char mkeys[][MAX_ITEM_KEY_LEN] = {
	"ID", "RR", "MFC", "AUD",
};

static char tkeys[][MAX_ITEM_KEY_LEN] = {
	"ID", "RR", "REGS",
};
