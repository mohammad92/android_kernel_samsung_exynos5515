/*
 * sec_vote_event.h
 * Samsung Mobile Battery Vote Event Header
 *
 * Copyright (C) 2021 Samsung Electronics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SEC_VOTE_EVENT_H
#define __SEC_VOTE_EVENT_H __FILE__

/* VOTER NAME LIST */
#define VN_WPC_PHM	"WPC-PHM"
#define VN_WPC_APM	"WPC-APM"
#define VN_WPC_LDO	"WPC-LDO"
#define VN_CHG_EN	"CHGEN"

enum WPC_PHM_SET {
	WPC_PHM_OFF	= 0,
	WPC_PHM_ON,
	WPC_PHM_EPT_ON,
};

enum WPC_APM_SET {
	WPC_APM_NONE = 0,
	WPC_APM_NORMAL,
	WPC_APM_STORE,
	WPC_APM_BOOT,
	WPC_APM_WAKE_UP,
	WPC_APM_VOUT_ON,
	WPC_APM_VOUT_OFF,
	WPC_APM_TEST,
};

enum WPC_LDO_SET {
	WPC_LDO_ON = 0,
	WPC_LDO_OFF,
};

#define FOREACH_VOTER(GENERATE)	{ \
	GENERATE(VOTER_CABLE),    \
	GENERATE(VOTER_TEST),    \
	GENERATE(VOTER_P9222_CONTROL),    \
	GENERATE(VOTER_S2MPW03_CONTROL),    \
	GENERATE(VOTER_HEAT_CONTROL),    \
	GENERATE(VOTER_CSTEP),    \
	GENERATE(VOTER_CC2_VOL),    \
	GENERATE(VOTER_BOOT_MODE),    \
	GENERATE(VOTER_AGING_STEP),	\
	GENERATE(VOTER_STORE_MODE),	\
	GENERATE(VOTER_SWELLING),	\
	GENERATE(VOTER_SYSOVLO),	\
	GENERATE(VOTER_VBUS_OVP),	\
	GENERATE(VOTER_FULL_CHARGE),	\
	GENERATE(VOTER_TIME_EXPIRED),	\
	GENERATE(VOTER_TOPOFF_CHANGE),    \
	GENERATE(VOTER_SLATE),	\
	GENERATE(VOTER_VBAT_OVP),	\
	GENERATE(VOTER_WPC_DET),	\
	GENERATE(VOTER_WPC_ID_CHK),	\
	GENERATE(VOTER_WPC_ALIGN_CHK),	\
	GENERATE(VOTER_WPC_TX_PHM),	\
	GENERATE(VOTER_WPC_CS100),	\
	GENERATE(VOTER_MAX)	\
}

#define GENERATE_ENUM(ENUM) ENUM
#define GENERATE_STRING(STRING)	#STRING

enum VOTER_ENUM
FOREACH_VOTER(GENERATE_ENUM);

extern const char *sec_voter_name[];

#endif /* __SEC_VOTE_EVENT_H */
