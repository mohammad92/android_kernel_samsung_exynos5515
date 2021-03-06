#ifndef __LINUX_PLATFORM_DATA_NANOHUB_H
#define __LINUX_PLATFORM_DATA_NANOHUB_H

#include <linux/types.h>

struct nanohub_flash_bank {
	int bank;
	u32 address;
	size_t length;
};

#define MAX_FILE_LEN (32)

struct nanohub_platform_data {
#ifdef CONFIG_NANOHUB_MAILBOX
	void *mailbox_client;
	int irq;
#else
	u32 wakeup_gpio;
	u32 nreset_gpio;
	u32 boot0_gpio;
	u32 irq1_gpio;
	u32 irq2_gpio;
	u32 spi_cs_gpio;
	u32 bl_addr;
	u32 num_flash_banks;
	struct nanohub_flash_bank *flash_banks;
	u32 num_shared_flash_banks;
	struct nanohub_flash_bank *shared_flash_banks;
#endif
};

enum CHUB_STATE {
    CHUB_FW_ST_INVAL,
    CHUB_FW_ST_POWERON,
    CHUB_FW_ST_OFF,
    CHUB_FW_ST_ON,
};

/* Structure of notifier block */
struct contexthub_notifier_block {
	const char *subsystem;
	unsigned int start_off;
	unsigned int end_off;
	enum CHUB_STATE state;
	int (*notifier_call)(struct contexthub_notifier_block *);
};

int contexthub_notifier_register(struct contexthub_notifier_block *nb);
#endif /* __LINUX_PLATFORM_DATA_NANOHUB_H */
