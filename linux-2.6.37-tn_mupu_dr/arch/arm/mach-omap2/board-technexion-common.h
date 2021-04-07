/*
 * Header for TechNexion Common:
 *
 * Extrat board file items which can be reuse
 *
 * Copyright (C) 2012 TechNexion Inc
 *
 * Maintainer: TechNexion <linuxfae@technexion.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _tncommon_h
#define _tncommon_h

enum {
        TNCOMMON_BOARD_TYPE_BLIZZARD=0,
        TNCOMMON_BOARD_TYPE_TWISTER,
        TNCOMMON_BOARD_TYPE_TSUNAMI,
        TNCOMMON_BOARD_TYPE_THUNDER,
        TNCOMMON_BOARD_TYPE_INFERNO,
        TNCOMMON_BOARD_TYPE_BLIZZARD_A2,
        TNCOMMON_BOARD_TYPE_THB,
        TNCOMMON_BOARD_TYPE_INVALID
};


struct tncommon_smsc911x_platform_control {
	char *name;
	int	irq_gpio;
	int	reset_gpio;
	int gpmc_cs_number;
};

struct tncommon_gpio_control_type {
	int gpio;
	int enable_type;
};

struct tncommon_backlight_control {
	char *name;
	int type;
	struct tncommon_gpio_control_type pwr_gpio;
};

#define TNCOMMON_MAX_DSS_CONTROL_GPIO	4

struct tncommon_dss_panel_control {
	char *name;
	struct tncommon_gpio_control_type gpios[TNCOMMON_MAX_DSS_CONTROL_GPIO];
	struct tncommon_backlight_control bklight;
};

extern int tncommon_display_subsystem_init(struct tncommon_dss_panel_control *dss_controls);
extern int tncommon_start_external_netdevs(struct tncommon_smsc911x_platform_control *smsc_chip);
extern int tncommon_start_tsc_devices(void);
extern void tncommon_android_gadget_init(void);
extern void tncommon_flash_devices_init(void);
extern void tncommon_enable_hdmi(void);
extern int tncommon_setup_board_type(char *str);
extern int tncommon_board_type;
extern void tncommon_mmc2_wifi_init(void);

extern struct tncommon_smsc911x_platform_control tncommon_smsc911x_settting[];

extern struct tncommon_dss_panel_control dss_controls[];

#endif
