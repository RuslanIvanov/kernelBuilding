/*
 * TechNexion TAM3517 
 *
 * Copyright (C) 2012 TechNexion Inc
 *
 * Maintainer: TechNexion <linuxfae@technexion.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/mmc/host.h>

#include <plat/usb.h>
#include <plat/board.h>
#include <plat/common.h>

#include <linux/regulator/machine.h>

#include <linux/davinci_emac.h>
#include <linux/can/platform/ti_hecc.h>
#include <media/davinci/vpfe_capture.h>
#include <mach/am35xx.h>

#include <plat/gpmc.h>

#include "mux.h"
#include "hsmmc.h"
#include "control.h"
#include "timer-gp.h"

#include <linux/usb/otg.h>
#include <mach/hardware.h>

#include <linux/serial_8250.h>
#include "board-technexion-common.h"
#include <linux/serial_sc16ix7xx.h>
/****************************************************************************
 *
 * TPS65023 voltage regulator
 *
 ****************************************************************************/

/* VDCDC1 -> VDD_CORE */
static struct regulator_consumer_supply tam3517_vdcdc1_supplies[] = {
	{
		.supply = "vdd_core",
	},
};

/* VDCDC2 -> VDDSHV */
static struct regulator_consumer_supply tam3517_vdcdc2_supplies[] = {
	{
		.supply = "vddshv",
	},
};

/* VDCDC2 |-> VDDS
	   |-> VDDS_SRAM_CORE_BG
	   |-> VDDS_SRAM_MPU */
static struct regulator_consumer_supply tam3517_vdcdc3_supplies[] = {
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdds_sram_mpu",
	},
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
};

/* LDO1 |-> VDDA1P8V_USBPHY
	 |-> VDDA_DAC */
static struct regulator_consumer_supply tam3517_ldo1_supplies[] = {
	{
		.supply = "vdda1p8v_usbphy",
	},
	{
		.supply = "vdda_dac",
	},
};

/* LDO2 -> VDDA3P3V_USBPHY */
static struct regulator_consumer_supply tam3517_ldo2_supplies[] = {
	{
		.supply = "vdda3p3v_usbphy",
	},
};


static struct regulator_init_data tam3517_regulators[] = {
	/* DCDC1 */
	{
		.constraints = {
			.min_uV = 1200000,
			.max_uV = 1200000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(tam3517_vdcdc1_supplies),
		.consumer_supplies = tam3517_vdcdc1_supplies,
	},
	/* DCDC2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(tam3517_vdcdc2_supplies),
		.consumer_supplies = tam3517_vdcdc2_supplies,
	},
	/* DCDC3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(tam3517_vdcdc3_supplies),
		.consumer_supplies = tam3517_vdcdc3_supplies,
	},
	/* LDO1 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = false,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(tam3517_ldo1_supplies),
		.consumer_supplies = tam3517_ldo1_supplies,
	},
	/* LDO2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = false,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(tam3517_ldo2_supplies),
		.consumer_supplies = tam3517_ldo2_supplies,
	},
};


/****************************************************************************
 *
 *  XR16L2751 Serial port (THB HMI only)
 *
 ****************************************************************************/

#define CONFIG_XR16L2571_PORTA_CS		1
#define CONFIG_XR16L2571_PORTB_CS		3
#define CONFIG_XR16L2571_PORTA_IRQ_PIN	155
#define CONFIG_XR16L2571_PORTB_IRQ_PIN	137
#define PORT(_base,_irq)								\
	{										\
		.mapbase	= _base,						\
		.irq		= _irq,						\
		.uartclk	= 14745600,						\
		.iotype	= UPIO_MEM,						\
		.flags		= UPF_BOOT_AUTOCONF|UPF_IOREMAP|UPF_SHARE_IRQ,	\
		.regshift	= 1,							\
		.irqflags	= IRQF_SHARED | IRQF_TRIGGER_HIGH,			\
	}

static struct plat_serial8250_port tam3517_xr16l2571_ports[] = {
	PORT(0x21000000,OMAP_GPIO_IRQ(CONFIG_XR16L2571_PORTA_IRQ_PIN)),
	PORT(0x23000000,OMAP_GPIO_IRQ(CONFIG_XR16L2571_PORTB_IRQ_PIN)),
	{}
};

static struct platform_device tam3517_xr16l2571_device =
{
        .name                   = "serial8250",
        .id                     = 3,
        .dev                    = {
                .platform_data  = &tam3517_xr16l2571_ports,
	},
};


static inline void __init tam3517_init_xr16l2571(void)
{
        unsigned long cs_mem_base;

	if (gpmc_cs_request(CONFIG_XR16L2571_PORTA_CS, SZ_16M, &cs_mem_base) < 0)
	{
		printk(KERN_ERR "Failed request for GPMC mem for xr16l2571 porta\n");
		return;
	}
	
	if(gpio_request(CONFIG_XR16L2571_PORTA_IRQ_PIN, "xr16l2571 porta irq"))
	{
		printk(KERN_ERR "Failed request gpio for xr16l2571 porta irq\n");
		return;
	}
	omap_mux_init_gpio(CONFIG_XR16L2571_PORTA_IRQ_PIN, OMAP_PIN_INPUT|OMAP_MUX_MODE4);

	gpio_direction_input(CONFIG_XR16L2571_PORTA_IRQ_PIN);

	if (gpmc_cs_request(CONFIG_XR16L2571_PORTB_CS, SZ_16M, &cs_mem_base) < 0)
	{
		printk(KERN_ERR "Failed request for GPMC mem for xr16l2571 portb\n");
		return;
	}

	if(gpio_request(CONFIG_XR16L2571_PORTB_IRQ_PIN, "xr16l2571 portb irq"))
	{
		printk(KERN_ERR "Failed request gpio for xr16l2571 portb irq\n");
		return;
	}
	omap_mux_init_gpio(CONFIG_XR16L2571_PORTB_IRQ_PIN, OMAP_PIN_INPUT|OMAP_MUX_MODE4);

	gpio_direction_input(CONFIG_XR16L2571_PORTB_IRQ_PIN);

	platform_device_register(&tam3517_xr16l2571_device);
}

/****************************************************************************
 *
 * EMAC LAN
 *
 ****************************************************************************/

#define TAM3517_MDIO_FREQUENCY	(1000000)

static struct mdio_platform_data tam3517_mdio_pdata = {
	.bus_freq	= TAM3517_MDIO_FREQUENCY,
};

static struct resource tam3517_mdio_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET,
		.end    = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET +
			  SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device tam3517_mdio_device = {
	.name		= "davinci_mdio",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(tam3517_mdio_resources),
	.resource	= tam3517_mdio_resources,
	.dev.platform_data = &tam3517_mdio_pdata,
};

static struct emac_platform_data tam3517_emac_pdata = {
	.rmii_en	= 1,
};

static struct resource tam3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x2FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device tam3517_emac_device = {
	.name		= "davinci_emac",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(tam3517_emac_resources),
	.resource	= tam3517_emac_resources,
};

static void tam3517_enable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR |
		AM35XX_CPGMAC_C0_MISC_PULSE_CLR |
		AM35XX_CPGMAC_C0_RX_THRESH_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void tam3517_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static __init void tam3517_ethernet_init(struct emac_platform_data *pdata) {
	u32 regval, mac_lo, mac_hi;
        char *ethaddr = strstr(saved_command_line, "eth0addr");
        
        if (ethaddr == NULL) {
                mac_lo = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_LSB);
                mac_hi = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_MSB);

		pdata->mac_addr[0] = (u_int8_t)((mac_hi & 0xFF0000) >> 16);
		pdata->mac_addr[1] = (u_int8_t)((mac_hi & 0xFF00) >> 8);
		pdata->mac_addr[2] = (u_int8_t)((mac_hi & 0xFF) >> 0);
		pdata->mac_addr[3] = (u_int8_t)((mac_lo & 0xFF0000) >> 16);
		pdata->mac_addr[4] = (u_int8_t)((mac_lo & 0xFF00) >> 8);
		pdata->mac_addr[5] = (u_int8_t)((mac_lo & 0xFF) >> 0);
	} else {
                int i;
                for(i = 0; i <  ETH_ALEN; i++)
                        pdata->mac_addr[i] = simple_strtol(ethaddr+9+3*i, NULL, 16);

        }

	pdata->ctrl_reg_offset		= AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset	= AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset		= AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->ctrl_ram_size		= AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version			= EMAC_VERSION_2;
	pdata->hw_ram_addr		= AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable		= tam3517_enable_ethernet_int;
	pdata->interrupt_disable	= tam3517_disable_ethernet_int;
	tam3517_emac_device.dev.platform_data	= pdata;

	platform_device_register(&tam3517_mdio_device);
	platform_device_register(&tam3517_emac_device);

	clk_add_alias(NULL, dev_name(&tam3517_mdio_device.dev),
		      NULL, &tam3517_emac_device.dev);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	return ;
}

/****************************************************************************
 *
 * HECC (High-End CAN Controller, for CAN-bus)
 *
 ****************************************************************************/

/*
 * HECC information
 */

/*
#define TAM3517_CAN_STB         42
static void tam3517_hecc_phy_control(int on)
{
        int r;

        r = gpio_request(TAM3517_CAN_STB, "can_stb");
        if (r) {
                printk(KERN_ERR "failed to get can_stb \n");
                return;
        }

        gpio_direction_output(TAM3517_CAN_STB, (on==1)?0:1);
}
*/

static struct resource tam3517_hecc_resources[] = {
	{
		.start	= AM35XX_IPSS_HECC_BASE,
		.end	= AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_35XX_HECC0_IRQ,
		.end	= INT_35XX_HECC0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tam3517_hecc_device = {
	.name		= "ti_hecc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(tam3517_hecc_resources),
	.resource	= tam3517_hecc_resources,
};

static struct ti_hecc_platform_data tam3517_hecc_pdata = {
	.scc_hecc_offset	= AM35XX_HECC_SCC_HECC_OFFSET,
	.scc_ram_offset		= AM35XX_HECC_SCC_RAM_OFFSET,
	.hecc_ram_offset	= AM35XX_HECC_RAM_OFFSET,
	.mbx_offset		= AM35XX_HECC_MBOX_OFFSET,
	.int_line		= AM35XX_HECC_INT_LINE,
	.version		= AM35XX_HECC_VERSION,
/*	.transceiver_switch     = tam3517_hecc_phy_control,*/
};

static void tam3517_hecc_init(struct ti_hecc_platform_data *pdata)
{
	tam3517_hecc_device.dev.platform_data = pdata;
	platform_device_register(&tam3517_hecc_device);
}

/****************************************************************************
 *
 *  GPIO_LED
 *
 ****************************************************************************/
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
#define TAM3517_STATUS_LED_GPIO		57

static struct gpio_led tam3517_gpio_leds[] = {
	{
		.name			= "tam3517::status",
		.default_trigger	= "mmc0",
		.gpio			= TAM3517_STATUS_LED_GPIO,
		.active_low		= true,
	},
};

static struct gpio_led_platform_data tam3517_gpio_led_info = {
	.leds		= tam3517_gpio_leds,
	.num_leds	= ARRAY_SIZE(tam3517_gpio_leds),
};

static struct platform_device tam3517_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &tam3517_gpio_led_info,
	},
};

static void __init tam3517_status_led_init(void)
{
	platform_device_register(&tam3517_leds_gpio);
}
#else
static inline void __init tam3517_status_led_init(void) { return; }
#endif

/****************************************************************************
 *
 * GPIO_BUTTON
 *
 ****************************************************************************/
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button tam3517_gpio_buttons[] = {
        {
                .code                   = KEY_HOME,
                .gpio                   = 65,
                .desc                   = "home",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_ENTER,
                .gpio                   = 64,
                .desc                   = "enter",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_BACK,
                .gpio                   = 63,
                .desc                   = "back",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_MENU,
                .gpio                   = 150,
                .desc                   = "menu",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_BRIGHTNESSUP,
                .gpio                   = 143,
                .desc                   = "brightness up",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_BRIGHTNESSDOWN,
                .gpio                   = 142,
                .desc                   = "brightness down",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_VOLUMEUP,
                .gpio                   = 141,
                .desc                   = "volume up",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_VOLUMEDOWN,
                .gpio                   = 140,
                .desc                   = "volume down",
                .wakeup                 = 1,
                .active_low             = 1,
        },
};

static struct gpio_keys_platform_data tam3517_gpio_key_info = {
        .buttons        = tam3517_gpio_buttons,
        .nbuttons       = ARRAY_SIZE(tam3517_gpio_buttons),
};

static struct platform_device tam3517_keys_gpio = {
        .name   = "gpio-keys",
        .id     = -1,
        .dev    = {
                .platform_data  = &tam3517_gpio_key_info,
        },
};

static void __init tam3517_gpio_keys_init(void)
{
	platform_device_register(&tam3517_keys_gpio);
}
#else
static inline void __init tam3517_gpio_keys_init(void) { return; }
#endif

/****************************************************************************
 *
 * I2C
 *
 ****************************************************************************/

#if 0
/*
 * RTC - S35390A
 */
#define GPIO_RTCS35390A_IRQ	55

static void __init tam3517_rtc_init(void)
{
	int r;

	omap_mux_init_gpio(GPIO_RTCS35390A_IRQ, OMAP_PIN_INPUT_PULLUP);
	r = gpio_request(GPIO_RTCS35390A_IRQ, "rtcs35390a-irq");
	if (r < 0) {
		printk(KERN_WARNING "failed to request GPIO#%d\n",
				GPIO_RTCS35390A_IRQ);
		return;
	}
	r = gpio_direction_input(GPIO_RTCS35390A_IRQ);
	if (r < 0) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as input\n",
				GPIO_RTCS35390A_IRQ);
		gpio_free(GPIO_RTCS35390A_IRQ);
		return;
	}
	tam3517_i2c1_boardinfo[0].irq = gpio_to_irq(GPIO_RTCS35390A_IRQ);
}
#endif

static struct i2c_board_info __initdata tam3517_i2c1_boardinfo[] = {
#if 0
	{
		I2C_BOARD_INFO("s35390a", 0x30),
	},
#endif
        {
                I2C_BOARD_INFO("tps65023", 0x48),
                .flags = I2C_CLIENT_WAKE,
                .platform_data = tam3517_regulators,
        },
	{
                I2C_BOARD_INFO("24c02", 0x50),
        },
	{
                I2C_BOARD_INFO("tlv320aic23", 0x1a),
        },
};

#define TWISTER_PRISM_GPIO_TS_RESET 176
#define TWISTER_PRISM_GPIO_TS_IRQ 177

#if defined(CONFIG_UPUPVV)
#if defined(CONFIG_SERIAL_SC16IS7XX_I2C)
	#define GPIO_PVV1  0
	#define GPIO_PVV2  136

static struct plat_serial_sxx __initdata sc16ix7xx_ports[] = {
	{
		.uartclk = 14745600,
		.num_i2c = 1,//1,//i2c2
		.name_i2c = "sc16is750",//"BRP_I2C",
	},
	{
		.uartclk = 14745600,
		.num_i2c = 2,//2,//i2c3
		.name_i2c ="sc16is750" ,//"BRP_I3C",
	},	
};

#endif
#endif

static struct i2c_board_info __initdata tam3517_i2c2_boardinfo[] = {

//	{I2C_BOARD_INFO("tlv320aic23", 0x1A),},

#if defined(CONFIG_TOUCHSCREEN_PRISM)
        {
                I2C_BOARD_INFO("prism", 0x10),
                .irq = OMAP_GPIO_IRQ(TWISTER_PRISM_GPIO_TS_IRQ),
                .flags = I2C_CLIENT_WAKE,
        },
#endif

#if defined(CONFIG_RTC_DRV_ISL12029)
	{
		I2C_BOARD_INFO("isl12029",0x6f),
	},
#endif

#if defined(CONFIG_UPUPVV)
#if defined(CONFIG_SERIAL_SC16IS7XX_I2C)
	{
        I2C_BOARD_INFO("sc16is750",0x4b),//SC16IS760 //sc16is760 //sc16is7xx
		.irq = OMAP_GPIO_IRQ(GPIO_PVV1),
		.flags = (IORESOURCE_IRQ | IRQF_TRIGGER_FALLING) & IRQF_TRIGGER_MASK, //I2C_CLIENT_WAKE
		.platform_data = (void*)&sc16ix7xx_ports[0],		
     },
#endif
#endif

};

static struct i2c_board_info __initdata tam3517_i2c3_boardinfo[] = {
#ifndef CONFIG_UPUPVV
        {
                I2C_BOARD_INFO("ds1307", 0x68),
        },
#else

#if defined(CONFIG_RTC_DRV_ISL12029)
	//{	I2C_BOARD_INFO("isl12029",0x6f),},
#endif

#if defined(CONFIG_SERIAL_SC16IS7XX)
	/*{
		I2C_BOARD_INFO("sc16is760",0x49),//SC16IS760 //sc16is760 //sc16is7xx
		.irq = OMAP_GPIO_IRQ(GPIO_PVV2),
		.flags = (IORESOURCE_IRQ | IRQF_TRIGGER_FALLING) & IRQF_TRIGGER_MASK, //I2C_CLIENT_WAKE
		.platform_data =&sc16ix7xx_ports[1],
	},*/
#endif

#endif
};

static struct i2c_board_info __initdata tam3517_tsc_i2c_boardinfo[] = {

};

static int __init tam3517_i2c_init(void)
{
#ifdef CONFIG_TOUCHSCREEN_PRISM
	if(!gpio_request(TWISTER_PRISM_GPIO_TS_RESET, "ts reset") && !gpio_direction_output(TWISTER_PRISM_GPIO_TS_RESET, 0))
	{
		printk(" Twister prism: resetting touchscreen\n");
		gpio_export(TWISTER_PRISM_GPIO_TS_RESET, 0);
		gpio_set_value(TWISTER_PRISM_GPIO_TS_RESET, 0);
		mdelay(50);
		gpio_set_value(TWISTER_PRISM_GPIO_TS_RESET, 1);
	}
	else
		printk("GPIO 176 could not be reserved or set as output\n");

	if(gpio_request(TWISTER_PRISM_GPIO_TS_IRQ, "ts irq") || gpio_direction_input(TWISTER_PRISM_GPIO_TS_IRQ))
	{
		printk("GPIO 177 could not be reserved or set as input\n");
		gpio_export(TWISTER_PRISM_GPIO_TS_IRQ, 0);
	}
#endif

	omap_register_i2c_bus(1, 400, NULL, 0);
	omap_register_i2c_bus(2, 400, tam3517_i2c2_boardinfo,
			ARRAY_SIZE(tam3517_i2c2_boardinfo));
#ifndef CONFIG_UPUPVV
	omap_register_i2c_bus(3, 400, tam3517_i2c3_boardinfo,
			ARRAY_SIZE(tam3517_i2c3_boardinfo));
#else
	omap_register_i2c_bus(3, 400, NULL,0);
#endif

	return 0;
}

/****************************************************************************
 *
 * USB
 *
 ****************************************************************************/

static struct omap_musb_board_data tam3517_musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
	.mode                   = MUSB_OTG,
	.power                  = 500,
};

static __init void tam3517_musb_init(void)
{
	u32 devconf2;

	/*
	 * Set up USB clock/mode in the DEVCONF2 register.
	 */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
			| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&tam3517_musb_board_data);
}

#define TAM3517_EHCI_RESET	25

static const struct ehci_hcd_omap_platform_data tam3517_ehci_pdata __initconst = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = TAM3517_EHCI_RESET,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static __init void tam3517_ehci_init(void)
{
	/* Configure GPIO for EHCI port */
	omap_mux_init_gpio(TAM3517_EHCI_RESET, OMAP_PIN_OUTPUT);

	gpio_request(TAM3517_EHCI_RESET, "USB_RESET");
	gpio_direction_output(TAM3517_EHCI_RESET, 0);
	gpio_export(TAM3517_EHCI_RESET, 0);
	msleep(100);
	gpio_set_value(TAM3517_EHCI_RESET, 1);

	usb_ehci_init(&tam3517_ehci_pdata);
}

/****************************************************************************
 *
 * MMC bus
 *
 ****************************************************************************/

static struct omap2_hsmmc_info tam3517_mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= 126,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.transceiver	= false,
		.ext_clock	= false,
	},
	{}      /* Terminator */
};

/****************************************************************************
 *
 * MUX
 *
 ****************************************************************************/

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux tam3517_board_mux[] __initdata = {
	/* USB OTG DRVVBUS offset = 0x212 */
#if 0
	OMAP3_MUX(SAD2D_MCAD23, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(SYS_NRESWARM, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
#endif

	OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN), /* spi */
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(MCSPI1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(MCSPI1_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_CS0, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

#ifdef CONFIG_UPUPVV

	OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN ), //pin 76
   	OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN ), //pin  27
//don't work OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN ), //pin  27
#else
	OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE), /* Touchscreen irq */
#endif
	OMAP3_MUX(SDMMC2_DAT6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* LCD */
	OMAP3_MUX(SDMMC2_DAT7, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* LCD */
	OMAP3_MUX(ETK_D10, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN), /* DVI */
	OMAP3_MUX(ETK_D11, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* USB RESET */
#if 0
	OMAP3_MUX(GPMC_NCS2, OMAP_MUX_MODE2 | OMAP_PULL_UP), /* BL */
#endif
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(GPMC_NCS6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT ), /*GPIO_57, LED*/
	OMAP3_MUX(GPMC_NCS7, OMAP_MUX_MODE4 | OMAP_PULL_UP),
	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT), /* GPIO 142 for keypad */
	OMAP3_MUX(SDMMC1_DAT4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT), /* GPIO 126 Card Detect for SD card */
#if 0
	OMAP3_MUX(SYS_NRESWARM, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT), /* For watchdog */
#endif
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif



/****************************************************************************
 *
 * Platform
 *
 ****************************************************************************/

/*
 * Board initialization
 */
static struct omap_board_config_kernel tam3517_config[] __initdata = {
};

#if 0
static struct platform_device *tam3517_devices[] __initdata = {
	&tam3517_dss_device,
	&usb_mass_storage_device,
};
#endif

static void __init tam3517_init_irq(void)
{
	omap_board_config = tam3517_config;
	omap_board_config_size = ARRAY_SIZE(tam3517_config);
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
	gpmc_init();
}

static int __init tam3517_setup_board_devices(void)
{
	/*This is write to init nand,
	  we should be able init something else as well.*/
	tncommon_flash_devices_init();

	tam3517_ehci_init();

	tncommon_display_subsystem_init(&dss_controls[tncommon_board_type]);

	tncommon_start_external_netdevs(&tncommon_smsc911x_settting[tncommon_board_type]);
 
	tncommon_start_tsc_devices();

	if(tncommon_board_type == TNCOMMON_BOARD_TYPE_THB)
	{
		/*init gpio buttom*/
		tam3517_gpio_keys_init();
		/*init exar xr16l2571 dual port uart*/
		tam3517_init_xr16l2571();
	}

#ifdef CONFIG_USB_ANDROID
        tncommon_android_gadget_init();
#endif
	return 0;
}

static int __init tam3517_setup_board_type(char *str)
{
	return tncommon_setup_board_type(str);
}
__setup("boardname=", tam3517_setup_board_type);

static void __init tam3517_init(void)
{
	omap3_mux_init(tam3517_board_mux, OMAP_PACKAGE_CBB);

	
#if defined(CONFIG_UPUPVV)
	printk(KERN_INFO "tam3517_init: CONFIG_UPUPVV");
#if defined(CONFIG_SERIAL_SC16IS7XX_I2C)
	printk(KERN_INFO "tam3517_init: CONFIG_SERIAL_SC16IS7XX_I2C");
#endif
#endif

	tam3517_i2c_init();
#if 0
	platform_add_devices(tam3517_devices,
			ARRAY_SIZE(tam3517_devices));
#endif

	omap_serial_init();

	tam3517_hecc_init(&tam3517_hecc_pdata);

	tam3517_status_led_init();

	i2c_register_board_info(1, tam3517_i2c1_boardinfo,
				ARRAY_SIZE(tam3517_i2c1_boardinfo));
	/*Ethernet*/
	tam3517_ethernet_init(&tam3517_emac_pdata);

	/* MUSB */
	tam3517_musb_init();

	/* MMC init function */
	omap2_hsmmc_init(tam3517_mmc);
	tncommon_mmc2_wifi_init();

	if(tncommon_board_type==TNCOMMON_BOARD_TYPE_INVALID)
		tncommon_board_type = TNCOMMON_BOARD_TYPE_TWISTER;

	tam3517_setup_board_devices();
}

/* Maintainer: TechNexion: linuxfae@technexion.com */

MACHINE_START(TAM3517, "TAM3517")
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= tam3517_init_irq,
	.init_machine	= tam3517_init,
	.timer		= &omap_timer,
MACHINE_END
