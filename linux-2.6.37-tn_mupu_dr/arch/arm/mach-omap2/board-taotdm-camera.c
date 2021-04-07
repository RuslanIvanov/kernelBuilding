/*
 * TAOTDM_CAMERA: Driver for Leopard Module Board
 *
 * Copyright (C) 2012 TechNexion Inc
 *
 * Author: Edward Lin  <linuxfae@technexion.com>
 *
 * Based on arch/arm/mach-omap2/board-omap3evm-camera.c by Vaibhav Hiremath
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>

#include <media/mt9t111.h>
#include <media/tvp514x.h>

#include <../drivers/media/video/isp/isp.h>

#include "mux.h"
#include "devices.h"

#define CAM_USE_XCLKA		0

#define T2_GPIO_2			194
#define nCAM_VD_SEL			157
#define nCAM_VD_EN			200

static struct regulator *taotdm_io_1v8;
static struct regulator *taotdm_core_1v8;
int taotdm_io_pwr_ready = 0;
int taotdm_core_pwr_ready = 0;

/*
		Blizzard	Tsunami	WebCam
tvp_ctl_en	18		22		
cam_rst	98		98		98
cam_core	20				58
cam_io		aux2				57

blizzard_cam_on	rst->0, dis-other(tvp_ctl_en->1), cam_core_en,cam_io_en, rst->1
blizzard_tvp_on  	tvp_ctl_en->0 ,  when no other devices

WebCam
*/


#define CAM_USE_XCLKA		0

#define LEOPARD_RESET_GPIO		98	/*There is no signal, close for CAM Expen Header,
						we have to see if reset will turn it off complete.
						without compromise tvp514x/adv7180 operation*/

#define MULTI_INPUTS			0	/*More than one adc video source*/

#define CAM_CORE_GPIO		58	/*Pull low/high for On/Off*/
#define CAM_IO_GPIO			57	/*Pull low/high for On/Off*/

/*There is HW co-lay with adv7180, they share same control line, therefore, only one of then can exit on the PCB.*/
#define TVP514X_CONTROL_GPIO	18	/*Pull low/high for On/Off*/
#define TVP5146_DEC_RST		19	/*Pull low/high for Rest/Normal*/
#define DISABLE_OTHER_INPUTS

enum taotam_camera_power_type {
TN_CAM_IO_POWER_REGUATOR = 0,
TN_CAM_IO_POWER_GPIO_LDO,
TN_CAM_CORE_POWER_REGUATOR,
TN_CAM_CORE_POWER_GPIO_LDO,
TN_CAM_POWER_INVALID,
};

enum taotam_camera_power_name {
TN_CAM_POWER_NAME_IO,
TN_CAM_POWER_NAME_CORE,
TN_CAM_POWER_NAME_INVALID,
};

struct taotdm_camera_power_source{
	enum taotam_camera_power_type type;
	char * name;
	int	gpio;
};

struct taotdm_camera_module_power_control {
	int (*cam_io_enable) (int OnOff);
	int (*cam_core_enable) (int OnOff);
};

#if 1
/*For TDM3730*/
static struct taotdm_camera_power_source camera_io_source = {TN_CAM_IO_POWER_REGUATOR,"cam_io_1v8",0};
static struct taotdm_camera_power_source camera_core_source = {TN_CAM_CORE_POWER_GPIO_LDO,"GPIO_20",20};
#else
/*For TAO3530 WEBCAM_EVB*/
static struct taotdm_camera_power_source camera_io_source = {TN_CAM_IO_POWER_GPIO_LDO,"GPIO_57",57};
static struct taotdm_camera_power_source camera_core_source = {TN_CAM_CORE_POWER_GPIO_LDO,"GPIO_58",57};
#endif

/* mux id to enable/disable signal routing to different peripherals */
enum taotdm_cam_mux {
	MUX_EN_TVP5146 = 0,
	MUX_EN_CAMERA_SENSOR,
	MUX_EN_EXP_CAMERA_SENSOR,
	MUX_INVALID,
};

struct taotdm_camera_module_power_control cam_ctrl;

int Camera_IO_Enable(int OnOff)
{
	struct taotdm_camera_power_source source = camera_io_source;
	if(source.type == TN_CAM_IO_POWER_REGUATOR)
	{
		if(OnOff)
			regulator_enable(taotdm_io_1v8);
		else
		{
			if (regulator_is_enabled(taotdm_io_1v8))
				regulator_disable(taotdm_io_1v8);
		}
	}
	else if(source.type == TN_CAM_IO_POWER_GPIO_LDO)
	{
		if(OnOff)
			gpio_set_value(source.gpio, 0);
		else
			gpio_set_value(source.gpio, 1);

	}
	return 0;
}

int Camera_CORE_Enable(int OnOff)
{
	struct taotdm_camera_power_source source = camera_core_source;
	if(source.type == TN_CAM_CORE_POWER_REGUATOR)
	{
		if(OnOff)
			regulator_enable(taotdm_core_1v8);
		else
		{
			if (regulator_is_enabled(taotdm_core_1v8))
				regulator_disable(taotdm_core_1v8);
		}
	}
	else if(source.type == TN_CAM_CORE_POWER_GPIO_LDO)
	{
		if(OnOff)
			gpio_set_value(source.gpio, 0);
		else
			gpio_set_value(source.gpio, 1);

	}
	return 0;
}


int camera_power_init(struct taotdm_camera_power_source source)
{
	int ret = 0;
	switch(source.type)
	{
		case TN_CAM_IO_POWER_REGUATOR:
		case TN_CAM_IO_POWER_GPIO_LDO:
			cam_ctrl.cam_io_enable = Camera_IO_Enable;
			if(source.type == TN_CAM_IO_POWER_REGUATOR)
			{
				taotdm_io_1v8 = regulator_get(NULL, source.name);
				if (IS_ERR(taotdm_io_1v8)) {
					printk(KERN_ERR "vio_1v8 regulator missing\n");
					ret = PTR_ERR(taotdm_io_1v8);
				}

			}else if(source.type == TN_CAM_IO_POWER_GPIO_LDO)
			{
				if((gpio_request(source.gpio, "CAM_IO") == 0) && 
						(gpio_direction_output(source.gpio, 1) == 0)){
							gpio_export(source.gpio, 0); 
							gpio_set_value(source.gpio, 1); 
				}
				else
					ret = 1;
			}
			taotdm_io_pwr_ready = 1 ;
			break;
		case TN_CAM_CORE_POWER_REGUATOR:
		case TN_CAM_CORE_POWER_GPIO_LDO:
			cam_ctrl.cam_core_enable = Camera_CORE_Enable;
			if(source.type == TN_CAM_CORE_POWER_REGUATOR)
			{
				taotdm_core_1v8 = regulator_get(NULL, source.name);
				if (IS_ERR(taotdm_core_1v8)) {
					printk(KERN_ERR "cam_2v8 regulator missing\n");
					ret = PTR_ERR(taotdm_core_1v8);
				}
			}
			else if(source.type == TN_CAM_CORE_POWER_GPIO_LDO)
			{
				if((gpio_request(source.gpio, "CAM_CORE") == 0) && 
							(gpio_direction_output(source.gpio, 1) == 0)) { 
								gpio_export(source.gpio, 0); 
								gpio_set_value(source.gpio, 1); 
				} else 
					ret = 1;
			}
			taotdm_core_pwr_ready = 1;
			break;
		default:
			printk("Invalid type of power source.\n");

	}
	return ret;

}

void camera_power_release(struct taotdm_camera_power_source source)
{
	switch(source.type)
	{
		case TN_CAM_IO_POWER_REGUATOR:
		case TN_CAM_IO_POWER_GPIO_LDO:
			if(source.type == TN_CAM_IO_POWER_REGUATOR)
			{
				if (regulator_is_enabled(taotdm_io_1v8))
					regulator_disable(taotdm_io_1v8);
				regulator_put(taotdm_io_1v8);
			}
			else if(source.type == TN_CAM_IO_POWER_GPIO_LDO)
			{
				gpio_free(source.gpio);
			}
			taotdm_io_pwr_ready = 0;
			break;
		case TN_CAM_CORE_POWER_REGUATOR:
		case TN_CAM_CORE_POWER_GPIO_LDO:
			if(source.type == TN_CAM_CORE_POWER_REGUATOR)
			{
				if (regulator_is_enabled(taotdm_core_1v8))
					regulator_disable(taotdm_core_1v8);
				regulator_put(taotdm_core_1v8);
			}
			else if(source.type == TN_CAM_CORE_POWER_GPIO_LDO)
			{
				gpio_free(source.gpio);
			}
			taotdm_core_pwr_ready = 0;
			break;
		default:
			printk("Invalid type of power source.\n");
	}
}

static int taotdm_regulator_ctrl(u32 on)
{

	if((taotdm_io_pwr_ready == 0)||(taotdm_core_pwr_ready == 0)){
		printk(KERN_ERR "No regulator available\n");
		return -ENODEV;
	}
	
	if((cam_ctrl.cam_io_enable == NULL)||(cam_ctrl.cam_core_enable == NULL)){
		printk(KERN_ERR "Control Function Not Attached\n");
		return -ENODEV;
	}

	if (on) {
			cam_ctrl.cam_io_enable(1);
			/* Turn on VDD */
			mdelay(1);
			cam_ctrl.cam_core_enable(1);
			mdelay(50);
		} else {
			/*
			 * Power Down Sequence
			 */
			cam_ctrl.cam_io_enable(0);
			cam_ctrl.cam_core_enable(0);			
		}

	return 0;
}

/**
 * @brief taotdm_set_mux - Sets mux to enable/disable signal routing to
 *                             different peripherals present on new EVM board
 *
 * @param mux_id - enum, mux id to enable/disable
 * @param value - enum, ENABLE_MUX for enabling and DISABLE_MUX for disabling
 *
 */
static void taotdm_set_mux(enum taotdm_cam_mux mux_id)
{
	switch (mux_id) {
		/*
		 * 
		 * 
		 */
		case MUX_EN_CAMERA_SENSOR:
			/*  */
			gpio_set_value(TVP514X_CONTROL_GPIO, 1);
			gpio_set_value(LEOPARD_RESET_GPIO, 1);
			break;

		case MUX_EN_TVP5146:
			/*  */
			gpio_set_value(TVP514X_CONTROL_GPIO, 0);
			gpio_set_value(TVP5146_DEC_RST, 1);
			gpio_set_value(LEOPARD_RESET_GPIO, 0);
			break;
		default:
			break;
	}
}

/* MT9T111: 3M sensor */

static int taotdm_mt9t111_s_power(struct v4l2_subdev *subdev, u32 on)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);
	int ret;

	ret = taotdm_regulator_ctrl(on);
	if (ret)
		return ret;

	taotdm_set_mux(MUX_EN_CAMERA_SENSOR);

	if (on) {
		/* Enable EXTCLK */
		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 24000000, CAM_USE_XCLKA);
		udelay(5);
	} else {
		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 0, CAM_USE_XCLKA);
	}

	return 0;
}

static struct mt9t111_platform_data taotdm_mt9t111_platform_data = {
	.s_power		= taotdm_mt9t111_s_power,
};

/* TVP5146: Video Decoder */

static int taotdm_tvp514x_s_power(struct v4l2_subdev *subdev, u32 on)
{
	int ret;

	ret = taotdm_regulator_ctrl(on);
	if (ret)
		return ret;

	taotdm_set_mux(MUX_EN_TVP5146);

	return 0;
}

static struct tvp514x_platform_data taotdm_tvp514x_platform_data = {
	.s_power		= taotdm_tvp514x_s_power,
};

#if defined(CONFIG_VIDEO_MT9P031) || defined(CONFIG_VIDEO_MT9P031_MODULE)
static int taotdm_mt9p031_s_power(struct v4l2_subdev *subdev, u32 on)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);
	int ret;

	if((taotdm_io_pwr_ready == 0)||(taotdm_core_pwr_ready == 0)){
		printk(KERN_ERR "No regulator available\n");
		return -ENODEV;
	}
	
	if((cam_ctrl.cam_io_enable == NULL)||(cam_ctrl.cam_core_enable == NULL)){
		printk(KERN_ERR "Control Function Not Attached\n");
		return -ENODEV;
	}

	taotdm_set_mux(MUX_EN_CAMERA_SENSOR);

	if (on) {
		/*
		 * Power Up Sequence
		 */
		/* Set RESET_BAR to 0 */
		gpio_set_value(LEOPARD_RESET_GPIO, 0);

		/* Turn on VDD */
		cam_ctrl.cam_io_enable(1);
		mdelay(1);

		/* Turn on VDD */
		cam_ctrl.cam_core_enable(1);
		mdelay(50);

		/* Enable EXTCLK */
		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 24000000, CAM_USE_XCLKA);
		/*
		 * Wait at least 70 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 70) / 24000000) = aprox 3 us.
		 */
		udelay(3);
		/* Set RESET_BAR to 1 */
		gpio_set_value(LEOPARD_RESET_GPIO, 1);
		/*
		 * Wait at least 100 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 100) / 24000000) = aprox 5 us.
		 */
		udelay(5);
	} else {
		/*
		 * Power Down Sequence
		 */
		cam_ctrl.cam_io_enable(0);
		cam_ctrl.cam_core_enable(0);
		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 0, CAM_USE_XCLKA);
	}

	return 0;
}

static int taotdm_mt9p031_configure_interface(struct v4l2_subdev *subdev,
					      u32 pixclk)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);

	if (isp->platform_cb.set_pixel_clock)
		isp->platform_cb.set_pixel_clock(isp, pixclk);

	return 0;
}

static struct mt9p031_platform_data taotdm_mt9p031_platform_data = {
	.s_power		= taotdm_mt9p031_s_power,
	.configure_interface	= taotdm_mt9p031_configure_interface,
};
#endif/*CONFIG_VIDEO_MT9P031||CONFIG_VIDEO_MT9P031_MODULE*/

#define MT9T111_I2C_BUS_NUM		2
#define TVP514X_I2C_BUS_NUM		2
#define MT9P031_I2C_BUS_NUM		2

static struct i2c_board_info taotdm_camera_i2c_devices[] = {
	{
		I2C_BOARD_INFO(MT9T111_MODULE_NAME, MT9T111_I2C_ADDR),
		.platform_data = &taotdm_mt9t111_platform_data,
	},
	{
		I2C_BOARD_INFO("tvp5146m2", 0x5D),
		.platform_data	= &taotdm_tvp514x_platform_data,
	},
#if defined(CONFIG_VIDEO_MT9P031) || defined(CONFIG_VIDEO_MT9P031_MODULE)
	{
		I2C_BOARD_INFO(MT9P031_MODULE_NAME, MT9P031_I2C_ADDR),
		.platform_data = &taotdm_mt9p031_platform_data,
	},
#endif
};

static struct isp_subdev_i2c_board_info taotdm_mt9t111_subdevs[] = {
	{
		.board_info = &taotdm_camera_i2c_devices[0],
		.i2c_adapter_id = MT9T111_I2C_BUS_NUM,
	},
	{ NULL, 0 },
};

static struct isp_subdev_i2c_board_info taotdm_tvp514x_subdevs[] = {
	{
		.board_info	= &taotdm_camera_i2c_devices[1],
		.i2c_adapter_id	= TVP514X_I2C_BUS_NUM,
	},
	{ NULL, 0 },
};

#if defined(CONFIG_VIDEO_MT9P031) || defined(CONFIG_VIDEO_MT9P031_MODULE)
static struct isp_subdev_i2c_board_info taotdm_mtp031_subdevs[] = {
	{
		.board_info = &taotdm_camera_i2c_devices[0],
		.i2c_adapter_id = MT9P031_I2C_BUS_NUM,
	},
	{ NULL, 0 },
};
#endif

static struct isp_v4l2_subdevs_group taotdm_camera_subdevs[] = {
	{
		.subdevs = taotdm_mt9t111_subdevs,
		.interface = ISP_INTERFACE_PARALLEL,
		.bus = {
			.parallel = {
				.data_lane_shift	= 1,
				.clk_pol		= 0,
				.hdpol			= 0,
				.vdpol			= 0,
				.fldmode		= 0,
				.bridge		= 3,
				.is_bt656		= 0,
			},
		},
	},
	{
		.subdevs	= taotdm_tvp514x_subdevs,
		.interface	= ISP_INTERFACE_PARALLEL,
		.bus		= {
			.parallel	= {
				.width			= 8,
				.data_lane_shift	= 1,
				.clk_pol		= 0,
				.hdpol			= 0,
				.vdpol			= 1,
				.fldmode		= 1,
				.bridge		= 0,
				.is_bt656		= 1,
			},
		},
	},
#if defined(CONFIG_VIDEO_MT9P031) || defined(CONFIG_VIDEO_MT9P031_MODULE)
	{
		.subdevs = taotdm_mt9p031_subdevs,
		.interface = ISP_INTERFACE_PARALLEL,
		.bus = {
			.parallel = {
				.data_lane_shift	= 2,
				.clk_pol		= 0,
				.bridge		= 3,
			},
		},
	},
#endif
	{ NULL, 0 },
};

static struct isp_platform_data taotdm_isp_platform_data = {
	.subdevs = taotdm_camera_subdevs,
};

static int __init taotdm_cam_init(void)
{
	int ret = 0;

	/*
	 * Regulator supply required for camera interface
	 */

	ret = camera_power_init(camera_io_source);
	if(ret)
		goto err_1;
	ret = camera_power_init(camera_core_source);
	if(ret)
		goto err_2;

	/*CAM_FLD(GPIO_98) as reset for CAM Exp. Header*/	
	omap_mux_init_gpio(LEOPARD_RESET_GPIO, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(LEOPARD_RESET_GPIO, "sensore reset") < 0) {
		printk(KERN_ERR "failed to get GPIO98_VID_SENSORE_RESET\n");
		goto err_3;
	}
	/* Assert the reset signal */
	gpio_direction_output(LEOPARD_RESET_GPIO, 0);
	mdelay(5);
	gpio_set_value(LEOPARD_RESET_GPIO, 1);

	/*GPIO_19 as reset for tvp514x/adv7180*/	
	omap_mux_init_gpio(TVP5146_DEC_RST, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(TVP5146_DEC_RST, "vid-dec reset") < 0) {
		printk(KERN_ERR "failed to get GPIO19_VID_DEC_RES\n");
		goto err_4;
	}
	/* Assert the reset signal */
	gpio_direction_output(TVP5146_DEC_RST, 0);
	mdelay(5);
	gpio_set_value(TVP5146_DEC_RST, 1);

	/*GPIO_18 to close singal from tvp514x/adv7180 */	
	omap_mux_init_gpio(TVP514X_CONTROL_GPIO, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(TVP514X_CONTROL_GPIO, "vid-dec datain") < 0) {
		printk(KERN_ERR "failed to get GPIO18_VID_DATA_IN\n");
		goto err_5;
	}
	/* Set the OE to Off state */
	gpio_direction_output(TVP514X_CONTROL_GPIO, 1);

	omap3_init_camera(&taotdm_isp_platform_data);

	printk(KERN_INFO "taotdm camera init done successfully...\n");
	return 0;

err_5:
	gpio_free(TVP514X_CONTROL_GPIO);
err_4:
	gpio_free(TVP5146_DEC_RST);
err_3:
	gpio_free(LEOPARD_RESET_GPIO);
err_2:
	camera_power_release(camera_core_source);
err_1:
	camera_power_release(camera_io_source);

	return ret;
}

static void __exit taotdm_cam_exit(void)
{
	gpio_free(TVP514X_CONTROL_GPIO);
	gpio_free(TVP5146_DEC_RST);
	gpio_free(LEOPARD_RESET_GPIO);
	camera_power_release(camera_core_source);
	camera_power_release(camera_io_source);
}

module_init(taotdm_cam_init);
module_exit(taotdm_cam_exit);

MODULE_AUTHOR("Edward Lin <linuxfae@technexion.com>");
MODULE_DESCRIPTION("Taotdm Camera Module");
MODULE_LICENSE("GPL");
