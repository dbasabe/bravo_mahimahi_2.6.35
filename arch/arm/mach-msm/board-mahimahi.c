/* linux/arch/arm/mach-msm/board-mahimahi.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Author: Dima Zavin <dima@android.com>
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

#include <linux/cy8c_tmg_ts.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-msm.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/usb/android_composite.h>

#include <linux/android_pmem.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/a1026.h>
#include <linux/capella_cm3602.h>
#include <linux/akm8973.h>
#include <linux/regulator/machine.h>
#include <linux/ds2784_battery.h>
#include <../../../drivers/staging/android/timed_gpio.h>
#include <../../../drivers/w1/w1.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <linux/bma150.h>
//#include <mach/atmega_microp.h>
#include <mach/vreg.h>
#include <mach/msm_panel.h>
#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_iomap.h>
#include <mach/msm_serial_debugger.h>
#include <mach/system.h>
#include <mach/msm_serial_hs.h>
#include <mach/bcm_bt_lpm.h>
#include <mach/msm_smd.h>
#include <mach/msm_flashlight.h>
#include <mach/perflock.h>
#include <mach/htc_usb.h>
#include <mach/board_htc.h>

#include "board-mahimahi.h"
#include "devices.h"
#include "proc_comm.h"
#include "board-mahimahi-tpa2018d1.h"
#include "board-mahimahi-smb329.h"
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_microp.h>

#define SMEM_SPINLOCK_I2C      6



#define SAMSUNG_PANEL           0
/*Bitwise mask for SONY PANEL ONLY*/
#define SONY_PANEL              0x1             /*Set bit 0 as 1 when it is SONY PANEL*/
#define SONY_PWM_SPI            0x2             /*Set bit 1 as 1 as PWM_SPI mode, otherwise it is PWM_MICROP mode*/
#define SONY_GAMMA              0x4             /*Set bit 2 as 1 when panel contains GAMMA table in its NVM*/
#define SONY_RGB666             0x8             /*Set bit 3 as 1 when panel is 18 bit, otherwise it is 16 bit*/






#ifdef CONFIG_ARCH_QSD8X50
extern unsigned char *get_bt_bd_ram(void);
#endif



static uint debug_uart;

module_param_named(debug_uart, debug_uart, uint, 0);

extern void notify_usb_connected(int);
void msm_init_pmic_vibrator(int);

extern void __init mahimahi_audio_init(void);

extern int microp_headset_has_mic(void);


#ifdef CONFIG_MICROP_COMMON
void __init mahimahi_microp_init(void);
#endif




static int mahimahi_phy_init_seq[] = {
	0x0C, 0x31,
	0x31, 0x32,
	0x1D, 0x0D,
	0x1D, 0x10,
	-1 };


/*static struct htc_battery_platform_data htc_battery_pdev_data = {
        .func_show_batt_attr = htc_battery_show_attr,
        .gpio_mbat_in = MAHIMAHI_GPIO_MBAT_IN,
        .gpio_mchg_en_n = MAHIMAHI_GPIO_MCHG_EN_N,
        .gpio_iset = MAHIMAHI_GPIO_ISET,
        .guage_driver = GUAGE_DS2784,
        .charger = LINEAR_CHARGER,
        .m2a_cable_detect = 1,
};*/


static struct htc_battery_platform_data htc_battery_pdev_data = {
        .func_show_batt_attr = htc_battery_show_attr,
        .gpio_mbat_in = MAHIMAHI_GPIO_MBAT_IN,
        .gpio_mchg_en_n = MAHIMAHI_GPIO_MCHG_EN_N,
        .gpio_iset = MAHIMAHI_GPIO_ISET,
        .guage_driver = GUAGE_DS2784,
        .charger = LINEAR_CHARGER,
        .m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
        .name = "htc_battery",
        .id = -1,
        .dev    = {
                .platform_data = &htc_battery_pdev_data,
        },
};




//static int capella_cm3602_power(int pwr_device, uint8_t enable);

/*static struct microp_function_config microp_functions[] = {
        {
                .name   = "microp_intrrupt",
                .category = MICROP_FUNCTION_INTR,
        },
        {
                .name   = "sdcard-detect",
                .category = MICROP_FUNCTION_SDCARD,
                .int_pin = 1 << 0,
                .mask_r = {0x80, 0x00, 0x00},
        },*/
/*        {
                .name   = "oj",
                .category = MICROP_FUNCTION_OJ,
                .int_pin = 1 << 12,
        },*/
//};




/*static struct microp_function_config microp_lightsensor_function = {
        .name = "light_sensor",
        .category = MICROP_FUNCTION_LSENSOR,
        .levels = { 0, 0x21, 0x4D, 0xDC, 0x134, 0x18D, 0x1E5, 0x3FF, 0x3FF, 0x3FF },
        .channel = 6,
        .int_pin = 1 << 9,
        .golden_adc = 0xC0,
        .ls_power = capella_cm3602_power,
};

static struct lightsensor_platform_data lightsensor_data = {
        .config = &microp_lightsensor_function,
        .irq = MSM_uP_TO_INT(9),
};*/

/*static struct microp_led_config led_config[] = {
        {
                .name = "amber",
                .type = LED_RGB,
        },
        {
                .name = "green",
                .type = LED_RGB,
        },
        {
                .name = "blue",
                .type = LED_GPO,
                .mask_w = {0x00, 0x02, 0x00},
        },
        {
                .name = "button-backlight",
                .type = LED_PWM,
                .mask_w = {0x02, 0x00, 0x00},
                .led_pin = 1 << 2,
                .init_value = 0xFF,
                .fade_time = 5,
        },
};

static struct microp_led_platform_data microp_leds_data = {
        .num_leds       = ARRAY_SIZE(led_config),
        .led_config     = led_config,
};*/

/*static struct bma150_platform_data mahimahi_g_sensor_pdata = {
        .microp_new_cmd = 1,
        .chip_layout = 1,
};*/



/*static struct platform_device microp_devices[] = {
        {
                .name = "lightsensor_microp",
                .dev = {
                        .platform_data = &lightsensor_data,
                },
        },*/
/*        {
                .name = "leds-microp",
                .id = -1,
                .dev = {
                        .platform_data = &microp_leds_data,
                },
        },*/
/*        {
                .name = BMA150_G_SENSOR_NAME,
                .dev = {
                        .platform_data = &mahimahi_g_sensor_pdata,
                },
        },*/
/*        {
                .name   = "HTC_HEADSET_MGR",
                .id     = -1,
                .dev    = {
                        .platform_data  = &htc_headset_mgr_data,
                },
        },*/
//};

/*static struct microp_i2c_platform_data microp_data = {
        .num_functions   = ARRAY_SIZE(microp_functions),
        .microp_function = microp_functions,
        .num_devices = ARRAY_SIZE(microp_devices),
        .microp_devices = microp_devices,
        .gpio_reset = MAHIMAHI_GPIO_UP_RESET_N,
//        .spi_devices = SPI_OJ | SPI_GSENSOR,
	.spi_devices = SPI_GSENSOR,
};*/




static struct resource qsd_spi_resources[] = {
        {
                .name   = "spi_irq_in",
                .start  = INT_SPI_INPUT,
                .end    = INT_SPI_INPUT,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .name   = "spi_irq_out",
                .start  = INT_SPI_OUTPUT,
                .end    = INT_SPI_OUTPUT,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .name   = "spi_irq_err",
                .start  = INT_SPI_ERROR,
                .end    = INT_SPI_ERROR,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .name   = "spi_base",
                .start  = 0xA1200000,
                .end    = 0xA1200000 + SZ_4K - 1,
                .flags  = IORESOURCE_MEM,
        },
        {
                .name   = "spi_clk",
                .start  = 17,
                .end    = 1,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .name   = "spi_mosi",
                .start  = 18,
                .end    = 1,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .name   = "spi_miso",
                .start  = 19,
                .end    = 1,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .name   = "spi_cs0",
                .start  = 20,
                .end    = 1,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .name   = "spi_pwr",
                .start  = 21,
                .end    = 0,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .name   = "spi_irq_cs0",
                .start  = 22,
                .end    = 0,
                .flags  = IORESOURCE_IRQ,
        },
};

static struct platform_device qsd_device_spi = {
        .name           = "spi_qsd",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(qsd_spi_resources),
        .resource       = qsd_spi_resources,
};





/*
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= mahimahi_phy_init_seq,
	.phy_reset		= mahimahi_usb_phy_reset,
	.hw_reset		= mahimahi_usb_hw_reset,
	.usb_connected		= notify_usb_connected,
};*/


/*static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

#ifdef CONFIG_USB_ANDROID_DIAG
static char *usb_functions_adb_diag[] = {
	"usb_mass_storage",
	"adb",
	"diag",
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
	"usb_mass_storage",
	"adb",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x4e11,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x4e12,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x4e13,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x4e14,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		.product_id	= 0x4e17,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag),
		.functions	= usb_functions_adb_diag,
	},
#endif
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "Google, Inc.",
	.product	= "Nexus One",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {*/
	/* ethaddr is filled by board_serialno_setup */
/*	.vendorID	= 0x18d1,
	.vendorDescr	= "Google, Inc.",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x18d1,
	.product_id	= 0x4e11,
	.version	= 0x0100,
	.product_name		= "Nexus One",
	.manufacturer_name	= "Google, Inc.",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};*/






static uint opt_usb_h2w_sw;
module_param_named(usb_h2w_sw, opt_usb_h2w_sw, uint, 0);

static uint32_t usb_phy_3v3_table[] = {
        PCOM_GPIO_CFG(MAHIMAHI_USB_PHY_3V3_ENABLE, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)
};

static uint32_t usb_ID_PIN_input_table[] = {
        PCOM_GPIO_CFG(MAHIMAHI_GPIO_USB_ID_PIN, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),
        PCOM_GPIO_CFG(MAHIMAHI_GPIO_USB_ID1_PIN, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),
};

static uint32_t usb_ID_PIN_ouput_table[] = {
        PCOM_GPIO_CFG(MAHIMAHI_GPIO_USB_ID_PIN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
};


void config_mahimahi_usb_id_gpios(bool output)
{
        if (output) {
                config_gpio_table(usb_ID_PIN_ouput_table, ARRAY_SIZE(usb_ID_PIN_ouput_table));
                gpio_set_value(MAHIMAHI_GPIO_USB_ID_PIN, 1);
                printk(KERN_INFO "%s %d output high\n",  __func__, MAHIMAHI_GPIO_USB_ID_PIN);
        } else {
                config_gpio_table(usb_ID_PIN_input_table, ARRAY_SIZE(usb_ID_PIN_input_table));
                printk(KERN_INFO "%s %d input none pull\n",  __func__, MAHIMAHI_GPIO_USB_ID_PIN);
        }
}

void mahimahi_usb_phy_reset(void)
{
        printk(KERN_INFO "%s\n", __func__);
        msm_hsusb_8x50_phy_reset();
        if (usb_phy_error) {
                printk(KERN_INFO "%s: power cycle usb phy\n",
                        __func__);
                gpio_set_value(MAHIMAHI_USB_PHY_3V3_ENABLE, 0);
                mdelay(10);
                gpio_set_value(MAHIMAHI_USB_PHY_3V3_ENABLE, 1);
                mdelay(10);
                msm_hsusb_8x50_phy_reset();
        }
}


#ifdef CONFIG_USB_ANDROID
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
        .phy_init_seq           = mahimahi_phy_init_seq,
        .phy_reset              = mahimahi_usb_phy_reset,
        .usb_id_pin_gpio =  MAHIMAHI_GPIO_USB_ID_PIN,
        .config_usb_id_gpios = config_mahimahi_usb_id_gpios,
        .accessory_detect = 1, /* detect by ID pin gpio */
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
        .nluns          = 1,
        .vendor         = "HTC",
        .product        = "Android Phone",
        .release        = 0x0100,
};

static struct platform_device usb_mass_storage_device = {
        .name   = "usb_mass_storage",
        .id     = -1,
        .dev    = {
                .platform_data = &mass_storage_pdata,
        },
};


static struct android_usb_platform_data android_usb_pdata = {
        .vendor_id      = 0x0bb4,
        .product_id     = 0x0c87,
        .version        = 0x0100,
        .product_name           = "Android Phone",
        .manufacturer_name      = "HTC",
        .num_products = ARRAY_SIZE(usb_products),
        .products = usb_products,
        .num_functions = ARRAY_SIZE(usb_functions_all),
        .functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
        .name   = "android_usb",
        .id             = -1,
        .dev            = {
                .platform_data = &android_usb_pdata,
        },
};

static void mahimahi_add_usb_devices(void)
{
        android_usb_pdata.products[0].product_id =
                android_usb_pdata.product_id;
        android_usb_pdata.serial_number = board_serialno();
        msm_hsusb_pdata.serial_number = board_serialno();
        msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
        config_mahimahi_usb_id_gpios(0);
        config_gpio_table(usb_phy_3v3_table, ARRAY_SIZE(usb_phy_3v3_table));
        gpio_set_value(MAHIMAHI_USB_PHY_3V3_ENABLE, 1);
        platform_device_register(&msm_device_hsusb);
        platform_device_register(&usb_mass_storage_device);
        platform_device_register(&android_usb_device);
}
#endif











static struct platform_device mahimahi_rfkill = {
	.name = "mahimahi_rfkill",
	.id = -1,
};

static struct vreg *vreg_lcm_rftx_2v6;
static struct vreg *vreg_lcm_aux_2v6;


static struct resource msm_kgsl_resources[] = {
	{
		.name	= "kgsl_reg_memory",
		.start	= MSM_GPU_REG_PHYS,
		.end	= MSM_GPU_REG_PHYS + MSM_GPU_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "kgsl_phys_memory",
		.start	= MSM_GPU_MEM_BASE,
		.end	= MSM_GPU_MEM_BASE + MSM_GPU_MEM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
	},
};

#define PWR_RAIL_GRP_CLK		8
static int mahimahi_kgsl_power_rail_mode(int follow_clk)
{
	int mode = follow_clk ? 0 : 1;
	int rail_id = PWR_RAIL_GRP_CLK;

	return msm_proc_comm(PCOM_CLKCTL_RPC_RAIL_CONTROL, &rail_id, &mode);
}

static int mahimahi_kgsl_power(bool on)
{
	int cmd;
	int rail_id = PWR_RAIL_GRP_CLK;

	cmd = on ? PCOM_CLKCTL_RPC_RAIL_ENABLE : PCOM_CLKCTL_RPC_RAIL_DISABLE;
	return msm_proc_comm(cmd, &rail_id, NULL);
}

static struct platform_device msm_kgsl_device = {
	.name		= "kgsl",
	.id		= -1,
	.resource	= msm_kgsl_resources,
	.num_resources	= ARRAY_SIZE(msm_kgsl_resources),
};

static struct android_pmem_platform_data mdp_pmem_pdata = {
	.name		= "pmem",
	.start		= MSM_PMEM_MDP_BASE,
	.size		= MSM_PMEM_MDP_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name		= "pmem_adsp",
	.start		= MSM_PMEM_ADSP_BASE,
	.size		= MSM_PMEM_ADSP_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_venc_pdata = {
        .name           = "pmem_venc",
        .start          = MSM_PMEM_VENC_BASE,
        .size           = MSM_PMEM_VENC_SIZE,
        .no_allocator   = 0,
        .cached         = 1,
};

static struct platform_device android_pmem_mdp_device = {
	.name		= "android_pmem",
	.id		= 0,
	.dev		= {
		.platform_data = &mdp_pmem_pdata
	},
};

static struct platform_device android_pmem_adsp_device = {
	.name		= "android_pmem",
	.id		= 1,
	.dev		= {
		.platform_data = &android_pmem_adsp_pdata,
	},
};

static struct platform_device android_pmem_venc_device = {
        .name           = "android_pmem",
        .id             = 3,
        .dev            = {
                .platform_data = &android_pmem_venc_pdata,
        },
};

static struct resource ram_console_resources[] = {
	{
		.start	= MSM_RAM_CONSOLE_BASE,
		.end	= MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};


static int mahimahi_ts_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	if (on) {
		/* level shifter should be off */
		gpio_set_value(MAHIMAHI_GPIO_TP_EN, 1);
		msleep(120);
		/* enable touch panel level shift */
		gpio_set_value(MAHIMAHI_GPIO_TP_LS_EN, 1);
		msleep(3);
	} else {
		gpio_set_value(MAHIMAHI_GPIO_TP_LS_EN, 0);
		gpio_set_value(MAHIMAHI_GPIO_TP_EN, 0);
		udelay(50);
	}

	return 0;
}

struct cy8c_i2c_platform_data mahimahi_cy8c_ts_data = {
	.version = 0x0001,
	.abs_x_min = 0,
	.abs_x_max = 479,
	.abs_y_min = 0,
	.abs_y_max = 799,
	.abs_pressure_min = 0,
	.abs_pressure_max = 255,
	.abs_width_min = 0,
	.abs_width_max = 10,
	.power = mahimahi_ts_power,
};

static struct synaptics_i2c_rmi_platform_data mahimahi_synaptics_ts_data[] = {
	{
		.version = 0x105,
		.power = mahimahi_ts_power,
		.flags = SYNAPTICS_FLIP_Y,
		.inactive_left = -15 * 0x10000 / 480,
		.inactive_right = -15 * 0x10000 / 480,
		.inactive_top = -15 * 0x10000 / 800,
		.inactive_bottom = -50 * 0x10000 / 800,
		.sensitivity_adjust = 9,
	},
	{
		.flags = SYNAPTICS_FLIP_Y,
		.inactive_left = -15 * 0x10000 / 480,
		.inactive_right = -15 * 0x10000 / 480,
		.inactive_top = -15 * 0x10000 / 800,
		.inactive_bottom = -40 * 0x10000 / 800,
		.sensitivity_adjust = 12,
	},
};

static struct a1026_platform_data a1026_data = {
	.gpio_a1026_micsel = MAHIMAHI_AUD_MICPATH_SEL,
	.gpio_a1026_wakeup = MAHIMAHI_AUD_A1026_WAKEUP,
	.gpio_a1026_reset = MAHIMAHI_AUD_A1026_RESET,
	.gpio_a1026_clk = MAHIMAHI_AUD_A1026_CLK,
	/*.gpio_a1026_int = MAHIMAHI_AUD_A1026_INT,*/
};

static struct akm8973_platform_data compass_platform_data = {
	.layouts = MAHIMAHI_LAYOUTS,
	.project_name = MAHIMAHI_PROJECT_NAME,
	.reset = MAHIMAHI_GPIO_COMPASS_RST_N,
	.intr = MAHIMAHI_GPIO_COMPASS_INT_N,
};

static uint32_t key_int_shutdown_gpio_table[] = {
        PCOM_GPIO_CFG(MAHIMAHI_GPIO_35MM_KEY_INT_SHUTDOWN, 0,
                GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static void headset_init(void)
{
        config_gpio_table(key_int_shutdown_gpio_table,
                ARRAY_SIZE(key_int_shutdown_gpio_table));
        gpio_set_value(MAHIMAHI_GPIO_35MM_KEY_INT_SHUTDOWN, 0);
        return;
}


static struct regulator_consumer_supply tps65023_dcdc1_supplies[] = {
	{
		.supply = "acpu_vcore",
	},
};

static struct regulator_init_data tps65023_data[5] = {
	{
		.constraints = {
			.name = "dcdc1", /* VREG_MSMC2_1V29 */
			.min_uV = 800000,
			.max_uV = 1325000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		},
		.consumer_supplies = tps65023_dcdc1_supplies,
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc1_supplies),
	},
        /* dummy values for unused regulators to not crash driver: */
        {
                .constraints = {
                        .name = "dcdc2", /* VREG_MSMC1_1V26 */
                        .min_uV = 1260000,
                        .max_uV = 1260000,
                },
        },
        {
                .constraints = {
                        .name = "dcdc3", /* unused */
                        .min_uV = 800000,
                        .max_uV = 3300000,
                },
        },
        {
                .constraints = {
                        .name = "ldo1", /* unused */
                        .min_uV = 1000000,
                        .max_uV = 3150000,
                },
        },
        {
                .constraints = {
                        .name = "ldo2", /* V_USBPHY_3V3 */
                        .min_uV = 3300000,
                        .max_uV = 3300000,
                },
        },
};


static void ds2482_set_slp_n(unsigned n)
{
	gpio_direction_output(MAHIMAHI_GPIO_DS2482_SLP_N, n);
}

static struct tpa2018d1_platform_data tpa2018_data = {
	.gpio_tpa2018_spk_en = MAHIMAHI_CDMA_GPIO_AUD_SPK_AMP_EN,
};

static struct i2c_board_info base_i2c_devices[] = {
        {
                I2C_BOARD_INFO("ds2482", 0x30 >> 1),
                .platform_data = ds2482_set_slp_n,
        },
        {
                I2C_BOARD_INFO("cy8c-tmg-ts", 0x34),
                .platform_data = &mahimahi_cy8c_ts_data,
                .irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_TP_INT_N),
        },
        {
                I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x40),
                .platform_data = mahimahi_synaptics_ts_data,
                .irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_TP_INT_N)
        },
        {
                I2C_BOARD_INFO("mahimahi-microp", 0x66),
                .irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_UP_INT_N)
        },
        {
                I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
        },
        {
                I2C_BOARD_INFO("tps65023", 0x48),
                .platform_data = tps65023_data,
        },
};

static struct i2c_board_info rev0_i2c_devices[] = {
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(MAHIMAHI_REV0_GPIO_COMPASS_INT_N),
	},
};

static struct i2c_board_info rev1_i2c_devices[] = {
	{
		I2C_BOARD_INFO("audience_a1026", 0x3E),
		.platform_data = &a1026_data,
		/*.irq = MSM_GPIO_TO_INT(MAHIMAHI_AUD_A1026_INT)*/
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_COMPASS_INT_N),
	},
};

static struct i2c_board_info rev_CX_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tpa2018d1", 0x58),
		.platform_data = &tpa2018_data,
	},
	{
		I2C_BOARD_INFO("smb329", 0x6E >> 1),
	},
};








#ifdef CONFIG_ARCH_QSD8X50
static char bdaddress[20];

static void bt_export_bd_address(void)
 {
        unsigned char cTemp[6];

        memcpy(cTemp, get_bt_bd_ram(), 6);
        sprintf(bdaddress, "%02x:%02x:%02x:%02x:%02x:%02x", cTemp[0], cTemp[1],cTemp[2],cTemp[3],cTemp[4],cTemp[5]);
        printk(KERN_INFO "YoYo--BD_ADDRESS=%s\n", bdaddress);
}

module_param_string(bdaddress, bdaddress, sizeof(bdaddress), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bdaddress, "BT MAC ADDRESS");
#endif


static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* MCLK */
};

void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] = {
	{
		.start	= MSM_VFE_PHYS,
		.end	= MSM_VFE_PHYS + MSM_VFE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		 INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static int flashlight_control(int mode)
{
        return aat1271_flashlight_control(mode);
}

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
        .camera_flash           = flashlight_control,
        .num_flash_levels       = FLASHLIGHT_NUM,
        .low_temp_limit         = 5,
        .low_cap_limit          = 15,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name = "s5k3e2fx",
	.sensor_reset = 144, /* CAM1_RST */
	.sensor_pwd = 143,  /* CAM1_PWDN, enabled in a9 */
	/*.vcm_pwd = 31, */  /* CAM1_VCM_EN, enabled in a9 */
	.pdata = &msm_camera_device_data,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.flash_cfg      = &msm_camera_sensor_flash_cfg,
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name      = "msm_camera_s5k3e2fx",
	.dev      = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};

static int capella_cm3602_power(int on)
{
	/* TODO eolsen Add Voltage reg control */
	if (on) {
		gpio_direction_output(MAHIMAHI_GPIO_PROXIMITY_EN, 0);
	} else {
		gpio_direction_output(MAHIMAHI_GPIO_PROXIMITY_EN, 1);
	}

	return 0;
}


static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.power = capella_cm3602_power,
	.p_out = MAHIMAHI_GPIO_PROXIMITY_INT_N
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.id = -1,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};


static uint32_t flashlight_gpio_table[] = {
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_FLASHLIGHT_TORCH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_FLASHLIGHT_FLASH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
};

static uint32_t flashlight_gpio_table_rev_CX[] = {
	PCOM_GPIO_CFG(MAHIMAHI_CDMA_GPIO_FLASHLIGHT_TORCH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_FLASHLIGHT_FLASH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
};


static void config_mahimahi_flashlight_gpios(void)
{
	if (is_cdma_version(system_rev)) {
		config_gpio_table(flashlight_gpio_table_rev_CX,
			ARRAY_SIZE(flashlight_gpio_table_rev_CX));
	} else {
		config_gpio_table(flashlight_gpio_table,
			ARRAY_SIZE(flashlight_gpio_table));
	}
}

static struct flashlight_platform_data mahimahi_flashlight_data = {
	.gpio_init  = config_mahimahi_flashlight_gpios,
	.torch = MAHIMAHI_GPIO_FLASHLIGHT_TORCH,
	.flash = MAHIMAHI_GPIO_FLASHLIGHT_FLASH,
	.flash_duration_ms = 600
};

static struct platform_device mahimahi_flashlight_device = {
	.name = "flashlight",
	.dev = {
		.platform_data  = &mahimahi_flashlight_data,
	},
};

static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = MAHIMAHI_GPIO_VIBRATOR_ON,
		.max_timeout = 15000,
	},
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(timed_gpios),
	.gpios		= timed_gpios,
};

static struct platform_device android_timed_gpios = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &timed_gpio_data,
	},
};

static int amoled_power(int on)
{
        if (on) {
                gpio_set_value(MAHIMAHI_LCD_RSTz, 1);
                mdelay(25);
                gpio_set_value(MAHIMAHI_LCD_RSTz, 0);
                mdelay(10);
                gpio_set_value(MAHIMAHI_LCD_RSTz, 1);
                mdelay(20);
        } else {
                gpio_set_value(MAHIMAHI_LCD_RSTz, 0);
        }
        return 0;
}

static int sonywvga_power(int on)
{
        unsigned id, on_off;

        if (on) {
                on_off = 0;

                vreg_enable(vreg_lcm_aux_2v6);
                vreg_enable(vreg_lcm_rftx_2v6);

                id = PM_VREG_PDOWN_AUX_ID;
                msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);

                id = PM_VREG_PDOWN_RFTX_ID;
                msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);

                gpio_set_value(MAHIMAHI_LCD_RSTz, 1);
                mdelay(10);
                gpio_set_value(MAHIMAHI_LCD_RSTz, 0);
                udelay(500);
                gpio_set_value(MAHIMAHI_LCD_RSTz, 1);
                mdelay(10);
        } else {
                on_off = 1;

                gpio_set_value(MAHIMAHI_LCD_RSTz, 0);

                mdelay(120);

                vreg_disable(vreg_lcm_rftx_2v6);
                vreg_disable(vreg_lcm_aux_2v6);

                id = PM_VREG_PDOWN_RFTX_ID;
                msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);

                id = PM_VREG_PDOWN_AUX_ID;
                msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
        }
        return 0;
}

#define LCM_GPIO_CFG(gpio, func, str) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, str)
static uint32_t display_on_gpio_table_samsung[] = {
        LCM_GPIO_CFG(MAHIMAHI_LCD_R1, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R2, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R3, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R4, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R5, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G0, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G1, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G2, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G3, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G4, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G5, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B1, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B2, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B3, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B4, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B5, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_PCLK, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_VSYNC, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_HSYNC, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_DE, 1, GPIO_4MA),
};


static uint32_t display_off_gpio_table_samsung[] = {
        LCM_GPIO_CFG(MAHIMAHI_LCD_R1, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R2, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R3, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R4, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R5, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G0, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G1, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G2, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G3, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G4, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G5, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B1, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B2, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B3, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B4, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B5, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_PCLK, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_VSYNC, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_HSYNC, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_DE, 0, GPIO_4MA),
};

static uint32_t display_on_gpio_table_sony[] = {
        LCM_GPIO_CFG(MAHIMAHI_LCD_R1, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R2, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R3, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R4, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R5, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G0, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G1, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G2, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G3, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G4, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G5, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B1, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B2, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B3, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B4, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B5, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_PCLK, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_VSYNC, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_HSYNC, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_DE, 1, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_SPI_CLK, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_SPI_DO, 1, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_SPI_CSz, 1, GPIO_4MA),
};


static uint32_t display_off_gpio_table_sony[] = {
        LCM_GPIO_CFG(MAHIMAHI_LCD_R1, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R2, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R3, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R4, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_R5, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G0, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G1, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G2, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G3, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G4, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_G5, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B1, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B2, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B3, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B4, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_B5, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_PCLK, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_VSYNC, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_HSYNC, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_DE, 0, GPIO_8MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_SPI_CLK, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_SPI_DO, 0, GPIO_4MA),
        LCM_GPIO_CFG(MAHIMAHI_LCD_SPI_CSz, 0, GPIO_4MA),
};

static int panel_gpio_switch_samsung(int on)
{
        config_gpio_table(
                !!on ? display_on_gpio_table_samsung : display_off_gpio_table_samsung,
                ARRAY_SIZE(display_on_gpio_table_samsung));

        return 0;
}

static int panel_gpio_switch_sony(int on)
{
        config_gpio_table(
                !!on ? display_on_gpio_table_sony : display_off_gpio_table_sony,
                ARRAY_SIZE(display_on_gpio_table_sony));

        return 0;
}


static struct resource resources_msm_fb[] = {
        {
                .start = MSM_FB_BASE,
                .end = MSM_FB_BASE + MSM_FB_SIZE - 1,
                .flags = IORESOURCE_MEM,
        },
};

static struct panel_platform_data amoled_data = {
        .fb_res = &resources_msm_fb[0],
        .power = amoled_power,
        .gpio_switch = panel_gpio_switch_samsung,
};

static struct platform_device amoled_panel = {
        .name = "panel-tl2796a",
        .id = -1,
        .dev = {
                .platform_data = &amoled_data,
        },
};

static struct panel_platform_data sonywvga_data = {
        .fb_res = &resources_msm_fb[0],
        .power = sonywvga_power,
        .gpio_switch = panel_gpio_switch_sony,
};

static struct platform_device sonywvga_panel = {
        .name = "panel-sonywvga-s6d16a0x21",
        .id = -1,
        .dev = {
                .platform_data = &sonywvga_data,
        },
};







static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_BT_HOST_WAKE),
	.inject_rx_on_wakeup = 0,

	/* for bcm */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin = MAHIMAHI_GPIO_BT_WAKE,
	.host_wakeup_pin = MAHIMAHI_GPIO_BT_HOST_WAKE,
};

static int mahimahi_init_panel(void)
{
        int ret = 0;

//        if (panel_type != SAMSUNG_PANEL) {

        if (gpio_get_value(MAHIMAHI_GPIO_LCD_ID0)) {
                ret = platform_device_register(&sonywvga_panel);

                //Here query power source of sony TFT, samsung won't use these parameters
                vreg_lcm_rftx_2v6 = vreg_get(0, "rftx");
                if (IS_ERR(vreg_lcm_rftx_2v6))
                        return PTR_ERR(vreg_lcm_rftx_2v6);
                vreg_set_level(vreg_lcm_rftx_2v6, 2600);

                vreg_lcm_aux_2v6 = vreg_get(0, "gp4");
                if (IS_ERR(vreg_lcm_aux_2v6))
                        return PTR_ERR(vreg_lcm_aux_2v6);
                vreg_set_level(vreg_lcm_aux_2v6, 2600);
        }
        else
                ret = platform_device_register(&amoled_panel);

        return ret;
}


static struct platform_device *devices[] __initdata = {
/*#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
	&msm_device_uart_dm1,
	&ram_console_device,
	&mahimahi_rfkill,
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_hsusb,
	&usb_mass_storage_device,
#ifdef CONFIG_USB_ANDROID_RNDIS
	&rndis_device,
#endif
	&android_usb_device,
	&android_pmem_mdp_device,
	&android_pmem_adsp_device,
#ifdef CONFIG_720P_CAMERA
	&android_pmem_venc_device,
#endif
	&msm_kgsl_device,
	&msm_device_i2c,
	&capella_cm3602,
	&msm_camera_sensor_s5k3e2fx,
	&mahimahi_flashlight_device,*/
        &msm_device_uart1,
#ifdef CONFIG_SERIAL_MSM_HS
        &msm_device_uart_dm1,
#endif
        &htc_battery_pdev,
        &ram_console_device,
        &mahimahi_rfkill,
        &msm_device_smd,
        &msm_device_nand,
        &android_pmem_mdp_device,
        &android_pmem_adsp_device,
        &android_pmem_venc_device,
        /*&android_pmem_camera_device,*/
#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_S5K3E2FX
        &msm_camera_sensor_s5k3e2fx,
#endif
#endif
        &msm_kgsl_device,
        &msm_device_i2c,
        &mahimahi_flashlight_device,
#if defined(CONFIG_SPI_QSD)
        &qsd_device_spi,
#endif
#ifdef CONFIG_INPUT_CAPELLA_CM3602
        &capella_cm3602,
#endif


};


static uint32_t bt_gpio_table[] = {
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_CTS, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_RX, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_RESET_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_WAKE, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
};

static uint32_t bt_gpio_table_rev_CX[] = {
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_CTS, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_RX, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_RESET_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_CDMA_GPIO_BT_WAKE, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
};

static uint32_t misc_gpio_table[] = {
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_LCD_RST_N, 0, GPIO_OUTPUT,
		      GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_LED_3V3_EN, 0, GPIO_OUTPUT,
		      GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_DOCK, 0, GPIO_OUTPUT,
		      GPIO_NO_PULL, GPIO_4MA),
};

/*static uint32_t key_int_shutdown_gpio_table[] = {
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_35MM_KEY_INT_SHUTDOWN, 0, GPIO_OUTPUT,
		      GPIO_NO_PULL, GPIO_2MA),
};*/

static void mahimahi_headset_init(void)
{
	if (is_cdma_version(system_rev))
		return;
	config_gpio_table(key_int_shutdown_gpio_table,
			ARRAY_SIZE(key_int_shutdown_gpio_table));
	gpio_set_value(MAHIMAHI_GPIO_35MM_KEY_INT_SHUTDOWN, 0);
}

static struct msm_i2c_device_platform_data msm_i2c_pdata = {
        .i2c_clock = 100000,
        .clock_strength = GPIO_8MA,
        .data_strength = GPIO_8MA,
};

static void __init msm_device_i2c_init(void)
{
        msm_i2c_gpio_init();
        msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

#define ATAG_BDADDR 0x43294329  /* mahimahi bluetooth address tag */
#define ATAG_BDADDR_SIZE 4
#define BDADDR_STR_SIZE 18

static char bdaddr[BDADDR_STR_SIZE];

module_param_string(bdaddr, bdaddr, sizeof(bdaddr), 0400);
MODULE_PARM_DESC(bdaddr, "bluetooth address");

static int __init parse_tag_bdaddr(const struct tag *tag)
{
	unsigned char *b = (unsigned char *)&tag->u;

	if (tag->hdr.size != ATAG_BDADDR_SIZE)
		return -EINVAL;

	snprintf(bdaddr, BDADDR_STR_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X",
			b[0], b[1], b[2], b[3], b[4], b[5]);

        return 0;
}

__tagtable(ATAG_BDADDR, parse_tag_bdaddr);

static struct msm_acpu_clock_platform_data mahimahi_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 245000,
	.wait_for_irq_khz	= 245000,
	.mpll_khz		= 245000
};

static struct msm_acpu_clock_platform_data mahimahi_cdma_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 235930,
	.wait_for_irq_khz	= 235930,
	.mpll_khz		= 235930
};

static unsigned mahimahi_perf_acpu_table[] = {
  245000000,
  576000000,
  998400000,
};

static struct perflock_platform_data mahimahi_perflock_data = {
  .perf_acpu_table = mahimahi_perf_acpu_table,
  .table_size = ARRAY_SIZE(mahimahi_perf_acpu_table),
};

static ssize_t mahimahi_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	if (system_rev > 2 && system_rev != 0xC0) {
		/* center: x: back: 60, menu: 172, home: 298, search 412, y: 840 */
		return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":55:840:90:60"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":172:840:125:60"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":298:840:115:60"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":412:840:95:60"
		   "\n");
	} else {
		/* center: x: home: 55, menu: 185, back: 305, search 425, y: 835 */
		return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":55:835:70:55"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":185:835:100:55"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":305:835:70:55"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":425:835:70:55"
		   "\n");
	}
}

static struct kobj_attribute mahimahi_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &mahimahi_virtual_keys_show,
};

static struct attribute *mahimahi_properties_attrs[] = {
	&mahimahi_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group mahimahi_properties_attr_group = {
	.attrs = mahimahi_properties_attrs,
};

static void mahimahi_reset(void)
{
	gpio_set_value(MAHIMAHI_GPIO_PS_HOLD, 0);
}

int mahimahi_init_mmc(int sysrev, unsigned debug_uart);

/*static const struct smd_tty_channel_desc smd_cdma_default_channels[] = {
	{ .id = 0, .name = "SMD_DS" },
	{ .id = 19, .name = "SMD_DATA3" },
	{ .id = 27, .name = "SMD_GPSNMEA" }
};*/

static void __init mahimahi_init(void)
{
	int ret;
struct kobject *properties_kobj;

	printk("mahimahi_init() revision=%d\n", system_rev);

  //android_usb_pdata.serial_number = board_serialno();

/*	if (is_cdma_version(system_rev))
		smd_set_channel_list(smd_cdma_default_channels,
					ARRAY_SIZE(smd_cdma_default_channels));*/

	msm_hw_reset_hook = mahimahi_reset;

	if (is_cdma_version(system_rev))
		msm_acpu_clock_init(&mahimahi_cdma_clock_data);
	else
		msm_acpu_clock_init(&mahimahi_clock_data);

	perflock_init(&mahimahi_perflock_data);
	
	msm_serial_debug_init(MSM_UART1_PHYS, INT_UART1,
			      &msm_device_uart1.dev, 1, MSM_GPIO_TO_INT(139));

#ifdef CONFIG_ARCH_QSD8X50
        bt_export_bd_address();
#endif
#ifdef CONFIG_SERIAL_MSM_HS
        msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
        msm_device_uart_dm1.name = "msm_serial_hs_bcm"; /* for bcm */
#endif

	config_gpio_table(misc_gpio_table, ARRAY_SIZE(misc_gpio_table));

	if (is_cdma_version(system_rev)) {
		mahimahi_flashlight_data.torch = MAHIMAHI_CDMA_GPIO_FLASHLIGHT_TORCH;
		config_gpio_table(bt_gpio_table_rev_CX, ARRAY_SIZE(bt_gpio_table_rev_CX));
	} else {
		config_gpio_table(bt_gpio_table, ARRAY_SIZE(bt_gpio_table));
	}

	gpio_request(MAHIMAHI_GPIO_TP_LS_EN, "tp_ls_en");
	gpio_direction_output(MAHIMAHI_GPIO_TP_LS_EN, 0);
	gpio_request(MAHIMAHI_GPIO_TP_EN, "tp_en");
	gpio_direction_output(MAHIMAHI_GPIO_TP_EN, 0);
	gpio_request(MAHIMAHI_GPIO_PROXIMITY_EN, "proximity_en");
	gpio_direction_output(MAHIMAHI_GPIO_PROXIMITY_EN, 1);
	gpio_request(MAHIMAHI_GPIO_COMPASS_RST_N, "compass_rst");
	gpio_direction_output(MAHIMAHI_GPIO_COMPASS_RST_N, 1);
	gpio_request(MAHIMAHI_GPIO_COMPASS_INT_N, "compass_int");
	gpio_direction_input(MAHIMAHI_GPIO_COMPASS_INT_N);

	gpio_request(MAHIMAHI_GPIO_DS2482_SLP_N, "ds2482_slp_n");
        msm_device_i2c_init();

	/* set the gpu power rail to manual mode so clk en/dis will not
	 * turn off gpu power, and hang it on resume */
	mahimahi_kgsl_power_rail_mode(0);
	mahimahi_kgsl_power(true);
        mahimahi_init_panel();

	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;

	platform_add_devices(devices, ARRAY_SIZE(devices));

	i2c_register_board_info(0, base_i2c_devices,
		ARRAY_SIZE(base_i2c_devices));

	if (system_rev == 0) {
		/* Only board after XB with Audience A1026 */
		i2c_register_board_info(0, rev0_i2c_devices,
			ARRAY_SIZE(rev0_i2c_devices));
	}

	if (system_rev > 0) {
		/* Only board after XB with Audience A1026 */
		i2c_register_board_info(0, rev1_i2c_devices,
			ARRAY_SIZE(rev1_i2c_devices));
	}

	if (is_cdma_version(system_rev)) {
		/* Only CDMA version with TI TPA2018D1 Speaker Amp. */
		i2c_register_board_info(0, rev_CX_i2c_devices,
			ARRAY_SIZE(rev_CX_i2c_devices));
		if ((system_rev & 0x0F) == 0x00) {
			a1026_data.gpio_a1026_clk = MAHIMAHI_CDMA_XA_AUD_A1026_CLK;
		} else if ((system_rev & 0x0F) >= 0x01) {
			a1026_data.gpio_a1026_wakeup = MAHIMAHI_CDMA_XB_AUD_A1026_WAKEUP;
			a1026_data.gpio_a1026_reset = MAHIMAHI_CDMA_XB_AUD_A1026_RESET;
			a1026_data.gpio_a1026_clk = MAHIMAHI_CDMA_XB_AUD_A1026_CLK;
		}
	}

        if (!opt_usb_h2w_sw) {
#ifdef CONFIG_USB_FUNCTION
                msm_register_usb_phy_init_seq(mahimahi_phy_init_seq);
                msm_add_usb_devices(mahimahi_usb_phy_reset, NULL);
#endif
#ifdef CONFIG_USB_ANDROID
                mahimahi_add_usb_devices();
#endif
        }

/*        for (ret = 0; ret < ARRAY_SIZE(base_i2c_devices); ret++) {
                if (!strcmp(base_i2c_devices[ret].type, AKM8973_I2C_NAME))
                        base_i2c_devices[ret].irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_COMPASS_INT_N);
        }
        compass_platform_data.intr = MAHIMAHI_GPIO_COMPASS_INT_N;*/

	ret = mahimahi_init_mmc(system_rev, debug_uart);
	if (ret != 0)
		pr_crit("%s: Unable to initialize MMC\n", __func__);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
					 &mahimahi_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");



        ret = platform_device_register(&android_timed_gpios);
        if (ret != 0)
                pr_err("failed to register vibrator\n");


	mahimahi_audio_init();
	mahimahi_headset_init();

/*	if (system_rev > 0)
		platform_device_register(&mahimahi_timed_gpios);
	else
		msm_init_pmic_vibrator();

	//ds2784_battery_init();*/
	headset_init();
}

static void __init mahimahi_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 2;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].node = PHYS_TO_NID(PHYS_OFFSET);
	mi->bank[0].size = MSM_EBI1_BANK0_SIZE;
	mi->bank[1].start = MSM_EBI1_BANK1_BASE;
	mi->bank[1].node = PHYS_TO_NID(MSM_EBI1_BANK1_BASE);
	mi->bank[1].size = MSM_EBI1_BANK1_SIZE;
}

static void __init mahimahi_map_io(void)
{
        msm_map_common_io();
        msm_clock_init();
}

extern struct sys_timer msm_timer;

MACHINE_START(MAHIMAHI, "mahimahi")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x20000100,
	.fixup		= mahimahi_fixup,
	.map_io		= mahimahi_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= mahimahi_init,
	.timer		= &msm_timer,
MACHINE_END
