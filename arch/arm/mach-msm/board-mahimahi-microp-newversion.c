/* board-mahimahi-microp.c
 * Copyright (C) 2009 Google.
 * Copyright (C) 2009 HTC Corporation.
 *
 * The Microp on mahimahi is an i2c device that supports
 * the following functions
 *   - LEDs (Green, Amber, Jogball backlight)
 *   - Lightsensor
 *   - Headset remotekeys
 *   - G-sensor
 *   - Interrupts
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
#ifdef CONFIG_MICROP_COMMON
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <mach/atmega_microp.h>
#include <mach/drv_callback.h>
#include <mach/htc_headset_mgr.h>

#include "board-bravoc.h"

#ifdef CONFIG_HTC_HEADSET
#define notify_35mm_headset_insert(insert) \
	htc_35mm_remote_notify_insert_ext_headset(insert)
#else
#define notify_35mm_headset_insert(insert) do {} while (0)
#endif

static int microp_function_initialize(struct i2c_client *client)
{
	struct microp_i2c_client_data *cdata;
	uint8_t data[20];
	uint16_t stat, interrupts = 0;
	int i;
	int ret;
	struct led_classdev *led_cdev;

	cdata = i2c_get_clientdata(client);

	/* Light Sensor */
	if (als_kadc >> 16 == ALS_CALIBRATED)
		cdata->als_kadc = als_kadc & 0xFFFF;
	else {
		cdata->als_kadc = 0;
		pr_info("%s: no ALS calibrated\n", __func__);
	}

	if (cdata->als_kadc && golden_adc) {
		cdata->als_kadc =
			(cdata->als_kadc > 0 && cdata->als_kadc < 0x400)
			? cdata->als_kadc : golden_adc;
		cdata->als_gadc =
			(golden_adc > 0)
			? golden_adc : cdata->als_kadc;
	} else {
		cdata->als_kadc = 1;
		cdata->als_gadc = 1;
	}
	pr_info("%s: als_kadc=0x%x, als_gadc=0x%x\n",
		__func__, cdata->als_kadc, cdata->als_gadc);

	for (i = 0; i < 10; i++) {
		data[i] = (uint8_t)(lsensor_adc_table[i]
			* cdata->als_kadc / cdata->als_gadc >> 8);
		data[i + 10] = (uint8_t)(lsensor_adc_table[i]
			* cdata->als_kadc / cdata->als_gadc);
	}
	ret = i2c_write_block(client, MICROP_I2C_WCMD_ADC_TABLE, data, 20);
	if (ret)
		goto exit;

	ret = gpio_request(MAHIMAHI_GPIO_LS_EN_N, "microp_i2c");
	if (ret < 0) {
		dev_err(&client->dev, "failed on request gpio ls_on\n");
		goto exit;
	}
	ret = gpio_direction_output(MAHIMAHI_GPIO_LS_EN_N, 0);
	if (ret < 0) {
		dev_err(&client->dev, "failed on gpio_direction_output"
				"ls_on\n");
		goto err_gpio_ls;
	}
	cdata->light_sensor_enabled = 1;

	/* Headset */
	for (i = 0; i < 6; i++) {
		data[i] = (uint8_t)(remote_key_adc_table[i] >> 8);
		data[i + 6] = (uint8_t)(remote_key_adc_table[i]);
	}
	ret = i2c_write_block(client,
		MICROP_I2C_WCMD_REMOTEKEY_TABLE, data, 12);
	if (ret)
		goto exit;

	INIT_DELAYED_WORK(
		&cdata->hpin_debounce_work, hpin_debounce_do_work);
	INIT_DELAYED_WORK(
		&cdata->ls_read_work, ls_read_do_work);

	/* SD Card */
	interrupts |= IRQ_SDCARD;

	/* set LED initial state */
	for (i = 0; i < BLUE_LED; i++) {
		led_cdev = &cdata->leds[i].ldev;
		microp_i2c_write_led_mode(client, led_cdev, 0, 0xffff);
	}

	/* enable the interrupts */
	ret = microp_interrupt_enable(client, interrupts);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to enable gpi irqs\n",
			__func__);
		goto err_irq_en;
	}

	microp_read_gpi_status(client, &stat);
	mahimahi_microp_sdslot_update_status(stat);

	return 0;

err_irq_en:
err_gpio_ls:
	gpio_free(MAHIMAHI_GPIO_LS_EN_N);
exit:
	return ret;
}

static struct microp_ops ops = {
	.init_microp_func = microp_function_initialize,
};

void __init mahimahi_microp_init(void)
{
	microp_register_ops(&ops);
}

#endif
