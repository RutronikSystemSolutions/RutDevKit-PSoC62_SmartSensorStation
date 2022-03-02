/*
 * bmp390.c
 *
 *  Created on: 2021-11-29
 *      Author: GDR
 */

#include "bmp390_app.h"

/*BMP390 Global Variables*/
struct bmp3_dev dev;
struct bmp3_data bmp_data = { 0 };
struct bmp3_settings settings = { 0 };
struct bmp3_status bmp_status = { { 0 } };

cy_rslt_t bmp390_app_init(void)
{
    int8_t rslt = 0;
    uint32_t settings_sel;

    rslt = bmp3_interface_init(&dev, BMP3_I2C_INTF);
    if(rslt != BMP3_OK)
    {
    	bmp3_check_rslt("bmp3_interface_init", rslt);
    	return 1;
    }

    rslt = bmp3_init(&dev);
    if(rslt != BMP3_OK)
    {
    	bmp3_check_rslt("bmp3_init", rslt);
    	return 1;
    }

    settings.int_settings.drdy_en = BMP3_DISABLE;
    settings.int_settings.latch = BMP3_INT_PIN_LATCH;
    settings.int_settings.level = BMP3_INT_PIN_ACTIVE_HIGH;
    settings.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL;

    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.odr = BMP3_ODR_12_5_HZ;
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_127;

    settings_sel = BMP3_SEL_PRESS_EN|BMP3_SEL_TEMP_EN |BMP3_SEL_PRESS_OS|BMP3_SEL_TEMP_OS|BMP3_SEL_ODR|BMP3_SEL_DRDY_EN|BMP3_SEL_LATCH|BMP3_SEL_LEVEL|BMP3_SEL_OUTPUT_MODE|BMP3_SEL_IIR_FILTER;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    if(rslt != BMP3_OK)
    {
    	bmp3_check_rslt("bmp3_set_sensor_settings", rslt);
    	return 1;
    }

    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &dev);
    if(rslt != BMP3_OK)
    {
    	bmp3_check_rslt("bmp3_set_op_mode", rslt);
    	return 1;
    }

    return CY_RSLT_SUCCESS;
}


