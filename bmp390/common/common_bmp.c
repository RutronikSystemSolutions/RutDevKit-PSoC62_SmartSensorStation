/**\
 * Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include "common_bmp.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "bmp3.h"

#ifdef FREERTOS_APP
/*FreeRTOS Header Files*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sensors_task.h"
#endif

/* Variable to store the device address */
static uint8_t dev_addr;

extern cyhal_i2c_t I2C_scb3;

/*!
 * I2C read function map to RutDevKit-PSoC62 platform
 */
BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	cy_rslt_t result;
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

#ifndef FREERTOS_APP
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)dev_addr, &reg_addr, 1, 100, false );
    if (result != CY_RSLT_SUCCESS)
    {
    	return BMP3_E_COMM_FAIL;
    }

	result = (int8_t)cyhal_i2c_master_read(&I2C_scb3,(uint16_t)dev_addr, reg_data, (uint16_t)len, 100, true);
	if (result != CY_RSLT_SUCCESS)
	{
		 return BMP3_E_COMM_FAIL;
	}
#else
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)dev_addr, &reg_addr, 1, 100, false );
    if (result != CY_RSLT_SUCCESS)
    {
    	xSemaphoreGive(i2c_mutex);
    	return BMP3_E_COMM_FAIL;
    }

	result = (int8_t)cyhal_i2c_master_read(&I2C_scb3,(uint16_t)dev_addr, reg_data, (uint16_t)len, 100, true);
	if (result != CY_RSLT_SUCCESS)
	{
		xSemaphoreGive(i2c_mutex);
		 return BMP3_E_COMM_FAIL;
	}
	xSemaphoreGive(i2c_mutex);
#endif

	 return BMP3_OK;
}

/*!
 * I2C write function map to RutDevKit-PSoC62 platform
 */
BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	cy_rslt_t result;
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    uint8_t* i2c_data = NULL;

	/*Allocate buffer for transmission*/
    i2c_data = malloc(len+1);
    if(i2c_data == NULL)
    {
    	return BMP3_E_NULL_PTR;
    }

    /*Copy register address and all the data*/
    i2c_data[0] = reg_addr;
    memcpy(&i2c_data[1], reg_data, len);

#ifndef FREERTOS_APP
    /*Write data to I2C*/
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)dev_addr, i2c_data, len+1, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	free(i2c_data);
    	return BMP3_E_COMM_FAIL;
    }
#else
    /*Write data to I2C*/
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)dev_addr, i2c_data, len+1, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	xSemaphoreGive(i2c_mutex);
    	free(i2c_data);
    	return BMP3_E_COMM_FAIL;
    }
    xSemaphoreGive(i2c_mutex);
#endif

    free(i2c_data);
    return BMP3_OK;
}

/*!
 * SPI read function map to RutDevKit-PSoC62 platform
 */
BMP3_INTF_RET_TYPE bmp3_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    return BMP3_E_COMM_FAIL;
}

/*!
 * SPI write function map to RutDevKit-PSoC62 platform
 */
BMP3_INTF_RET_TYPE bmp3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    return BMP3_E_COMM_FAIL;
}

/*!
 * Delay function map to RutDevKit-PSoC62 platform
 */
void bmp3_delay_us(uint32_t period, void *intf_ptr)
{
#ifndef FREERTOS_APP
	if(period <= 0xFFFF)
	{
		CyDelayUs(period);
	}
	else
	{
		CyDelay(period/1000);
	}
#else
	if(period <= 1000)
	{
		CyDelayUs(period);
	}
	else
	{
		vTaskDelay(pdMS_TO_TICKS(period/1000));
	}
#endif
}

void bmp3_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMP3_OK:

            /* Do nothing */
            break;
        case BMP3_E_NULL_PTR:
            printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMP3_E_COMM_FAIL:
            printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMP3_E_INVALID_LEN:
            printf("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BMP3_E_DEV_NOT_FOUND:
            printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMP3_E_CONFIGURATION_ERR:
            printf("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
            break;
        case BMP3_W_SENSOR_NOT_ENABLED:
            printf("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
            break;
        case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
            printf("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, rslt);
            break;
        default:
            printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf)
{
	if (bmp3 != NULL)
	{
	    /* Bus configuration : I2C */
	    if (intf == BMP3_I2C_INTF)
	    {
	        printf("BMP3xx I2C Interface Initialized.\r\n");
	        dev_addr = BMP3_ADDR_I2C_SEC;
	        bmp3->read = bmp3_i2c_read;
	        bmp3->write = bmp3_i2c_write;
	        bmp3->intf = BMP3_I2C_INTF;
	    }
	    /* Bus configuration : SPI */
	    else if (intf == BMP3_SPI_INTF)
	    {
	        printf("SPI Interface not supported.\r\n");
	        return BMP3_E_CONFIGURATION_ERR;
	    }

	    bmp3->delay_us = bmp3_delay_us;
	    bmp3->intf_ptr = &dev_addr;
	}
	else
	{
		return BMP3_E_NULL_PTR;
	}

    return BMP3_OK;
}

void bmp3_coines_deinit(void)
{
	return;
}
