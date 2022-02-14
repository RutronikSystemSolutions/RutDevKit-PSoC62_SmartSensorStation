/*
 * pi4ioe5v9535.c
 *
 *  Created on: 2021-12-01
 *      Author: GDR
 */


#include "pi4ioe5v9535.h"

extern cyhal_i2c_t I2C_scb3;

PI4IO_RET_TYPE pi4io_pins_write(uint8_t address, uint16_t pins)
{
	cy_rslt_t result;
	uint8_t i2c_data[3];
	i2c_data[0] = PI4IO_OUTPUT_0_CMD;
    i2c_data[1] = pins;
    i2c_data[2] = pins >> 8;

    result = cyhal_i2c_master_write(&I2C_scb3,(uint16_t)address, i2c_data, 3, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	return PI4IO_COM_ERR;
    }

    return PI4IO_OK;
}

PI4IO_RET_TYPE pi4io_pins_read(uint8_t address, uint16_t* pins)
{
	cy_rslt_t result;
	uint8_t i2c_data = PI4IO_INPUT_0_CMD;
	uint8_t tx_data[3];

    result = cyhal_i2c_master_write(&I2C_scb3,(uint16_t)address, &i2c_data, 1, 100, false);
    if (result != CY_RSLT_SUCCESS)
    {
    	return PI4IO_COM_ERR;
    }

    result = cyhal_i2c_master_read(&I2C_scb3,(uint16_t)address, (uint8_t*)pins, 2, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	return PI4IO_COM_ERR;
    }

	tx_data[0] = PI4IO_OUTPUT_0_CMD;
	tx_data[1] = *pins;
	tx_data[2] = *pins >> 8;
    result = cyhal_i2c_master_write(&I2C_scb3,(uint16_t)address, tx_data, 3, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	return PI4IO_COM_ERR;
    }

	return PI4IO_OK;
}

PI4IO_RET_TYPE pi4io_read_config_dir(uint8_t address, uint16_t* pins)
{
	cy_rslt_t result;
	uint8_t i2c_data = PI4IO_CONFIG_0_CMD;

    result = cyhal_i2c_master_write(&I2C_scb3,(uint16_t)address, &i2c_data, 1, 100, false);
    if (result != CY_RSLT_SUCCESS)
    {
    	return PI4IO_COM_ERR;
    }

    result = cyhal_i2c_master_read(&I2C_scb3,(uint16_t)address, (uint8_t*)pins, 2, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	return PI4IO_COM_ERR;
    }

	return PI4IO_OK;
}

PI4IO_RET_TYPE pi4io_read_config_pol(uint8_t address, uint16_t* pins)
{
	cy_rslt_t result;
	uint8_t i2c_data = PI4IO_POLARITY_0_CMD;

    result = cyhal_i2c_master_write(&I2C_scb3,(uint16_t)address, &i2c_data, 1, 100, false);
    if (result != CY_RSLT_SUCCESS)
    {
    	return PI4IO_COM_ERR;
    }

    result = cyhal_i2c_master_read(&I2C_scb3,(uint16_t)address, (uint8_t*)pins, 2, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	return PI4IO_COM_ERR;
    }

	return PI4IO_OK;
}

PI4IO_RET_TYPE pi4io_write_config_dir(uint8_t address, uint16_t pins)
{
	cy_rslt_t result;
	uint8_t i2c_data[3];
	i2c_data[0] = PI4IO_CONFIG_0_CMD;
    i2c_data[1] = pins;
    i2c_data[2] = pins >> 8;

    result = cyhal_i2c_master_write(&I2C_scb3,(uint16_t)address, i2c_data, 3, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	return PI4IO_COM_ERR;
    }

    return PI4IO_OK;
}

PI4IO_RET_TYPE pi4io_write_config_pol(uint8_t address, uint16_t pins)
{
	cy_rslt_t result;
	uint8_t i2c_data[3];
	i2c_data[0] = PI4IO_POLARITY_0_CMD;
    i2c_data[1] = pins;
    i2c_data[2] = pins >> 8;

    result = cyhal_i2c_master_write(&I2C_scb3,(uint16_t)address, i2c_data, 3, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	return PI4IO_COM_ERR;
    }

    return PI4IO_OK;
}

