/*
 * pi4ioe_app.c
 *
 *  Created on: 2021-12-01
 *      Author: GDR
 */

#include "pi4ioe_app.h"

cy_rslt_t pi4ioe_app_init(void)
{
	PI4IO_RET_TYPE result;
	uint16_t pins;

	/*Configure the polarity of all IOs*/
	result = pi4io_write_config_pol(PI4IO_DEV_ADDRESS, PI4IOE_POL_CONFIG);
    if(result != PI4IO_OK)
    {
    	return 1;
    }

	/*Configure the direction of all IOs*/
	result = pi4io_write_config_dir(PI4IO_DEV_ADDRESS, PI4IOE_DIR_CONFIG);
    if(result != PI4IO_OK)
    {
    	return 1;
    }

	/*Configure the outputs of all IOs*/
	result = pi4io_pins_write(PI4IO_DEV_ADDRESS, PI4IOE_OUT_CONFIG);
    if(result != PI4IO_OK)
    {
    	return 1;
    }

    /*Reading all pins clears the interrupt that might be triggered*/
    pi4io_pins_read(PI4IO_DEV_ADDRESS, &pins);

    return CY_RSLT_SUCCESS;
}
