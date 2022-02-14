/*
 * pi4ioe5v9535.h
 *
 *  Created on: 2021-12-01
 *      Author: GDR
 */

/*RutDevKit-PSoC62 platform includes*/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#ifndef PI4IOE5V9535_PI4IOE5V9535_H_
#define PI4IOE5V9535_PI4IOE5V9535_H_

#define PI4IO_DEV_ADDRESS						UINT8_C(0x20)

#ifndef PI4IO_RET_TYPE
#define PI4IO_RET_TYPE                      	int8_t
#endif

#define PI4IO_INPUT_0_CMD						UINT8_C(0)
#define PI4IO_INPUT_1_CMD						UINT8_C(1)
#define PI4IO_OUTPUT_0_CMD						UINT8_C(2)
#define PI4IO_OUTPUT_1_CMD						UINT8_C(3)
#define PI4IO_POLARITY_0_CMD					UINT8_C(4)
#define PI4IO_POLARITY_1_CMD					UINT8_C(5)
#define PI4IO_CONFIG_0_CMD						UINT8_C(6)
#define PI4IO_CONFIG_1_CMD						UINT8_C(7)

#define PI4IO_OK                                INT8_C(0)
#define PI4IO_COM_ERR                        	INT8_C(-1)

PI4IO_RET_TYPE pi4io_pins_write(uint8_t address, uint16_t pins);
PI4IO_RET_TYPE pi4io_pins_read(uint8_t address, uint16_t* pins);
PI4IO_RET_TYPE pi4io_read_config_dir(uint8_t address, uint16_t* pins);
PI4IO_RET_TYPE pi4io_read_config_pol(uint8_t address, uint16_t* pins);
PI4IO_RET_TYPE pi4io_write_config_dir(uint8_t address, uint16_t pins);
PI4IO_RET_TYPE pi4io_write_config_pol(uint8_t address, uint16_t pins);

#endif /* PI4IOE5V9535_PI4IOE5V9535_H_ */
