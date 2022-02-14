/*
 * pi4ioe_app.h
 *
 *  Created on: 2021-12-01
 *      Author: GDR
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "pi4ioe5v9535.h"

#ifndef PI4IOE5V9535_PI4IOE_APP_H_
#define PI4IOE5V9535_PI4IOE_APP_H_

#define PI4IOE_POL_CONFIG	UINT16_C(0x0000)
#define PI4IOE_DIR_CONFIG	UINT16_C(0x001C)
#define PI4IOE_OUT_CONFIG	UINT16_C(0x0760)

/*Function prototypes*/
cy_rslt_t pi4ioe_app_init(void);

#endif /* PI4IOE5V9535_PI4IOE_APP_H_ */
