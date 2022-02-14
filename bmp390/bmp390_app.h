/*
 * bmp390_app.h
 *
 *  Created on: 2021-11-29
 *      Author: GDR
 */


#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "common_bmp.h"
#include "bmp3.h"

#ifndef BMP390_BMP390_H_
#define BMP390_BMP390_H_

/*Function prototypes*/
cy_rslt_t bmp390_app_init(void);

/*BMP390 Global Variables*/
extern struct bmp3_dev dev;
extern struct bmp3_data bmp_data;
extern struct bmp3_settings settings;
extern struct bmp3_status bmp_status;

#endif /* BMP390_BMP390_H_ */
