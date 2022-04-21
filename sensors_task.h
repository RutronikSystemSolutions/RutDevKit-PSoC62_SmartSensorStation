/*
 * sensors_task.h
 *
 *  Created on: 2021-12-06
 *      Author: GDR
 */

#ifndef SENSORS_TASK_H_
#define SENSORS_TASK_H_

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

typedef struct app_sensor_data
{
	double bmp_temperature;
	double bmp_pressure;
	double bmp_altitude;
	double bmp_altitude_offset;

	int16_t bmi_acc_x;
	int16_t bmi_acc_y;
	int16_t bmi_acc_z;

	int16_t bmi_gyr_x;
	int16_t bmi_gyr_y;
	int16_t bmi_gyr_z;

	float bme_temperature;
	float bme_pressure;
	float bme_humidity;
	float bme_gas_resistance;
	uint8_t bme_gas_index;

	uint16_t sgp_sraw_voc;
	int32_t sgp_voc_index;

	int32_t sht_temperature;
	int32_t sht_humidity;

	uint16_t scd_co2;
	int32_t scd_temperature;
	int32_t scd_humidity;

	uint16_t pas_co2;

	uint8_t slider_pos;

}sensor_data_t;

extern TaskHandle_t env_sensors_task_handle;
extern SemaphoreHandle_t i2c_mutex;
extern sensor_data_t sensor_data_storage;
extern _Bool motion;

void env_sensors_task(void *param);
void motion_sensors_task(void *param);

#endif /* SENSORS_TASK_H_ */
