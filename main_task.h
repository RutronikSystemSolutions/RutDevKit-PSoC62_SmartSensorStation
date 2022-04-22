/*
 * main_task.h
 *
 *  Created on: 2021-12-06
 *      Author: GDR
 */

#ifndef MAIN_TASK_H_
#define MAIN_TASK_H_

#define GREETING_PHRASE			0
#define CO2_NORMAL_PHRASE		1
#define CO2_MEDIUM_PHRASE		2
#define CO2_MAXIMUM_PHRASE		3
#define MOTION_DETECTED			6
#define VOC_NORMAL_PHRASE		13
#define VOC_MEDIUM_PHRASE		14
#define VOC_HIGH_PHRASE			15
#define TEMP_NORMAL_PHRASE		8
#define TEMP_LOW_PHRASE			7
#define TEMP_HIGH_PHRASE		9
#define HUMID_NORMAL_PHRASE		11
#define HUMID_LOW_PHRASE		10
#define HUMID_HIGH_PHRASE		12

#define LVL_NORMAL				0
#define LVL_MEDIUM				1000
#define LVL_MAXIMUM				2000

#define TEMP_LVL_NORMAL 		18
#define TEMP_LVL_HIGH			26
#define HUM_LVL_NORMAL			35
#define HUM_LVL_HIGH			60
#define VOC_LVL_NORMAL			0
#define VOC_LVL_MEDIUM			150
#define VOC_LVL_HIGH			350

#define CO2_MEDIUM_REPEAT		60000U
#define CO2_MAXIMUM_REPEAT		30000U
#define SPEECH_ENGINE_CHECK		5000U
#define SPEECH_ENGINE_INIT		10000U

#define ACTIVATE_FORM0_MSEC		300000U
#define TEMP_ALARM_REPEAT		300000U
#define VOC_ALARM_REPEAT		300000U
#define HUMID_ALARM_REPEAT		300000U

typedef enum co2status
{
	CO2_NORMAL,
	CO2_MEDIUM,
	CO2_MAXIMUM,
	CO2_UNDEFINED
}co2status_t;

typedef enum temp_status
{
	TEMP_NORMAL,
	TEMP_LOW,
	TEMP_HIGH,
	TEMP_UNDEFINED
}temp_status_t;

typedef enum humid_status
{
	HUMID_NORMAL,
	HUMID_LOW,
	HUMID_HIGH,
	HUMID_UNDEFINED
}humid_status_t;

typedef enum voc_status
{
	VOC_NORMAL,
	VOC_MEDIUM,
	VOC_HIGH,
	VOC_UNDEFINED
}voc_status_t;

extern TaskHandle_t main_task_handle;
void main_task(void *param);

#endif /* MAIN_TASK_H_ */
